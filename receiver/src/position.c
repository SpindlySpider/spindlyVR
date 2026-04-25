#include "data_handle.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include <math.h>
#include <stdint.h>

K_MSGQ_DEFINE(positioning_queue, sizeof(struct solver_packet_t), 10, 4);
#define USE_ROTATION_B 0  // 0 = use A, 1 = use B
#define DEBUG_ROTATIONS 0 // set to 0 once chosen
#define DEBUG_TX_DISABLED 0

#define FREEZE_RX_QUAT_FOR_TEST 0

static struct data_container_t local_data_tx = {0};
static struct data_container_t local_data_rx = {0};

typedef struct {
  float x;
  float y;
  float z;
} vec3_t;

typedef struct {
  float w;
  float x;
  float y;
  float z;
} quat_t;
// Empirical calibration value. Recalibrate this after raw tracking works.
static float position_K = 88.46078431372548f;

// Simple display smoothing only. Safer than Kalman while debugging.
static vec3_t pos_filt = {0.0f, 0.0f, 0.0f};
static int pos_filt_initialized = 0;

static int have_prev_B = 0;
static vec3_t prev_B = {0.0f, 0.0f, 0.0f};

static bool have_frozen_q = false;
static quat_t frozen_q_rx;

static vec3_t stabilize_global_phase_flip(vec3_t B) {
  /*
   * Emergency front-volume demo assumption:
   * We expect the valid front-hemisphere solution to have B.z positive.
   * If your current demo volume is different, remove this block.
   */
  if (B.z < 0.0f) {
    B.x = -B.x;
    B.y = -B.y;
    B.z = -B.z;
  }

  /*
   * Reject global 180-degree phase flips.
   * Consecutive magnetic vectors should not suddenly point in the exact
   * opposite direction unless phase sync glitched.
   */
  if (have_prev_B) {
    float dot = (B.x * prev_B.x) + (B.y * prev_B.y) + (B.z * prev_B.z);

    if (dot < 0.0f) {
      B.x = -B.x;
      B.y = -B.y;
      B.z = -B.z;
      printk("GLOBAL PHASE FLIP CORRECTED\n");
    }
  }

  prev_B = B;
  have_prev_B = 1;

  return B;
}

static vec3_t rx_coil_to_imu_frame(vec3_t b) {
  /*
   * Temporary identity mapping.
   * Replace this with your measured/sign/permutation/alignment correction.
   *
   * Do NOT duplicate GAIN_X/Y/Z here if you already applied them in adc thread.
   */
  return (vec3_t){
      .x = b.x,
      .y = b.y,
      .z = b.z,
  };
}

static vec3_t stabilize_for_front_demo(vec3_t B) {
  /*
   * Demo mode: force front hemisphere.
   * This prevents the bad z≈0.16 branch.
   */
  B.z = fabsf(B.z);
  return B;
}

int solve_position(vec3_t B, vec3_t *pos) {
  float bx = B.x;
  float by = B.y;
  float bz = B.z;

  float bxy = sqrtf((bx * bx) + (by * by));

  // The closed-form solver is unstable near the Tx axis.
  if (bxy < 1e-6f) {
    return -1;
  }

  float c1 = bz / bxy;
  float c2 = ((3.0f * c1) + sqrtf((9.0f * c1 * c1) + 8.0f)) / 4.0f;

  if (!isfinite(c2) || c2 <= 0.0f) {
    return -2;
  }

  float denom = powf(1.0f + (c2 * c2), 2.5f) * bxy;

  if (!isfinite(denom) || fabsf(denom) < 1e-12f) {
    return -3;
  }

  float rho_cubed = (position_K * c2) / denom;

  if (!isfinite(rho_cubed) || rho_cubed <= 0.0f) {
    return -4;
  }

  float rho = cbrtf(rho_cubed);

  pos->x = rho * bx / bxy;
  pos->y = rho * by / bxy;
  pos->z = c2 * rho;

  if (!isfinite(pos->x) || !isfinite(pos->y) || !isfinite(pos->z)) {
    return -5;
  }

  return 0;
}

static quat_t quat_conj(quat_t q) {
  return (quat_t){
      .w = q.w,
      .x = -q.x,
      .y = -q.y,
      .z = -q.z,
  };
}

static quat_t quat_mul(quat_t a, quat_t b) {
  return (quat_t){
      .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

static quat_t quat_normalize(quat_t q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

  if (n < 1e-6f) {
    return (quat_t){.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
  }

  float inv = 1.0f / n;
  return (quat_t){
      .w = q.w * inv,
      .x = q.x * inv,
      .y = q.y * inv,
      .z = q.z * inv,
  };
}

static vec3_t quat_rotate_vec(quat_t q, vec3_t v) {
  q = quat_normalize(q);

  quat_t p = {
      .w = 0.0f,
      .x = v.x,
      .y = v.y,
      .z = v.z,
  };

  quat_t r = quat_mul(quat_mul(q, p), quat_conj(q));

  return (vec3_t){
      .x = r.x,
      .y = r.y,
      .z = r.z,
  };
}

static vec3_t smooth_position(vec3_t raw) {
  const float alpha = 0.15f;

  if (!pos_filt_initialized) {
    pos_filt = raw;
    pos_filt_initialized = 1;
    return pos_filt;
  }

  pos_filt.x = (1.0f - alpha) * pos_filt.x + alpha * raw.x;
  pos_filt.y = (1.0f - alpha) * pos_filt.y + alpha * raw.y;
  pos_filt.z = (1.0f - alpha) * pos_filt.z + alpha * raw.z;

  return pos_filt;
}

static void print_solve_candidate(const char *name, vec3_t B) {
  vec3_t p;
  int err = solve_position(B, &p);

  float mag = sqrtf(B.x * B.x + B.y * B.y + B.z * B.z);
  float bxy = sqrtf(B.x * B.x + B.y * B.y);

  if (err == 0) {
    printk("%s B %.2f %.2f %.2f | mag %.2f | bxy %.2f | p %.3f %.3f %.3f\n",
           name, B.x, B.y, B.z, mag, bxy, p.x, p.y, p.z);
  } else {
    printk("%s FAIL err=%d | B %.2f %.2f %.2f | mag %.2f | bxy %.2f\n", name,
           err, B.x, B.y, B.z, mag, bxy);
  }
}

void start_positioning_thread(void *, void *, void *) {
  struct solver_packet_t incoming_data;

  static const vec3_t rx_body_to_coil = {
      .x = -0.045f, // -4.5 cm along RX body X
      .y = 0.0f,
      .z = 0.008f, // +0.8 cm along RX body Z
  };

  while (1) {
    k_msgq_get(&positioning_queue, &incoming_data, K_FOREVER);

    vec3_t b_rx_coil = {
        .x = incoming_data.bx,
        .y = incoming_data.by,
        .z = incoming_data.bz,
    };

    /*
     * RX quaternion comes from this board's IMU.
     * TX quaternion is already passed through the solver packet.
     */
    read_data(&local_data_rx);

    quat_t q_tx_frame = {
        .w = incoming_data.q0,
        .x = incoming_data.q1,
        .y = incoming_data.q2,
        .z = incoming_data.q3,
    };

    quat_t q_rx_frame = {
        .w = local_data_rx.q0,
        .x = local_data_rx.q1,
        .y = local_data_rx.q2,
        .z = local_data_rx.q3,
    };

    q_tx_frame = quat_normalize(q_tx_frame);
    q_rx_frame = quat_normalize(q_rx_frame);

#if FREEZE_RX_QUAT_FOR_TEST
    if (!have_frozen_q) {
      frozen_q_rx = q_rx_frame;
      have_frozen_q = true;
    }
    q_rx_frame = frozen_q_rx;
#endif

    /*
     * Convert magnetic vector from RX coil wiring frame into RX IMU/body frame.
     */
    vec3_t b_rx_body = rx_coil_to_imu_frame(b_rx_coil);

    float b_rx_mag =
        sqrtf(b_rx_body.x * b_rx_body.x + b_rx_body.y * b_rx_body.y +
              b_rx_body.z * b_rx_body.z);

#if DEBUG_TX_DISABLED
    printk("RXDBG Brx %.3f %.3f %.3f | |Brx| %.3f | "
           "q_rx %.4f %.4f %.4f %.4f | "
           "q_tx %.4f %.4f %.4f %.4f\n",
           b_rx_body.x, b_rx_body.y, b_rx_body.z, b_rx_mag, q_rx_frame.w,
           q_rx_frame.x, q_rx_frame.y, q_rx_frame.z, q_tx_frame.w, q_tx_frame.x,
           q_tx_frame.y, q_tx_frame.z);

    continue;
#endif

    if (b_rx_mag < 20.0f || b_rx_mag > 350.0f || !isfinite(b_rx_mag) ||
        !isfinite(b_rx_body.x) || !isfinite(b_rx_body.y) ||
        !isfinite(b_rx_body.z)) {
      printk("BAD_BRX Brx %.3f %.3f %.3f | |Brx| %.3f\n", b_rx_body.x,
             b_rx_body.y, b_rx_body.z, b_rx_mag);
      continue;
    }

    /*
     * Candidate A:
     * RX body frame -> TX body frame
     * This is the one I would use first based on your latest logs.
     */
    quat_t q_a = quat_mul(quat_conj(q_tx_frame), q_rx_frame);
    vec3_t b_a = quat_rotate_vec(q_a, b_rx_body);

    /*
     * Candidate B:
     * Alternative convention. Keep only for debugging.
     */
    quat_t q_b = quat_mul(q_tx_frame, quat_conj(q_rx_frame));
    vec3_t b_b = quat_rotate_vec(q_b, b_rx_body);

#if DEBUG_ROTATIONS
    float mag_a = sqrtf(b_a.x * b_a.x + b_a.y * b_a.y + b_a.z * b_a.z);
    float mag_b = sqrtf(b_b.x * b_b.x + b_b.y * b_b.y + b_b.z * b_b.z);

    printk("YAWDBG Brx %.2f %.2f %.2f | "
           "A %.2f %.2f %.2f | |A| %.2f | "
           "B %.2f %.2f %.2f | |B| %.2f\n",
           b_rx_body.x, b_rx_body.y, b_rx_body.z, b_a.x, b_a.y, b_a.z, mag_a,
           b_b.x, b_b.y, b_b.z, mag_b);

    continue;
#endif

    quat_t q_rx_to_tx;
    vec3_t b_tx;

#if USE_ROTATION_B
    q_rx_to_tx = q_b;
    b_tx = b_b;
#else
    q_rx_to_tx = q_a;
    b_tx = b_a;
#endif

    // printk("DATA,%.6f,%.6f,%.6f,"
    //        "%.6f,%.6f,%.6f,%.6f,"
    //        "%.6f,%.6f,%.6f,%.6f\n",
    //        b_rx_coil.x, b_rx_coil.y, b_rx_coil.z, q_tx_frame.w,
    //        q_tx_frame.x, q_tx_frame.y, q_tx_frame.z, q_rx_frame.w,
    //        q_rx_frame.x, q_rx_frame.y, q_rx_frame.z);
    //
    //

    // printk("DATA,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", b_rx_coil.x,
    //        b_rx_coil.y, b_rx_coil.z, q_rx_frame.w, q_rx_frame.x,
    //        q_rx_frame.y, q_rx_frame.z);

    /*
     * Rotate RX body-to-coil offset into TX frame.
     * This gives the coil offset expressed in the same frame as the solved
     * position.
     */
    vec3_t coil_offset_tx = quat_rotate_vec(q_rx_to_tx, rx_body_to_coil);

    /*
     * Solve position of the RX coil center.
     */
    vec3_t pos_coil;
    int err = solve_position(b_tx, &pos_coil);

    b_rx_mag = sqrtf(b_rx_body.x * b_rx_body.x + b_rx_body.y * b_rx_body.y +
                     b_rx_body.z * b_rx_body.z);

    float b_tx_mag = sqrtf(b_tx.x * b_tx.x + b_tx.y * b_tx.y + b_tx.z * b_tx.z);

    float bxy = sqrtf(b_tx.x * b_tx.x + b_tx.y * b_tx.y);

    if (err != 0) {
      printk("SOLVE_FAIL err=%d | Brx %.3f %.3f %.3f | Btx %.3f %.3f %.3f | "
             "bxy %.3f\n",
             err, b_rx_body.x, b_rx_body.y, b_rx_body.z, b_tx.x, b_tx.y, b_tx.z,
             bxy);
      continue;
    }

    /*
     * Convert coil-center position to RX body/IMU-origin position.
     */
    vec3_t pos_body_raw = {
        .x = pos_coil.x - coil_offset_tx.x,
        .y = pos_coil.y - coil_offset_tx.y,
        .z = pos_coil.z - coil_offset_tx.z,
    };

    /*
     * Smooth body position, not coil position.
     */
    vec3_t pos_body = smooth_position(pos_body_raw);

    // printk(
    //     "coil %.3f %.3f %.3f | off %.3f %.3f %.3f | body_raw %.3f %.3f
    //     %.3f\n", pos_coil.x, pos_coil.y, pos_coil.z, coil_offset_tx.x,
    //     coil_offset_tx.y, coil_offset_tx.z, pos_body_raw.x, pos_body_raw.y,
    //     pos_body_raw.z);

    printk("TEST Brx %.2f %.2f %.2f | |Brx| %.2f | "
           "Btx %.2f %.2f %.2f | |Btx| %.2f | "
           "coil %.3f %.3f %.3f\n",
           b_rx_body.x, b_rx_body.y, b_rx_body.z, b_rx_mag, b_tx.x, b_tx.y,
           b_tx.z, b_tx_mag, pos_coil.x, pos_coil.y, pos_coil.z);

    // printk("Btx %.3f %.3f %.3f | mag %.3f %.3f | coil %.3f %.3f %.3f | off
    // "
    //        "%.3f %.3f %.3f | body %.3f %.3f %.3f\n",
    //        b_tx.x, b_tx.y, b_tx.z, b_rx_mag, b_tx_mag, pos_coil.x,
    //        pos_coil.y, pos_coil.z, coil_offset_tx.x, coil_offset_tx.y,
    //        coil_offset_tx.z, pos_body.x, pos_body.y, pos_body.z);
  }
}
