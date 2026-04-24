#include "data_handle.h"
#include "zephyr/kernel.h"
#include <math.h>
// using static K value here which will need to be adjusted for each Rx, this is
// because 3*m*s/4*PI is constant and can be substituted for as a single value.
//
static float calibration_value = 86.5f;
static struct data_container_t local_data_tx;
static struct rx_data_container_t local_data_rx;
static struct pos_q_t local_data_rx_q;

K_MSGQ_DEFINE(positioning_queue, sizeof(struct solver_packet_t), 10, 4);

typedef struct {
  float p;       // Position estimate
  float v;       // Velocity estimate
  float P[2][2]; // Covariance matrix (C0, C1, C2, C3 in the paper)
} Kalman1D_t;

// Call this once before the while(1) loop
void kalman_init(Kalman1D_t *kf, float init_p) {
  kf->p = init_p;
  kf->v = 0.0f;
  kf->P[0][0] = 1.0f;
  kf->P[0][1] = 0.0f;
  kf->P[1][0] = 0.0f;
  kf->P[1][1] = 1.0f;
}

// The core filter based on Appendix A kinematics
float kalman_update(Kalman1D_t *kf, float p_meas, float a_meas, float dt) {
  // Tuning Parameters (v_u and v_p from the paper)
  float noise_accel = 0.5f; // Process noise (Variance of acceleration)
  float noise_pos = 0.01f;  // Measurement noise (Variance of magnetic position)

  // 1. Predict Next State [cite: 521]
  // p_est = p_est + v_est*dt + 0.5*a_meas*dt^2
  kf->p = kf->p + (kf->v * dt) + (0.5f * a_meas * dt * dt);
  kf->v = kf->v + (a_meas * dt);

  // 2. Predict Covariance [cite: 523]
  kf->P[0][0] += dt * (dt * kf->P[1][1] + kf->P[0][1] + kf->P[1][0]) +
                 (noise_accel * noise_accel * 0.25f * dt * dt * dt * dt);
  kf->P[0][1] +=
      dt * kf->P[1][1] + (noise_accel * noise_accel * 0.5f * dt * dt * dt);
  kf->P[1][0] +=
      dt * kf->P[1][1] + (noise_accel * noise_accel * 0.5f * dt * dt * dt);
  kf->P[1][1] += noise_accel * noise_accel * dt * dt;

  // 3. Calculate Kalman Gain [cite: 532]
  float S = kf->P[0][0] + (noise_pos * noise_pos);
  float K0 = kf->P[0][0] / S;
  float K1 = kf->P[1][0] / S;

  // 4. Update State Estimate [cite: 536]
  float y = p_meas - kf->p; // Innovation (Measurement residual)
  kf->p += K0 * y;
  kf->v += K1 * y;

  // 5. Update Covariance [cite: 539]
  float P00_temp = kf->P[0][0];
  float P01_temp = kf->P[0][1];
  kf->P[0][0] -= K0 * P00_temp;
  kf->P[0][1] -= K0 * P01_temp;
  kf->P[1][0] -= K1 * P00_temp;
  kf->P[1][1] -= K1 * P01_temp;

  return kf->p;
}

// Helper function to rotate a vector by a quaternion
void rotate_vector_by_quaternion(float q0, float q1, float q2, float q3,
                                 float *bx, float *by, float *bz) {
  float vx = *bx;
  float vy = *by;
  float vz = *bz;
  float cx = q2 * vz - q3 * vy;
  float cy = q3 * vx - q1 * vz;
  float cz = q1 * vy - q2 * vx;
  cx *= 2.0f;
  cy *= 2.0f;
  cz *= 2.0f;
  *bx = vx + q0 * cx + (q2 * cz - q3 * cy);
  *by = vy + q0 * cy + (q3 * cx - q1 * cz);
  *bz = vz + q0 * cz + (q1 * cy - q2 * cx);
}

// Transforms the raw magnetic vector into the Transmitter's local coordinate
// frame
void align_to_tx_frame(float *bx, float *by, float *bz) {
  // get quaterions data
  read_tx_data(&local_data_tx);
  read_pos_q_data(&local_data_rx);
  // 1. Invert Tx Quaternion (Assuming unit quaternions, inverse = conjugate)
  float inv_tx_w = local_data_tx.q0;
  float inv_tx_x = -local_data_tx.q1;
  float inv_tx_y = -local_data_tx.q2;
  float inv_tx_z = -local_data_tx.q3;

  // 2. Multiply Q_relative = (Inverse Q_tx) * Q_rx
  float rel_w = inv_tx_w * local_data_rx.q0 - inv_tx_x * local_data_rx.q1 -
                inv_tx_y * local_data_rx.q2 - inv_tx_z * local_data_rx.q3;
  float rel_x = inv_tx_w * local_data_rx.q1 + inv_tx_x * local_data_rx.q0 +
                inv_tx_y * local_data_rx.q3 - inv_tx_z * local_data_rx.q2;
  float rel_y = inv_tx_w * local_data_rx.q2 - inv_tx_x * local_data_rx.q3 +
                inv_tx_y * local_data_rx.q0 + inv_tx_z * local_data_rx.q1;
  float rel_z = inv_tx_w * local_data_rx.q3 + inv_tx_x * local_data_rx.q2 -
                inv_tx_y * local_data_rx.q1 + inv_tx_z * local_data_rx.q0;

  // 3. Rotate the magnetic vector using the relative quaternion
  rotate_vector_by_quaternion(rel_w, rel_x, rel_y, rel_z, bx, by, bz);
}

void calculate_pos(float *bx_val, float *by_val, float *bz_val) {
  float bx = *bx_val;
  float by = *by_val;
  float bz = *bz_val;
  // align_to_tx_frame(&bx, &by, &bz);
  // cache XY magnitude
  float bxy_mag = sqrtf(powf(bx, 2) + powf(by, 2)); // prevent divide by zero
  if (bxy_mag < 0.0001f) {
    bxy_mag = 0.0001f;
  }

  float c1 = bz / bxy_mag;
  float c2 = ((3.0f * c1) / 4.0f) + (sqrtf(9.0f * powf(c1, 2) + 8.0f) / 4.0f);

  float denominator = powf(1.0f + powf(c2, 2), 2.5f) * bxy_mag;
  float x0 = cbrtf((calibration_value * c2) / denominator);

  float zp = c2 * x0;

  if (fabsf(bx) < 0.0001f) {
    bx = (bx < 0.0f) ? -0.0001f : 0.0001f;
  }

  float xp_mag = x0 / sqrtf(1.0f + powf((by / bx), 2));
  float yp_mag = sqrtf(powf(x0, 2) - powf(xp_mag, 2));

  // 3. QUADRANT UNPACKING (The Magic Trick)
  // sign(x) = sign(Bx) * sign(z)
  // sign(y) = sign(By) * sign(z)

  float xp_final = xp_mag;
  float yp_final = yp_mag;

  // We use standard C signbit logic. If (Bx is negative) XOR (Z is negative)...
  if ((bx < 0.0f) != (zp < 0.0f)) {
    xp_final = -xp_mag;
  }

  if ((by < 0.0f) != (zp < 0.0f)) {
    yp_final = -yp_mag;
  }

  // Done! You now have true 3D spatial coordinates!
  // will need to store these in data structure and transmit to pc dongle
  *bx_val = xp_final;
  *by_val = yp_final;
  *bz_val = zp;
}

void get_global_linear_accel(float *ax_global, float *ay_global,
                             float *az_global) {
  // 2. Extract expected gravity vector from the Rx Quaternion
  // Assuming q0 = W, q1 = X, q2 = Y, q3 = Z
  float q0 = local_data_rx_q.q0;
  float q1 = local_data_rx_q.q1;
  float q2 = local_data_rx_q.q2;
  float q3 = local_data_rx_q.q3;

  float gx = 2.0f * (q1 * q3 - q0 * q2);
  float gy = 2.0f * (q0 * q1 + q2 * q3);
  float gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // 3. Subtract gravity to get Local Linear Acceleration
  float lin_x = ax - (gx * 9.81f);
  float lin_y = ay - (gy * 9.81f);
  float lin_z = az - (gz * 9.81f);

  // 4. Align the Local Linear Acceleration to the Transmitter's Global Frame
  // First, calculate the relative quaternion (Inverse Tx * Rx)
  float inv_tx_w = local_data_tx.q0;
  float inv_tx_x = -local_data_tx.q1;
  float inv_tx_y = -local_data_tx.q2;
  float inv_tx_z = -local_data_tx.q3;

  float rel_w = inv_tx_w * q0 - inv_tx_x * q1 - inv_tx_y * q2 - inv_tx_z * q3;
  float rel_x = inv_tx_w * q1 + inv_tx_x * q0 + inv_tx_y * q3 - inv_tx_z * q2;
  float rel_y = inv_tx_w * q2 - inv_tx_x * q3 + inv_tx_y * q0 + inv_tx_z * q1;
  float rel_z = inv_tx_w * q3 + inv_tx_x * q2 - inv_tx_y * q1 + inv_tx_z * q0;

  // Rotate the linear acceleration vector using the relative quaternion
  rotate_vector_by_quaternion(rel_w, rel_x, rel_y, rel_z, &lin_x, &lin_y,
                              &lin_z);

  // Output the fully corrected, global-aligned accelerations
  *ax_global = lin_x;
  *ay_global = lin_y;
  *az_global = lin_z;
}

void start_positioning_thread(void *, void *, void *) {
  struct solver_packet_t incoming_data;
  // 1. Initialize the 3 independent Kalman filters
  Kalman1D_t kf_x, kf_y, kf_z;
  kalman_init(&kf_x, 0.0f);
  kalman_init(&kf_y, 0.0f);
  kalman_init(&kf_z, 0.0f);

  // 1. Capture the starting time
  uint32_t last_time = k_uptime_get_32();
  while (1) {
    k_msgq_get(&positioning_queue, &incoming_data, K_FOREVER);

    uint32_t current_time = k_uptime_get_32();
    calculate_pos(&incoming_data.bx, &incoming_data.by, &incoming_data.bz);
    // 2. CALCULATE THE EXACT TRUE DT

    // Get the gravity-canceled, frame-aligned linear acceleration
    float a_global_x, a_global_y, a_global_z;
    get_global_linear_accel(&a_global_x, &a_global_y, &a_global_z);

    float dt =
        (float)(current_time - last_time) / 1000.0f; // Convert ms to seconds
    last_time = current_time;

    // 3. Safety Clamp (If the thread stalled for >50ms, ignore the velocity
    // spike)
    if (dt > 0.050f) {
      dt = 0.0025f;
    }

    // get accell data
    read_rx_data(&local_data_rx);
    // 2. Filter the raw data independently
    float smooth_x = kalman_update(&kf_x, incoming_data.bx, 0.0f,
                                   dt); // Replace 0.0f with IMU accel_x later
    float smooth_y = kalman_update(&kf_y, incoming_data.by, 0.0f,
                                   dt); // Replace 0.0f with IMU accel_y later
    float smooth_z = kalman_update(&kf_z, incoming_data.bz, 0.0f,
                                   dt); // Replace 0.0f with IMU accel_z later

    // float smooth_x = kalman_update(&kf_x, incoming_data.bx, 0.0f, dt) *
    // 1000.0f; float smooth_y = kalman_update(&kf_y, incoming_data.by, 0.0f,
    // dt) * 1000.0f; float smooth_z = kalman_update(&kf_z, incoming_data.bz,
    // 0.0f, dt) * 1000.0f;

    printk("Final Position: X: %.3f, Y: %.3f, Z: %.3f\n", smooth_x, smooth_y,
           smooth_z);

    // printk("Final Position: X: %.3f, Y: %.3f, Z: %.3f\n", incoming_data.bx,
    // incoming_data.by, incoming_data.bz);
  }
}
