#include "position.h"
#include "data_handle.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include <math.h>
#include <stdint.h>

K_MSGQ_DEFINE(positioning_queue, sizeof(struct solver_packet_t), 10, 4);

// precompute this
static const float ONE_DIVIDE_THREE = 1.0f / 3.0f;
static const float FIVE_DIVIDE_TWO = 5.0f / 2.0f;
static const float GRAVITY = 9.80665;

static struct data_container_t local_data_tx = {0};
static struct data_container_t local_data_rx = {0};

// Calibrate magnetic moment
static float cal_magnetic_moment = 10696.906455172766f;

struct vector_t rotate_vector(struct vector_t raw_pos, struct quaternion_t tx_q,
                              struct quaternion_t inverse_rx_q) {
  // ensure normalise the quaterion
  normalise_quaternion(&tx_q);

  // multiple tx quaternion by inverse of RX quaterionion to find the relative
  // orientation to rotate vectors
  struct quaternion_t q_realtive = multiply_quaternion(inverse_rx_q, tx_q);
  struct quaternion_t inverse_q_relative = inverse_quaternion(&q_realtive);

  struct quaternion_t mag_vector = {
      .w = 0.0f, .x = raw_pos.x, .y = raw_pos.y, .z = raw_pos.z};

  // rotate magnetic vectors (rotation via shoemakers paper q^-1 * V * q)
  mag_vector = multiply_quaternion(inverse_q_relative, mag_vector);
  mag_vector = multiply_quaternion(mag_vector, q_realtive);

  // rotated RX vector
  struct vector_t rx_pos_rot = {mag_vector.x, mag_vector.y, mag_vector.z};
  return rx_pos_rot;
}

void calculate_pos(float bx, float by, float bz, float a_x, float a_y,
                   float a_z) {
  // cache values so this is slightly less expensive
  // also this is entirely taken from research paper
  float sqrt_bx_by = sqrtf((bx * bx) + (by * by));

  float c1 = bz / sqrt_bx_by;
  // calculate with +/-
  float c2_part1 = (3 * c1 / 4);
  float c2_part2 = (sqrtf((9 * (c1 * c1)) + 8) / 4);

  float c2_plus = c2_part1 + c2_part2;
  float c2_minus = c2_part1 - c2_part2;

  // TODO: decide which +/- here or calculate both -/+ and decide at the end?
  // not sure this is the right place since (5) has +/- but (6)(c2) does not
  // only has + but it says only considering first quadrant?
  float c2 = c2_plus;

  float x0_denominator =
      (4 * PI * (1 + (c2 * c2)) * FIVE_DIVIDE_TWO * sqrt_bx_by);
  float x0_numerator = 3 * cal_magnetic_moment * c2;
  float x0 = (x0_numerator / x0_denominator) * (ONE_DIVIDE_THREE);

  // need to apply sign from rotated vector

  float zp = c2 * x0;

  float xp = x0 / sqrtf(1 + powf((by / bx), 2.0f));
  float yp = sqrtf((x0 * x0) - (xp * xp));

  // will need to apply zp, sign after ? might not since the + - will probbaly produce the correct results
  xp = (bx >= 0.0f) ? xp : -xp;
  yp = (by >= 0.0f) ? yp : -yp;

  // printk(
  //     " c1: %-7.3f | c2: %-7.3f | x0: %-7.3f | x0_n: %-7.3f | x0_d:
  //     %-7.3f\n", c1, c2, x0, x0_numerator, x0_denominator);
  // print values

  // print values for debug and calibration
  printk(" X: %-10.8f | Y: %-10.8f | Z: %-10.8f | accel X: %-10.8f | accel Y: "
         "%-10.8f | accel Z: %-10.8f\n",
         xp, yp, zp, a_x, a_y, a_z);
  // TODO: add kalman filter activation here in another func
}

void calculate_linear_acceleration(float *a_x, float *a_y, float *a_z,
                                   struct quaternion_t rx_q,
                                   struct quaternion_t inverse_rx_q) {
  // calculated for all axis for later kalman filter, for position estimation
  // only, only z axis is required
  struct quaternion_t gravity_q = {.w = 0, .x = 0, .y = 0, .z = -GRAVITY};
  // rotate sensor frame to world frame and then remove gravity
  // ISSUE: this will cause errors if this is not the correct convention.
  gravity_q = multiply_quaternion(inverse_rx_q, gravity_q);
  gravity_q = multiply_quaternion(gravity_q, rx_q);
  // Remove gravity from the accelerometer data
  *a_x = *a_x - gravity_q.x;
  *a_y = *a_y - gravity_q.y;
  *a_z = *a_z - gravity_q.z;
}

void start_positioning_thread(void *, void *, void *) {
  struct solver_packet_t incoming_data;

  while (1) {
    // TODO: maybe dont wait on queue to free up, just keep guessing using accel
    // data and if a position item comes in use that to stablize
    k_msgq_get(&positioning_queue, &incoming_data, K_FOREVER);

    struct vector_t rx_pos_raw = {incoming_data.bx, incoming_data.by,
                                  incoming_data.bz};

    // NOTE: forced postive to see how position does, if this fixes most
    // positoning stuff then phase sync sign needs fixing

    // struct vector_t rx_pos_raw = {
    //     fabsf(incoming_data.bx), fabsf(incoming_data.by),
    //     fabsf(incoming_data.bz)};

    // printk(" RAW X: %-7.3f | RAW Y: %-7.3f | RAW Z: %-7.3f\n", rx_pos_raw.x,
    //        rx_pos_raw.y, rx_pos_raw.z);

    struct quaternion_t q_tx_frame = {
        .w = incoming_data.tx_q0,
        .x = incoming_data.tx_q1,
        .y = incoming_data.tx_q2,
        .z = incoming_data.tx_q3,
    };

    struct quaternion_t q_rx_frame = {
        .w = incoming_data.rx_q0,
        .x = incoming_data.rx_q1,
        .y = incoming_data.rx_q2,
        .z = incoming_data.rx_q3,
    };

    // acceleration raw values
    float a_x, a_y, a_z;

    normalise_quaternion(&q_rx_frame);
    // find inverse of rx quaternion
    struct quaternion_t inverse_rx_q = inverse_quaternion(&q_rx_frame);

    calculate_linear_acceleration(&a_x, &a_y, &a_z, q_rx_frame, inverse_rx_q);

    // rotated vector
    struct vector_t rot_vector;
    rot_vector = rotate_vector(rx_pos_raw, q_tx_frame, inverse_rx_q);

    // rot_vector = rx_pos_raw;

    // printk(" ROT X: %-7.3f | ROT Y: %-7.3f | ROT Z: %-7.3f\n", rot_vector.x,
    //        rot_vector.y, rot_vector.z);

    // estimate position from rotated vector
    calculate_pos(rot_vector.x, rot_vector.y, rot_vector.z, a_x, a_y, a_z);
  }
}
