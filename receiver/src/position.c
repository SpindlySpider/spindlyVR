#include "position.h"
#include "data_handle.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include <math.h> #include <stdint.h>
K_MSGQ_DEFINE(positioning_queue, sizeof(struct solver_packet_t), 10, 4);

static struct data_container_t local_data_tx = {0};
static struct data_container_t local_data_rx = {0};

// Calibrate magnetic moment
static float position_M = 1.0f;

void normalise_quaternion(struct quaternion_t *q) {
  // create unit quaternion
  struct quaternion_t q_in = *q;
  // get magnitude
  float mag = (q_in.w * q_in.w) + (q_in.x * q_in.x) + (q_in.y * q_in.y) +
              (q_in.z * q_in.z);
  // normalise
  q_in.w = q_in.w / mag;
  q_in.x = q_in.x / mag;
  q_in.y = q_in.y / mag;
  q_in.z = q_in.z / mag;
  *q = q_in;
}

// im sure this could be a lambda function
struct quaternion_t inverse_quaternion(struct quaternion_t *q) {
  struct quaternion_t q_in = {.w = q->w, .x = -q->x, .y = -q->y, .z = -q->z};
  return q_in;
}

struct quaternion_t multiply_quaternion(struct quaternion_t q1,
                                        struct quaternion_t q2) {
  // following this equation
  // https://en.wikipedia.org/wiki/Quaternion#Hamilton_product
  struct quaternion_t product = {
      .w = (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z),
      .x = (q1.w * q2.x) + (q1.x * q2.w) + (q1.y * q2.z) - (q1.z * q2.y),
      .y = (q1.w * q2.y) - (q1.x * q2.z) + (q1.y * q2.w) + (q1.z * q2.x),
      .z = (q1.w * q2.z) + (q1.x * q2.y) - (q1.y * q2.x) + (q1.z * q2.w)};
  return product;
}

struct vector_t rotate_vector(vector_t raw_pos, struct quaternion_t tx_q,
                              struct quaternion_t rx_q) {
  // ensure normalise the quaterion
  normalise_quaternion(&tx_q);
  normalise_quaternion(&rx_q);

  // find inverse of rx quaternion
  rx_q = inverse_quaternion(&rx_q);

  // multiple tx quaternion by inverse of RX quaterionion to find the relative
  // orientation to rotate vectors
  struct quaternion_t q_realtive = multiply_quaternion(rx_q, tx_q);
  struct quaternion_t inverse_q_relative = inverse_quaternion(&q_realtive);

  struct quaternion_t mag_vector = {
      .w = 0.0f, .x = rx_pos_raw.x, .y = rx_pos_raw.y, .z = rx_pos_raw.z};

  // rotate magnetic vectors
  mag_vector = multiply_quaternion(inverse_q_relative, mag_vector);
  mag_vector = multiply_quaternion(mag_vector, q_realtive);

  // rotated RX vector
  struct vector_t rx_pos_rot = {mag_vector.x, mag_vector.y, mag_vector.z};
  return rx_pos_rot;
}

void start_positioning_thread(void *, void *, void *) {
  struct solver_packet_t incoming_data;

  while (1) {
    // TODO: maybe dont wait on queue to free up, just keep guessing and if a
    // position item comes in use that
    k_msgq_get(&positioning_queue, &incoming_data, K_FOREVER);

    struct vector_t rx_pos_raw = {incoming_data.bx, incoming_data.by,
                                  incoming_data.bz};

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

    // rotated vector
    struct vector_t rot_vector = rotate_vector(rx_pos_raw, q_tx_frame, q_rx_frame);

    // estimate position from rotated vector
  }
