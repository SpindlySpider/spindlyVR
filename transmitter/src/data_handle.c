#include "data_handle.h"
#include <zephyr/kernel.h>

// handles data: quaternions, amp and phase
// in a thead safe way
// TODO: let the data container struct be passed in form args

K_MUTEX_DEFINE(container_mutex);

// struct data_container_t  data_container;
// struct data_container_t data_container = {
//     .q0 = 0.0, .q1 = 1.1, .q2 = 2.2, .q3 = 3.3, .tx_phase = 4.4, .tx_amp = 5.5};

struct data_container_t data_container = {0};

void update_orientation_data(float q0, float q1, float q2, float q3) {
  // lock
  k_mutex_lock(&container_mutex, K_FOREVER);

  // Update struct
  data_container.q0 = q0;
  data_container.q1 = q1;
  data_container.q2 = q2;
  data_container.q3 = q3;

  // release lock
  k_mutex_unlock(&container_mutex);
}

void update_signal_data(float amp, float phase) {
  // lock
  k_mutex_lock(&container_mutex, K_FOREVER);

  // Update struct
  data_container.tx_amp = amp;
  data_container.tx_phase = phase;

  // release lock
  k_mutex_unlock(&container_mutex);
}

void read_data(struct data_container_t *data_destination) {
  // struct data_container_t data_container = {.q0 = 0.0f,
  //                                           .q1 = 1.1f,
  //                                           .q2 = 2.2f,
  //                                           .q3 = 3.3f,
  //                                           .tx_phase = 4.4f,
  //                                           .tx_amp = 5.5f};

  // stores data in data destination struct;
  k_mutex_lock(&container_mutex, K_FOREVER);
  *data_destination = data_container;
  k_mutex_unlock(&container_mutex);
}
