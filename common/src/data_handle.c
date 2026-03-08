#include "data_handle.h"
#include <zephyr/kernel.h>

// handles data: quaternions, amp and phase
// in a thead safe way
// TODO: let the data container struct be passed in form args

K_MUTEX_DEFINE(container_mutex);

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
  // stores data in data destination struct;
  k_mutex_lock(&container_mutex, K_FOREVER);
  *data_destination = data_container;
  k_mutex_unlock(&container_mutex);
}
