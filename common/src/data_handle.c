#include "data_handle.h"
#include <hal/nrf_saadc.h>
#include <nrfx_saadc.h>
#include <zephyr/kernel.h>

// handles data: quaternions, amp and phase
// in a thead safe way
// TODO: let the data container struct be passed in form args

// required for both Tx & Rx, handles local quaternion data
K_MUTEX_DEFINE(container_mutex);

#if IS_ENABLED(CONFIG_IS_RECEIVER)
#include <nrfx_saadc.h>

K_MUTEX_DEFINE(rx_mutex);
K_MUTEX_DEFINE(tx_mutex);
K_MUTEX_DEFINE(pos_mutex);
// store phase and quaternion of TX
struct data_container_t tx_data_container = {0};
// create empty container for ADC & accel & gyro
struct rx_data_container_t rx_data_container = {0};
// initialize container for orientation
struct pos_q_t pos_orientation_container = {0};
#endif

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

#if IS_ENABLED(CONFIG_IS_RECEIVER)

// --- TX container manipulation
void read_tx_data(struct data_container_t *data_destination) {
  // stores data in data destination struct;
  k_mutex_lock(&tx_mutex, K_FOREVER);
  *data_destination = tx_data_container;
  k_mutex_unlock(&tx_mutex);
}

void update_tx_data(float q0, float q1, float q2, float q3, float phase) {
  k_mutex_lock(&tx_mutex, K_FOREVER);

  // Update struct
  data_container.q0 = q0;
  data_container.q1 = q1;
  data_container.q2 = q2;
  data_container.q3 = q3;
  data_container.tx_phase = phase;

  // release lock
  k_mutex_unlock(&tx_mutex);
}

// --- RX container manipulation
void read_rx_data(struct rx_data_container_t *data_destination) {
  // stores data in data destination struct;
  k_mutex_lock(&rx_mutex, K_FOREVER);
  *data_destination = rx_data_container;
  k_mutex_unlock(&rx_mutex);
}

void update_rx_adc_data(float x, float y, float z) {
  k_mutex_lock(&rx_mutex, K_FOREVER);
  rx_data_container.adc_x = x;
  rx_data_container.adc_y = y;
  rx_data_container.adc_z = z;
  k_mutex_unlock(&rx_mutex);
}

void update_rx_gain_data(nrf_saadc_gain_t gain) {
  k_mutex_lock(&rx_mutex, K_FOREVER);
  rx_data_container.gain = gain;
  k_mutex_unlock(&rx_mutex);
}

void update_rx_accel_gyro_data(float accel_x, float accel_y, float accel_z,
                               float gyro_x, float gyro_y, float gyro_z) {
  k_mutex_lock(&rx_mutex, K_FOREVER);
  rx_data_container.accel_x = accel_x;
  rx_data_container.accel_y = accel_y;
  rx_data_container.accel_z = accel_z;
  rx_data_container.gyro_x = gyro_x;
  rx_data_container.gyro_y = gyro_y;
  rx_data_container.gyro_z = gyro_z;
  k_mutex_unlock(&rx_mutex);
}

// --- Position and quaternion container manipulation

void read_pos_q_data(struct pos_q_t *data_destination) {
  // stores data in data destination struct;
  k_mutex_lock(&pos_mutex, K_FOREVER);
  *data_destination = pos_orientation_container;
  k_mutex_unlock(&pos_mutex);
}

void update_pos_q_data(float q0, float q1, float q2, float q3, float x, float y,
                       float z) {
  k_mutex_lock(&pos_mutex, K_FOREVER);
  pos_orientation_container.q0 = q0;
  pos_orientation_container.q1 = q1;
  pos_orientation_container.q2 = q2;
  pos_orientation_container.q3 = q3;
  pos_orientation_container.x = x;
  pos_orientation_container.y = y;
  pos_orientation_container.z = z;
  k_mutex_unlock(&pos_mutex);
}

#endif
