#include "data_handle.h"
#include <hal/nrf_saadc.h>
#include <math.h>
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
K_MUTEX_DEFINE(rx_signal_mutex);
K_MUTEX_DEFINE(tx_mutex);
K_MUTEX_DEFINE(pos_mutex);
K_MUTEX_DEFINE(timestamp_mutex);
// store phase and quaternion of TX
struct data_container_t tx_data_container = {0};
// create empty container for ADC & accel & gyro
struct rx_data_container_t rx_data_container = {0};
// initialize container for orientation
struct pos_q_t pos_orientation_container = {0};
// init container for matched filter and dynamic gain
struct rx_data_signal_t rx_data_signal_container = {.gain = NRF_SAADC_GAIN1_6};
// store timestamp to determine how much to fast forward ADC reading times
static uint32_t phase_timestamp;
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

int setup_sin_cos_cache(struct cached_sin_cos_t *cached_s_c) {
  // cache values of target sin and cos wave for use during execution

  for (int i = 0; i < CONFIG_TX_SAMPLE_NUMBER; i++) {
    // *1000 because in KHZ
    float time = (float)i / (CONFIG_TX_ADC_FREQUENCY * 1000.0f);

    float angle = 2.0f * PI * (CONFIG_TX_PWM_FREQUENCY * 1000.0f) * time;
    // cached_s_c->sine[i] = sinf(angle);
    // cached_s_c->cosine[i] = cosf(angle);

    // 3. Generate the raw waves
    float raw_sine = sinf(angle);
    float raw_cosine = cosf(angle);

    // 4. Calculate the Hann Window to eliminate clock-drift spectral leakage
    float hann_multiplier =
        0.5f * (1.0f - cosf(2.0f * PI * (float)i / (float)(CONFIG_TX_SAMPLE_NUMBER - 1)));

    // 5. Apply the window and save to cache
    cached_s_c->sine[i] = raw_sine * hann_multiplier;
    cached_s_c->cosine[i] = raw_cosine * hann_multiplier;
  }
  return 0;
}

#if IS_ENABLED(CONFIG_IS_RECEIVER)

// -- Update timestamp
void update_timestamp(int32_t timestamp) {
  k_mutex_lock(&timestamp_mutex, K_FOREVER);
  phase_timestamp = timestamp;
  k_mutex_unlock(&timestamp_mutex);
}

// -- Read timestamp
void read_timestamp(int32_t *timestamp) {
  k_mutex_lock(&timestamp_mutex, K_FOREVER);
  *timestamp = phase_timestamp;
  k_mutex_unlock(&timestamp_mutex);
}

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

void update_rx_adc_data(float amp, float phase, char *axis) {
  // update amp and phase for specific axis (x,y,z)
  // TODO: improve this too messy
  k_mutex_lock(&rx_signal_mutex, K_FOREVER);
  if (axis == "x") {
    rx_data_signal_container.adc_x_amp = amp;
    rx_data_signal_container.adc_x_phase = phase;
  } else if (axis == "y") {
    rx_data_signal_container.adc_y_amp = amp;
    rx_data_signal_container.adc_y_phase = phase;
  } else {
    rx_data_signal_container.adc_z_amp = amp;
    rx_data_signal_container.adc_z_phase = phase;
  }
  k_mutex_unlock(&rx_signal_mutex);
}

void read_rx_adc_data(struct rx_data_signal_t *data_destination) {
  // stores data in data destination struct;
  k_mutex_lock(&rx_signal_mutex, K_FOREVER);
  *data_destination = rx_data_signal_container;
  k_mutex_unlock(&rx_signal_mutex);
}

void update_rx_gain_data(nrf_saadc_gain_t gain) {
  k_mutex_lock(&rx_mutex, K_FOREVER);
  rx_data_signal_container.gain = gain;
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
