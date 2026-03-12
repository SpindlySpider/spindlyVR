#ifndef DATA_HANDLE_H
#define DATA_HANDLE_H

#include <stdint.h>
#include <hal/nrf_saadc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

struct data_container_t {
  // Quaternion orientation
  float q0, q1, q2, q3;

  // transmitted to Rx so it knows its signed magnetic field
  float tx_amp, tx_phase;
};

// used for quick conversion of data to bytes
union esb_packet {
  struct data_container_t data;
  uint8_t bytes[sizeof(struct data_container_t)];
};

// ESB radio address (make sure your Rx and Tx share the same)
static uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
static uint8_t addr_prefix[8] = {0xC2, 0xC3, 0xC4, 0xC5,
                                 0xC6, 0xC7, 0xC8, 0xC9};

// local data
void update_orientation_data(float q0, float q1, float q2, float q3);

void update_signal_data(float amp, float phase);

void read_data(struct data_container_t *data_destination);

#if IS_ENABLED(CONFIG_IS_RECEIVER)
struct rx_data_container_t {
  // data container for rx specific attributes
  // Received voltage on Rx tri-axis coil
  float adc_x, adc_y, adc_z;
  // ADC gain (incase low signal)
  nrf_saadc_gain_t gain;
  // accel & gyro data for dynamic calibration & position conformation
  float accel_x,accel_y,accel_z;
  float gyro_x, gyro_y, gyro_z;
};

struct pos_q_t {
  // struct for position and quaterion, seperate from other containers to
  // reduced lock operations when transmitting
  float x, y, z;
  float q0, q1, q2, q3;
};

void read_tx_data(struct data_container_t *data_destination);
void update_tx_data(float q0, float q1, float q2, float q3, float phase);
void update_rx_adc_data(float x, float y, float z);
void update_rx_accel_gyro_data(float accel_x, float accel_y, float accel_z,
                               float gyro_x, float gyro_y, float gyro_z);

void read_pos_q_data(struct pos_q_t *data_destination);
void update_pos_q_data(float q0, float q1, float q2, float q3, float x, float y,
                       float z);
#endif
#endif
