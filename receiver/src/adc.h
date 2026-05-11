#ifndef ADC_H
#define ADC_H

#include "data_handle.h"
#include <nrfx_saadc.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

struct sample_buf_t {
  int16_t x[2][CONFIG_RX_SAMPLE_NUMBER];
  int16_t y[2][CONFIG_RX_SAMPLE_NUMBER];
  int16_t z[2][CONFIG_RX_SAMPLE_NUMBER];
};

struct coil_data_storage {
  float *amp;
  float *phase;
  char axis[2];
  uint32_t *axis_timestamp; // used to store pointer of the timestamp
  float phase_offset_rad;   // phase offset for each coil
  float min_amp;            // Minimum amp to actually listen to phase readings
  float deadband_cos; // if reading is strongly pos or neg switch it, if near 0
                      // (90deg) dont use result.
  float last_sign;
  uint8_t hold_number; // how long the sign has been consistant
};

static void saadc_handler(nrfx_saadc_evt_t const *p_event);

int config_saadc();

int config_timer();

int config_ppi();

int matched_filter(int16_t *signal_buf, struct cached_sin_cos_t *cached_s_c,
                   char *axis);

int setup_sin_cos_cache(struct cached_sin_cos_t *cached_s_c);

void start_adc_thread(void *, void *, void *);

#endif
