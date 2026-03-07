#include "adc.h"
#include "data_handle.h"
#include <math.h>
#include <stdint.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PI 3.14159265f

int16_t sample_buf[CONFIG_TX_SAMPLE_NUMBER];
int32_t val_mv;
int32_t err;

// NOTE: store 100(default) signal entries for phase and amp calculation
static float amp_bias;
static int32_t signal_arr[CONFIG_TX_SAMPLE_NUMBER] = {};
struct cached_sin_cos_t cached_s_c = {.sine = {}, .cosine = {}};

// NOTE: dividing by 1000 to get microseconds
struct adc_sequence_options options = {
    .extra_samplings = CONFIG_TX_SAMPLE_NUMBER - 1,
    .interval_us = 1000 / CONFIG_TX_ADC_FREQUENCY,
};

struct adc_sequence sequence = {
    .buffer = &sample_buf,
    /* buffer size in bytes, not number of samples */
    .buffer_size = sizeof(sample_buf),
    .options = &options};

// Get ADC transmission amplitude pin
static const struct adc_dt_spec tx_adc = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

int tx_matched_filter(uint32_t *signal_buf,
                      struct cached_sin_cos_t *cached_s_c) {
  // NOTE: signal buf should be an array of samples from ADC
  float sin_accumulation = 0.0f;
  float cos_accumulation = 0.0f;
  float amp = 0.0f;
  float phase = 0.0f;
  for (int i = 0; i < CONFIG_TX_SAMPLE_NUMBER; i++) {
    sin_accumulation = sin_accumulation + (cached_s_c->sine[i] * signal_buf[i]);
    cos_accumulation =
        cos_accumulation + (cached_s_c->cosine[i] * signal_buf[i]);
  }
  amp = ((sqrtf(powf(sin_accumulation, 2) + powf(cos_accumulation, 2)) /
          CONFIG_TX_SAMPLE_NUMBER)) *
        amp_bias;
  phase = atan2f(sin_accumulation, cos_accumulation);

  // update container
  update_signal_data(amp, phase);

  return 0;
}

// TODO: add closed loop duty cycle adjustment

int read_adc() {
  // result is a pointer to store result
  // NOTE: may need to reset buffer variable here, if program crashes in future
  // check this
  int err;
  err = adc_read(tx_adc.dev, &sequence);

  if (err < 0) {
    printk("Could not read (%d)", err);
    return 1;
  }
  for (int i = 0; i < CONFIG_TX_SAMPLE_NUMBER; i++) {
    int32_t val_mv = (int32_t)sample_buf[i];
    err = adc_raw_to_millivolts_dt(&tx_adc, &val_mv);
    if (err < 0) {
      printk(" (value in mV not available)\n");
    } else {
      printk("%d mV\n", val_mv);
      signal_arr[i] = val_mv;
    }
  }
  return 0;
}

int setup_sin_cos_cache(struct cached_sin_cos_t *cached_s_c) {
  // cache values of target sin and cos wave for use during execution
  for (int i = 0; i < CONFIG_TX_SAMPLE_NUMBER; i++) {
    // *1000 because in KHZ
    float time = (float)i / (float)CONFIG_TX_ADC_FREQUENCY;
    float angle = 2.0f * PI * (CONFIG_TX_PWM_FREQUENCY * 1000.0f) * time;
    cached_s_c->sine[i] = sin(angle);
    cached_s_c->cosine[i] = cos(angle);
    // get sine and cosine for every number of signal
  }

  // NOTE: 12 is comming from ADC bit resolution, this should probably be place
  // in the KCONFIG
  amp_bias = 3.3 / powf(2, 12);

  // NOTE: need to store how long to sleep
  return 0;
}

int setup_adc() {
  if (!adc_is_ready_dt(&tx_adc)) {
    // issue with this ADC pin
    printk("transmittor ADC is not ready\n");
    return err;
  }

  err = adc_channel_setup_dt(&tx_adc);
  if (err < 0) {
    // LOG_ERR("Could not setup channel #%d (%d)", 0, err);
    printk("Could not setup channel #%d (%d)", 0, err);
    return err;
  }

  // set up adc sequence
  err = adc_sequence_init_dt(&tx_adc, &sequence);
  if (err < 0) {
    printk("Could not initalize sequnce");
    return err;
  }

  return 0;
}

// TODO: make a thread function which keeps track of signal buffer and updates
// shared data var would need to call read ADC a couple of times and then save
// result in buffer

void start_adc_thread(void *, void *, void *) {
  // this function acts as entry point for starting a thread
  int err;
  err = setup_sin_cos_cache(&cached_s_c);
  if (err != 0) {
    printk("Failed to setup sine and cosine cache: %d", err);
    // NOTE: should continue and read to read again
  }

  while (1) {
    // get all signal data
    err = read_adc();
    if (err != 0) {
      printk("ADC error: %d", err);
      // NOTE: should continue and read to read again
      continue;
    }

    tx_matched_filter(&signal_arr, &cached_s_c);

    // sleep so it does this at 100 hz
    // 1000 (ms) / hz to get millisecond sleep time
    k_msleep(1000 / CONFIG_TX_BROADCAST_FREQUENCY);
  }
}
