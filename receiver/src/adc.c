#include "data_handle.h"
#include "hal/nrf_saadc.h"
#include "hal/nrf_saadc.h"
#include "hal/nrf_timer.h"
#include "helpers/nrfx_analog_common.h"
#include "nrfx_templates_config.h"
#include <helpers/nrfx_gppi.h>
#include <math.h>

#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// https://github.com/NordicPlayground/nRF52-ADC-examples/tree/master/nrfx_saadc_multi_channel_ppi

// pin out from here: https://nicekeyboards.com/docs/nice-nano/pinout-schematic/
// #define PIN_X NRFX_ANALOG_EXTERNAL_AIN0 // pin 0.02
// #define PIN_Y NRFX_ANALOG_EXTERNAL_AIN5 // pin 0.29
// #define PIN_Z NRFX_ANALOG_EXTERNAL_AIN7 // pin 0.31
//
// ADC pins
#define PIN_X NRF_SAADC_INPUT_AIN0 // pin 0.02
#define PIN_Y NRF_SAADC_INPUT_AIN5 // pin 0.29
#define PIN_Z NRF_SAADC_INPUT_AIN7 // pin 0.31
// pins not exposed so use as hidden
#define PIN_X_HIDDEN NRF_SAADC_INPUT_AIN1 // pin 0.02
#define PIN_Y_HIDDEN NRF_SAADC_INPUT_AIN3 // pin 0.02
#define PIN_Z_HIDDEN NRF_SAADC_INPUT_AIN4 // pin 0.02
//

K_SEM_DEFINE(adc_semaphore, 0, 1);

// setup timer
static nrfx_timer_t adc_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(3));

// this variable is a pointer to the full buffer.
static uint32_t current_buffer = 0;

static struct rx_data_signal_t local_signal_data;

int16_t sample_buf_x[CONFIG_RX_SAMPLE_NUMBER] = {0};
int16_t sample_buf_y[CONFIG_RX_SAMPLE_NUMBER] = {0};
int16_t sample_buf_z[CONFIG_RX_SAMPLE_NUMBER] = {0};

int32_t val_mv;
int32_t err;

// NOTE: store 100(default) signal entries for phase and amp calculation
static float amp_bias;

struct cached_sin_cos_t cached_s_c = {.sine = {0}, .cosine = {0}};

static void saadc_handler(nrfx_saadc_evt_t const *p_event) {
  // if the next buffer is ready to be filled
  switch (p_event->type) {
  case NRFX_SAADC_EVT_DONE:
    k_sem_give(&adc_semaphore);
    break;
  }
}

int config_saadc() {
  int err;
  // incredibly useful:
  // https://github.com/zephyrproject-rtos/hal_nordic/tree/master/nrfx/samples/src/nrfx_saadc/advanced_non_blocking_internal_timer

  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST,
              nrfx_saadc_irq_handler, 0, 0);

  err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
  if (err != 0) {
    printk("Error setting up SAADC: %d\n", err);
    k_msleep(1000);
    return err;
  }
  printk("Got passed init SAAADC\n");
  k_msleep(1000);

  nrfx_saadc_channel_t default_channel =
      NRFX_SAADC_DEFAULT_CHANNEL_SE(PIN_X, 0);
  default_channel.channel_config.acq_time = NRF_SAADC_ACQTIME_3US;
  nrfx_saadc_channel_config(&default_channel);

  printk("Got passed default channel\n");
  k_msleep(1000);

  nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;

  uint32_t channel_mask = nrfx_saadc_channels_configured_get();
  err = nrfx_saadc_advanced_mode_set(channel_mask, NRF_SAADC_RESOLUTION_12BIT,
                                     &adv_config, saadc_handler);

  // err = nrfx_saadc_advanced_mode_set(BIT(1), NRF_SAADC_RESOLUTION_12BIT,
  // &adv_config, saadc_handler);
  if (err != 0) {
    return err;
  }
  printk("Got passed advanced mode set\n");
  k_msleep(1000);

  printk("Got passed buffer set\n");
  k_msleep(1000);

  // err = nrfx_saadc_offset_calibrate(saadc_handler);
  // if (err != 0) {
  // return err;
  // }

  // printk("Got passed offset callibrate\n");
  // k_msleep(1000);

  return 0;
}

int config_timer() {
  // incredibly useful:
  // https://github.com/zephyrproject-rtos/hal_nordic/tree/master/nrfx/samples/src/nrfx_timer

  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER3), IRQ_PRIO_LOWEST,
              nrfx_timer_irq_handler, &adc_timer, 0);

  uint32_t frequency = NRF_TIMER_BASE_FREQUENCY_GET(adc_timer.p_reg);

  nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(frequency);

  err = nrfx_timer_init(&adc_timer, &config, NULL);

  nrfx_timer_clear(&adc_timer);

  // convert kilohertz to micro seconds
  uint32_t khz_to_us = (uint32_t)(1000 / CONFIG_RX_ADC_FREQUENCY);
  uint32_t desired_ticks = nrfx_timer_us_to_ticks(&adc_timer, khz_to_us);

  nrfx_timer_extended_compare(&adc_timer, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  return 0;
}

int config_ppi() {
  // set up generic PPI
  nrfx_gppi_handle_t gppi_handle;

  // connect timer event compare0 to saadc task sample (take a sample each time
  // timer ticks)
  int err = nrfx_gppi_conn_alloc(
      nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE0),
      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE),
      &gppi_handle);

  if (err != 0) {
    return err;
  }

  // enable this connection
  nrfx_gppi_conn_enable(gppi_handle);
  return 0;
}

int tx_matched_filter(int16_t *signal_buf, struct cached_sin_cos_t *cached_s_c,
                      char *axis) {
  // TODO: this can be moved into a common file and values can be passed in, so
  // it is common between receiver and transmitter NOTE: signal buf should be an
  // array of samples from ADC
  float sin_accumulation = 0.0f;
  float cos_accumulation = 0.0f;
  float amp = 0.0f;
  float phase = 0.0f;
  int32_t dc_sum = 0;

  for (int i = 0; i < CONFIG_RX_SAMPLE_NUMBER; i++) {
    dc_sum += signal_buf[i];
  }
  float true_dc_offset = (float)dc_sum / (float)CONFIG_RX_SAMPLE_NUMBER;

  for (int i = 0; i < CONFIG_RX_SAMPLE_NUMBER; i++) {
    float ac_wave_signal = (float)signal_buf[i] - true_dc_offset;
    // float ac_wave_signal = (float)signal_buf[i];

    sin_accumulation += (cached_s_c->sine[i] * ac_wave_signal);
    // sin_accumulation += (cached_s_c->sine[i] * signal_buf[i]);
    cos_accumulation += (cached_s_c->cosine[i] * ac_wave_signal);
    // cos_accumulation += (cached_s_c->cosine[i] * signal_buf[i]);
  }

  // int32_t millivolts = (signal_buf[0] * 3600) / 4096;
  // printk("ADC Reading: %d mV\n", millivolts);

  amp = ((sqrtf(powf(sin_accumulation, 2) + powf(cos_accumulation, 2)) /
          CONFIG_RX_SAMPLE_NUMBER));

  amp = amp * 2.0f;
  // normalize after gain
  // amp = (amp / 2048.0f) * 0.15f;
  amp = (amp / 4096.0f) * 3.6f;
  // amp = amp * amp_bias;

  amp = amp  * 1000.0f;

  phase = atan2f(sin_accumulation, cos_accumulation);

  // update container
  update_rx_adc_data(amp, phase, axis);

  return 0;
}

void switch_channel(nrf_saadc_input_t pin,nrf_saadc_input_t reference) {

  // nrf_saadc_channel_pos_input_set(NRF_SAADC, 0, pin);
  // nrfx_saadc_channel_t channel_config =
  // NRFX_SAADC_DEFAULT_CHANNEL_SE(NRFX_ANALOG_EXTERNAL_AIN5, 0);
  // nrfx_saadc_channel_t channel_config =
  // NRFX_SAADC_DEFAULT_CHANNEL_SE(_pin_p, _index);
  // nrf_saadc_channel_input_set(NRF_SAADC, 0, pin, NRF_SAADC_INPUT_DISABLED);

  // channel_config.channel_config.acq_time = NRF_SAADC_ACQTIME_3US;
  // get / adjust gain here
  struct rx_data_signal_t data;
  read_rx_adc_data(&data);
  // channel_config.channel_config.gain = data.gain;
  // err = nrfx_saadc_channel_config(&channel_config);

  nrf_saadc_channel_config_t hw_config = {
      .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
      .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
      // .resistor_n = NRF_SAADC_RESISTOR_VDD1_2,
    .gain = data.gain,
      .reference = NRF_SAADC_REFERENCE_INTERNAL,
      .acq_time = NRF_SAADC_ACQTIME_3US,
      // .mode = NRF_SAADC_MODE_DIFFERENTIAL,
      .mode = NRF_SAADC_MODE_SINGLE_ENDED,
      .burst = NRF_SAADC_BURST_DISABLED};

  nrf_saadc_channel_init(NRF_SAADC, 0, &hw_config);
  // nrf_saadc_channel_input_set(NRF_SAADC, 0, pin, reference);
  nrf_saadc_channel_input_set(NRF_SAADC, 0, pin, NRF_SAADC_INPUT_DISABLED);

  if (err != 0) {
    printk("Error switching channels %d\n", err);
    k_msleep(1000);
  }
  // TODO:
  // work out how to incorparate dynamic gain
}

void adc_sample(int16_t *buf, nrf_saadc_input_t pin,nrf_saadc_input_t reference) {
  // switch channel
  switch_channel(pin, reference);
  // printk("Switched channel\n");
  // k_msleep(1000);
  // collect samples
  nrfx_saadc_buffer_set(buf, CONFIG_RX_SAMPLE_NUMBER);
  // printk("Set buffer\n");
  // k_msleep(1000);

  nrfx_saadc_mode_trigger();

  // printk("Triggered SAADC\n");
  // k_msleep(1000);

  nrfx_timer_clear(&adc_timer);

  nrfx_timer_enable(&adc_timer);
  // printk("Started timer\n");
  // k_msleep(1000); // use semaphore to know when samples ready
  k_sem_take(&adc_semaphore, K_FOREVER);

  nrfx_timer_disable(&adc_timer);
}

void start_adc_thread(void *, void *, void *) {
  // this function acts as entry point for starting a thread
  int err;

  read_rx_adc_data(&local_signal_data);
  printk("setting up adc\n");

  err = setup_sin_cos_cache(&cached_s_c);

  if (err != 0) {
    printk("Failed to setup sine and cosine cache: %d\n", err);
  }

  // NOTE: 12 is comming from ADC bit resolution, this should probably be
  // place in the KCONFIG
  // amp_bias = 3.3 / powf(2, 12);
  amp_bias = 3.3 / 2048;

  printk("setting up saadc\n");
  // k_msleep(1000);

  config_saadc();

  printk("setting up ppi\n");
  k_msleep(1000);

  config_ppi();

  printk("setup ppi\n");
  k_msleep(1000);

  irq_enable(DT_IRQN(DT_NODELABEL(adc)));

  printk("setting up timer\n");
  k_msleep(1000);

  config_timer();
printk("ADC setup finished, taking samples at %d Hz\n", CONFIG_RX_UPDATE_RATE);
  k_msleep(1000);

  while (1) {

    adc_sample(sample_buf_x, PIN_X, PIN_X_HIDDEN);
    adc_sample(sample_buf_y, PIN_Y, PIN_Y_HIDDEN);
    adc_sample(sample_buf_z, PIN_Z, PIN_Z_HIDDEN);

    tx_matched_filter(sample_buf_x, &cached_s_c, "x");
    tx_matched_filter(sample_buf_y, &cached_s_c, "y"); // seems to be 0.02
    tx_matched_filter(sample_buf_z, &cached_s_c, "z");

    read_rx_adc_data(&local_signal_data);
    printk("received magnet, X: amp: %.6f phase: %.6f | Y: amp: %.6f phase: %.6f | Z: amp: %.6f phase: %.6f\n",
           local_signal_data.adc_x_amp, local_signal_data.adc_x_phase,
           local_signal_data.adc_y_amp, local_signal_data.adc_y_phase,
           local_signal_data.adc_z_amp, local_signal_data.adc_z_phase);
    // TODO: update gain based on average value -> increase if below a certain
    // value, decrease if too high?

    k_msleep(1000 / CONFIG_RX_UPDATE_RATE);
  }
}
