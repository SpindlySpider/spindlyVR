#include "data_handle.h"
#include "hal/nrf_timer.h"
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

#include "adc.h"

#define PI 3.14159265f
#define SAADC_INPUT_PIN NRFX_ANALOG_EXTERNAL_AIN0 // pin 0.02

#define SAADC_SAMPLE_INTERVAL_US 50
#define INTERUPT_PRIORITY 6

K_SEM_DEFINE(adc_semaphore, 0, 1);

// setup ADC pin for use
static nrfx_saadc_channel_t channel =
    NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN, 0);
// setup timer
static nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(2));

// this variable is a pointer to the full buffer.
static int16_t *current_buffer_ptr = NULL;
static uint32_t current_buffer = 0;

// use double buffer to store readings from ADC, so we can continually read
int16_t sample_buf[2][CONFIG_TX_SAMPLE_NUMBER];
int32_t val_mv;
int32_t err;

// NOTE: store 100(default) signal entries for phase and amp calculation
static float amp_bias;
struct cached_sin_cos_t cached_s_c = {.sine = {}, .cosine = {}};

static void saadc_handler(nrfx_saadc_evt_t const *p_event) {
  // if the next buffer is ready to be filled
  switch (p_event->type) {
  case NRFX_SAADC_EVT_BUF_REQ:
    nrfx_saadc_buffer_set(sample_buf[(current_buffer++) % 2],
                          CONFIG_TX_SAMPLE_NUMBER);
    break;
  case NRFX_SAADC_EVT_DONE:
    current_buffer_ptr = p_event->data.done.p_buffer;
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
    return err;
  }

  // setup channel for 3 microsecond
  channel.channel_config.acq_time = NRF_SAADC_ACQTIME_3US;
  channel.channel_config.gain = NRF_SAADC_GAIN1_6;
  channel.channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;
  channel.channel_config.burst = NRF_SAADC_BURST_DISABLED;

  err = nrfx_saadc_channel_config(&channel);

  if (err != 0) {
    printk("Error setting up SAADC channel: %d\n", err);
    return err;
  }

  nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;

  uint32_t channel_mask = nrfx_saadc_channels_configured_get();
  err = nrfx_saadc_advanced_mode_set(channel_mask, NRF_SAADC_RESOLUTION_12BIT,
                                     &adv_config, saadc_handler);

  if (err != 0) {
    return err;
  }

  // init buffer
  nrfx_saadc_buffer_set(sample_buf[0], CONFIG_TX_SAMPLE_NUMBER);
  nrfx_saadc_buffer_set(sample_buf[1], CONFIG_TX_SAMPLE_NUMBER);

  err = nrfx_saadc_offset_calibrate(saadc_handler);

  return 0;
}

int config_timer() {
  // incredibly useful:
  // https://github.com/zephyrproject-rtos/hal_nordic/tree/master/nrfx/samples/src/nrfx_timer

  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER2), IRQ_PRIO_LOWEST,
              nrfx_timer_irq_handler, &timer_inst, 0);

  uint32_t frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);

  nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(frequency);

  err = nrfx_timer_init(&timer_inst, &config, NULL);

  nrfx_timer_clear(&timer_inst);

  // convert kilohertz to micro seconds
  uint32_t khz_to_us = (uint32_t)(1000 / CONFIG_TX_ADC_FREQUENCY) ;
  uint32_t desired_ticks = nrfx_timer_us_to_ticks(&timer_inst, khz_to_us);

  nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  nrfx_timer_enable(&timer_inst);
  printk("Started timer\n");
  k_msleep(100);

  return 0;
}

int config_ppi() {
  // set up generic PPI
  nrfx_gppi_handle_t gppi_handle;
  nrfx_gppi_handle_t gppi_start;

  // connect timer event compare0 to saadc task sample (take a sample each time
  // timer ticks)
  int err = nrfx_gppi_conn_alloc(
      nrf_timer_event_address_get(NRF_TIMER2, NRF_TIMER_EVENT_COMPARE0),
      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE),
      &gppi_handle);

  if (err != 0) {
    return err;
  }

  // when buffer is full start new SAADC task
  err = nrfx_gppi_conn_alloc(
      nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START), &gppi_start);
  if (err != 0) {
    return err;
  }

  // enable this connection
  nrfx_gppi_conn_enable(gppi_start);
  nrfx_gppi_conn_enable(gppi_handle);
  return 0;
}

int tx_matched_filter(int16_t *signal_buf,
                      struct cached_sin_cos_t *cached_s_c) {
  // NOTE: signal buf should be an array of samples from ADC
  float sin_accumulation = 0.0f;
  float cos_accumulation = 0.0f;
  float amp = 0.0f;
  float phase = 0.0f;
  int32_t dc_sum = 0;
  for (int i = 0; i < CONFIG_TX_SAMPLE_NUMBER; i++) {
    dc_sum += signal_buf[i];
  }
  float true_dc_offset = (float)dc_sum / (float)CONFIG_TX_SAMPLE_NUMBER;

  for (int i = 0; i < CONFIG_TX_SAMPLE_NUMBER; i++) {
    float ac_wave_signal = (float)signal_buf[i] - true_dc_offset;

    sin_accumulation += (cached_s_c->sine[i] * ac_wave_signal);
    cos_accumulation += (cached_s_c->cosine[i] * ac_wave_signal);
  }
  // int32_t millivolts = (signal_buf[0] * 3600) / 4096;
  // printk("ADC Reading: %d mV\n",millivolts);
  amp = ((sqrtf(powf(sin_accumulation, 2) + powf(cos_accumulation, 2)) /
          CONFIG_TX_SAMPLE_NUMBER));
  amp = amp * 2.0f;
  amp = (amp / 4096.0f) * 3.6f;

  phase = atan2f(sin_accumulation, cos_accumulation);

  // update container
  update_signal_data(amp, phase);

  return 0;
}

int setup_sin_cos_cache(struct cached_sin_cos_t *cached_s_c) {
  // cache values of target sin and cos wave for use during execution

  for (int i = 0; i < CONFIG_TX_SAMPLE_NUMBER; i++) {
    // *1000 because in KHZ
    float time = (float)i / (CONFIG_TX_ADC_FREQUENCY * 1000.0f);
    float angle = 2.0f * PI * (CONFIG_TX_PWM_FREQUENCY * 1000.0f) * time;
    cached_s_c->sine[i] = sin(angle);
    cached_s_c->cosine[i] = cos(angle);
  }

  // NOTE: 12 is comming from ADC bit resolution, this should probably be
  // place in the KCONFIG
  amp_bias = 3.3 / powf(2, 12);

  // NOTE: need to store how long to sleep
  return 0;
}

// TODO: make a thread function which keeps track of signal buffer and updates
// shared data var would need to call read ADC a couple of times and then save
// result in buffer

void start_adc_thread(void *, void *, void *) {
  // this function acts as entry point for starting a thread
  int err;

  printk("setting up adc\n");

  err = setup_sin_cos_cache(&cached_s_c);

  if (err != 0) {
    printk("Failed to setup sine and cosine cache: %d\n", err);
    // NOTE: should continue and read to read again
  }

  printk("setting up saadc\n");

  config_saadc();

  printk("setting up ppi\n");

  config_ppi();

  printk("setup ppi\n");

  irq_enable(DT_IRQN(DT_NODELABEL(adc)));

  nrfx_saadc_mode_trigger();

  printk("setting up timer\n");
  k_msleep(1000);

  // TODO: this can be improved, cycles are wasted constantly scanning ADC,
  // if we set an event in while loop to activate timer to initiate ADC readings that would be better
  config_timer();

  printk("ADC setup finished, taking samples at %d Hz\n",
         CONFIG_TX_BROADCAST_FREQUENCY);
  while (1) {
    // use semaphore to know when samples ready
    k_sem_take(&adc_semaphore, K_FOREVER);

    tx_matched_filter(current_buffer_ptr, &cached_s_c);
    k_msleep(1000 / CONFIG_TX_BROADCAST_FREQUENCY);
  }
}
