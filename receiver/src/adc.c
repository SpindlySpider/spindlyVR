#include "data_handle.h"
#include "hal/nrf_saadc.h"
#include "hal/nrf_timer.h"
#include "nrfx_templates_config.h"
#include "timer.h"
#include <helpers/nrfx_gppi.h>
#include <math.h>
#include <zephyr/drivers/gpio.h>

#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// https://github.com/NordicPlayground/nRF52-ADC-examples/tree/master/nrfx_saadc_multi_channel_ppi

// pin out from here: https://nicekeyboards.com/docs/nice-nano/pinout-schematic/
// ADC pins
#define PIN_X NRF_SAADC_INPUT_AIN0 // pin 0.02
// TODO: move these into KCONFIG
#define GAIN_X 1
#define GAIN_Y 2.2307
#define GAIN_Z 1

K_SEM_DEFINE(adc_semaphore, 0, 1);

// used to pass messages to positioning thread
extern struct k_msgq positioning_queue;
extern struct k_sem radio_sync_sem;
// setup timer
// static nrfx_timer_t adc_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(3));

// used for calculating phase for sign
static uint32_t local_timestamp;
// static uint32_t adc_timestamp;
static uint32_t adc_timestamp_x;
static uint32_t adc_timestamp_y;
static uint32_t adc_timestamp_z;

// this variable is a pointer to the full buffer. static uint32_t current_buffer
// = 0;

static struct rx_data_signal_t local_signal_data;
static struct data_container_t local_tx_data;

static const struct gpio_dt_spec mux_s0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(mux_s0), gpios);
static const struct gpio_dt_spec mux_s1 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(mux_s1), gpios);
static const struct gpio_dt_spec mux_s2 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(mux_s2), gpios);

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

int config_adc_ppi() {
  // set up generic PPI
  nrfx_gppi_handle_t gppi_handle;
  nrfx_gppi_handle_t gppi_adc_timestamp;
  nrfx_gppi_handle_t gppi_timer_start;

  // connect timer event compare0 to saadc task sample (take a sample each
  // time timer ticks)
  int err = nrfx_gppi_conn_alloc(
      nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE0),
      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE),
      &gppi_handle);

  if (err != 0) {
    return err;
  }

  // capture time of finished ADC sampling
  err = nrfx_gppi_conn_alloc(
      nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_STARTED),
      nrf_timer_task_address_get(NRF_TIMER4, NRF_TIMER_TASK_CAPTURE2),
      &gppi_adc_timestamp);

  if (err != 0) {
    return err;
  }

  nrfx_gppi_conn_alloc(
      nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_STARTED),
      nrf_timer_task_address_get(NRF_TIMER3, NRF_TIMER_TASK_START),
      &gppi_timer_start);

  if (err != 0) {
    return err;
  }
  // enable this connection
  nrfx_gppi_conn_enable(gppi_handle);
  nrfx_gppi_conn_enable(gppi_timer_start);
  nrfx_gppi_conn_enable(gppi_adc_timestamp);
  return 0;
}

float angle_diff(float angle1, float angle2) {
  // finds shortest distance between input angles (radians)
  float diff = fmodf(fabs(angle1 - angle2), 2.0f * PI);
  if (diff > PI) {
    diff = 2.0f * PI - diff;
  }
  return diff;
}

uint32_t get_safe_delta(uint32_t later_time, uint32_t earlier_time) {
  if (later_time < earlier_time) {
    return (4294967000 - earlier_time) + later_time;
  }
  return later_time - earlier_time;
}

// Helper function for safe signed angle wrapping (-PI to PI)
float wrap_to_pi(float angle) {
  angle = fmodf(angle, 2.0f * PI);
  if (angle > PI)
    angle -= 2.0f * PI;
  if (angle < -PI)
    angle += 2.0f * PI;
  return angle;
}

void phase_sync_signs() {
  float saved_tx_phase = local_tx_data.tx_phase;
  uint32_t radio_time = local_timestamp;

  uint32_t u_delta_x = get_safe_delta(adc_timestamp_x, radio_time);
  uint32_t u_delta_y = get_safe_delta(adc_timestamp_y, radio_time);
  uint32_t u_delta_z = get_safe_delta(adc_timestamp_z, radio_time);

  float delta_phase_x = (float)(u_delta_x % 500) * (2.0f * PI / 500.0f);
  float delta_phase_y = (float)(u_delta_y % 500) * (2.0f * PI / 500.0f);
  float delta_phase_z = (float)(u_delta_z % 500) * (2.0f * PI / 500.0f);

  float expected_x_phase = fmodf(saved_tx_phase + delta_phase_x, 2.0f * PI);
  float expected_y_phase = fmodf(saved_tx_phase + delta_phase_y, 2.0f * PI);
  float expected_z_phase = fmodf(saved_tx_phase + delta_phase_z, 2.0f * PI);

  // Calculate RAW difference (expected - measured) keeping the sign!
  float x_raw = expected_x_phase - local_signal_data.adc_x_phase;
  float y_raw = expected_y_phase - local_signal_data.adc_y_phase;
  float z_raw = expected_z_phase - local_signal_data.adc_z_phase;

  // --- THE COSTAS LOOP (Closed-Loop PLL) ---
  static float global_clock_drift = 0.0f;

  // Step A: Apply the global drift compensation
  float x_diff = wrap_to_pi(x_raw - global_clock_drift);
  float y_diff = wrap_to_pi(y_raw - global_clock_drift);
  float z_diff = wrap_to_pi(z_raw - global_clock_drift);

  // Step B: Find distance to nearest 0 or PI.
  // Multiplying by 2 turns PI into 2PI (which wraps to 0). Divide by 2 to
  // restore scale!
  float pll_error = wrap_to_pi(x_diff * 2.0f) / 2.0f;

  // Step C: Nudge the drift compensator
  global_clock_drift = wrap_to_pi(global_clock_drift + (pll_error * 0.05f));

  // Step D: Use absolute value just for the sign check thresholds
  float abs_x_diff = fabsf(x_diff);
  float abs_y_diff = fabsf(y_diff);
  float abs_z_diff = fabsf(z_diff);

  // --- CALIBRATION LOG ---
  // printk("CALIBRATION -> X Diff: %.3f | Y Diff: %.3f | Z Diff: %.3f\n",
  //        abs_x_diff, abs_y_diff, abs_z_diff);

  // 7. Flip signs based on the 90-degree (1.57 rad) threshold
  local_signal_data.adc_x_amp = (abs_x_diff > 1.5708f)
                                    ? -fabsf(local_signal_data.adc_x_amp)
                                    : fabsf(local_signal_data.adc_x_amp);
  local_signal_data.adc_y_amp = (abs_y_diff > 1.5708f)
                                    ? -fabsf(local_signal_data.adc_y_amp)
                                    : fabsf(local_signal_data.adc_y_amp);
  local_signal_data.adc_z_amp = (abs_z_diff > 1.5708f)
                                    ? -fabsf(local_signal_data.adc_z_amp)
                                    : fabsf(local_signal_data.adc_z_amp);

  update_rx_adc_data(local_signal_data.adc_x_amp, local_signal_data.adc_x_phase,
                     "x");
  update_rx_adc_data(local_signal_data.adc_y_amp, local_signal_data.adc_y_phase,
                     "y");
  update_rx_adc_data(local_signal_data.adc_z_amp, local_signal_data.adc_z_phase,
                     "z");
}
int tx_matched_filter(int16_t *signal_buf, struct cached_sin_cos_t *cached_s_c,
                      char *axis) {
  // TODO: this can be moved into a common file and values can be passed in, so
  // it is common between receiver and transmitter
  // NOTE: signal buf should be an
  // array of samples from ADC float sin_accumulation = 0.0f;
  float cos_accumulation = 0.0f;
  float sin_accumulation = 0.0f;
  float amp = 0.0f;
  float phase = 0.0f;
  int32_t dc_sum = 0;

  for (int i = 0; i < CONFIG_RX_SAMPLE_NUMBER; i++) {
    dc_sum += signal_buf[i];
  }
  float true_dc_offset = (float)dc_sum / (float)CONFIG_RX_SAMPLE_NUMBER;

  for (int i = 0; i < CONFIG_RX_SAMPLE_NUMBER; i++) {
    float ac_wave_signal =
        (float)signal_buf[i] -
        true_dc_offset; // float ac_wave_signal = (float)signal_buf[i];
    sin_accumulation += (cached_s_c->sine[i] * ac_wave_signal);
    // sin_accumulation += (cached_s_c->sine[i] * signal_buf[i]);
    cos_accumulation += (cached_s_c->cosine[i] * ac_wave_signal);
    // cos_accumulation += (cached_s_c->cosine[i] * signal_buf[i]);
  }

  // printk("Raw Data: ");
  // for (int i = 0; i < 20; i++) {
  //   printk("%d, ", signal_buf[i]);
  // }
  // printk("\n");

  // int32_t millivolts = (signal_buf[0] * 3600) / 4096;
  // printk("ADC Reading: %d mV\n", millivolts);

  amp = ((sqrtf(powf(sin_accumulation, 2) + powf(cos_accumulation, 2)) /
          CONFIG_RX_SAMPLE_NUMBER));

  amp = amp * 2.0f;
  // normalize after gain
  // amp = (amp / 2048.0f) * 0.15f;
  amp = (amp / 4096.0f) * 3.6f;
  // amp = amp * amp_bias;

  amp = amp * 1000.0f;

  phase = atan2f(-sin_accumulation, cos_accumulation);

  // update container
  update_rx_adc_data(amp, phase, axis);

  return 0;
}

void switch_channel(uint8_t channel_num) {
  gpio_pin_set_dt(&mux_s0, (channel_num & 0x01) ? 1 : 0);
  gpio_pin_set_dt(&mux_s1, (channel_num & 0x02) ? 1 : 0);
  gpio_pin_set_dt(&mux_s2, (channel_num & 0x04) ? 1 : 0);

  k_busy_wait(150);
}

void adc_sample(uint8_t channel_num, nrf_saadc_value_t *buf) {
  // switch channel
  switch_channel(channel_num);

  nrfx_timer_disable(&adc_timer);
  nrfx_timer_clear(&adc_timer);
  nrfx_saadc_buffer_set(buf, CONFIG_RX_SAMPLE_NUMBER);
  nrfx_saadc_mode_trigger();
  // nrfx_timer_enable(&adc_timer);
  k_sem_take(&adc_semaphore, K_FOREVER);
  nrfx_timer_disable(&adc_timer);
}
int setup_mux_pins() {
  int err;
  err = gpio_pin_configure_dt(&mux_s0, GPIO_OUTPUT_INACTIVE);
  if (err != 0) {
    printk("error setting up mux: %d", err);
    return err;
  }
  err = gpio_pin_configure_dt(&mux_s1, GPIO_OUTPUT_INACTIVE);
  if (err != 0) {
    printk("error setting up mux: %d", err);
    return err;
  }
  err = gpio_pin_configure_dt(&mux_s2, GPIO_OUTPUT_INACTIVE);
  if (err != 0) {
    printk("error setting up mux: %d", err);
    return err;
  }
  return 0;
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

  config_adc_ppi();

  printk("setup ppi\n");
  k_msleep(1000);

  printk("ADC setup finished, taking samples at %d Hz\n",
         CONFIG_RX_UPDATE_RATE);
  k_msleep(1000);

  printk("setting up mux pins\n");
  setup_mux_pins();
  k_msleep(1000);

  while (1) {
    k_sem_take(&radio_sync_sem, K_FOREVER);
    // printk("taken semphore\n");
    read_timestamp(&local_timestamp);
    read_tx_data(&local_tx_data);

    // need to implmenet 2 stage change
    adc_sample(5, sample_buf_x);
    adc_timestamp_x = nrfx_timer_capture_get(&rx_timer, NRF_TIMER_CC_CHANNEL2);

    adc_sample(3, sample_buf_y);
    adc_timestamp_y = nrfx_timer_capture_get(&rx_timer, NRF_TIMER_CC_CHANNEL2);

    adc_sample(2, sample_buf_z);
    adc_timestamp_z = nrfx_timer_capture_get(&rx_timer, NRF_TIMER_CC_CHANNEL2);

    tx_matched_filter(sample_buf_x, &cached_s_c, "x");
    tx_matched_filter(sample_buf_y, &cached_s_c, "y"); // seems to be 0.02
    tx_matched_filter(sample_buf_z, &cached_s_c, "z");

    read_rx_adc_data(&local_signal_data);

    // phase sync here
    phase_sync_signs();

    read_rx_adc_data(&local_signal_data);
    // TODO: wrap this in some config print statement?
    //
    // printk("received magnet, X: amp: %.6f phase: %.6f | Y: amp: %.6f phase: "
    //        "%.6f | Z: amp: %.6f phase: %.6f\n",
    //        local_signal_data.adc_x_amp, local_signal_data.adc_x_phase,
    //        local_signal_data.adc_y_amp, local_signal_data.adc_y_phase,
    //        local_signal_data.adc_z_amp, local_signal_data.adc_z_phase);

    // NOTE: values must be calibrated to your specific coil
    struct solver_packet_t pos_packet = {
        .bx = local_signal_data.adc_x_amp * GAIN_X,
        .by = local_signal_data.adc_y_amp * GAIN_Y,
        .bz = local_signal_data.adc_z_amp * GAIN_Z};

    k_msgq_put(&positioning_queue, &pos_packet, K_NO_WAIT);

    // // TODO: update gain based on average value -> increase if below a
    // certain value, decrease if too high?
    // even if the SNR is bad, with enough samples it should work, additionally it should run in psuedo differntial mode

    // k_msleep(1000 / CONFIG_RX_UPDATE_RATE);
  }
}
