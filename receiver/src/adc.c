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
// #define PIN_X NRF_SAADC_INPUT_AIN7 // pin 0.31
// TODO: move these into KCONFIG
// #define GAIN_X 1
// #define GAIN_Y 1
// #define GAIN_Z 1

#define DEBUG_GAIN_CAL 0
#define GAIN_X 1.109552f
#define GAIN_Y 0.963442f
#define GAIN_Z 0.935464f
#define ADC_DISCARD_SAMPLES 16

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define TWO_PI (2.0f * PI)
#define TX_FREQ_HZ 32000.0f
#define TIMER_FREQ_HZ 16000000.0f

#define TX_FREQ_HZ 32000.0f
#define TIMER_FREQ_HZ 16000000.0f

extern struct k_msgq rx_sync_msgq;

static float ticks_per_period = TIMER_FREQ_HZ / TX_FREQ_HZ;

typedef struct {
  float phase_offset_rad; // Calibrate this per axis
  float min_amp;          // Ignore sign updates below this amplitude
  float deadband_cos;     // Prevent sign flips near 90 deg uncertainty
  float last_sign;
} phase_axis_cal_t;

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
static uint32_t dummy_ts;

// this variable is a pointer to the full buffer. static uint32_t current_buffer
// = 0;

static struct rx_data_container_t local_sensor_data;

static struct rx_data_signal_t local_signal_data;
static struct data_container_t local_data_rx;
static struct timestamp_data_container_t local_tx_data;

static const struct gpio_dt_spec mux_s0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(mux_s0), gpios);
static const struct gpio_dt_spec mux_s1 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(mux_s1), gpios);
static const struct gpio_dt_spec mux_s2 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(mux_s2), gpios);

int16_t sample_buf_x[CONFIG_RX_SAMPLE_NUMBER] __attribute__((aligned(4))) = {0};
int16_t sample_buf_y[CONFIG_RX_SAMPLE_NUMBER] __attribute__((aligned(4))) = {0};
int16_t sample_buf_z[CONFIG_RX_SAMPLE_NUMBER] __attribute__((aligned(4))) = {0};

int16_t dummy_buf[CONFIG_RX_SAMPLE_NUMBER] __attribute__((aligned(4))) = {0};
int32_t val_mv;
int32_t err;

// NOTE: store 100(default) signal entries for phase and amp calculation
static float amp_bias;
struct cached_sin_cos_t cached_s_c = {.sine = {0}, .cosine = {0}};

static bool have_last_sync_dbg = false;
static uint32_t last_sync_ts = 0;
static float last_sync_phase = 0.0f;

// Start with zero offsets. You will calibrate these.
static phase_axis_cal_t phase_x = {
    .phase_offset_rad = -2.392f,
    .min_amp = 20.0f,
    .deadband_cos = 0.50f,
    .last_sign = 1.0f,
};

static phase_axis_cal_t phase_y = {
    .phase_offset_rad = -2.792f,
    .min_amp = 20.0f,
    .deadband_cos = 0.50f,
    .last_sign = 1.0f,
};

static phase_axis_cal_t phase_z = {
    .phase_offset_rad = -2.652f,
    .min_amp = 20.0f,
    .deadband_cos = 0.50f,
    .last_sign = 1.0f,
};

static float wrap_to_pi(float angle) {
  angle = fmodf(angle + PI, TWO_PI);
  if (angle < 0.0f) {
    angle += TWO_PI;
  }
  return angle - PI;
}

static inline uint32_t ticks_since(uint32_t now, uint32_t then) {
  return now - then;
}

static float wrap_pi(float x) {
  while (x > PI) {
    x -= 2.0f * PI;
  }

  while (x <= -PI) {
    x += 2.0f * PI;
  }

  return x;
}

// static float phase_advance_from_ticks(uint32_t dt_ticks) {
//   uint32_t ticks_per_period = (uint32_t)(TIMER_FREQ_HZ / TX_FREQ_HZ); // 500
//   uint32_t phase_ticks = dt_ticks % ticks_per_period;
//   return ((float)phase_ticks) * (2.0f * PI / (float)ticks_per_period);
// }

static float phase_advance_from_ticks(uint32_t dt_ticks) {
  const uint32_t ticks_per_period = 500; // 16 MHz / 32 kHz
  uint32_t phase_ticks = dt_ticks % ticks_per_period;
  return ((float)phase_ticks) * (2.0f * PI / 500.0f);
}

float phase_advance(uint32_t dt_ticks) {
  float cycles = ((float)dt_ticks * TX_FREQ_HZ) / TIMER_FREQ_HZ;
  return 2.0f * PI * (cycles - floorf(cycles));
}

static void saadc_handler(nrfx_saadc_evt_t const *p_event) {
  // if the next buffer is ready to be filled
  switch (p_event->type) {
  case NRFX_SAADC_EVT_DONE:
    k_sem_give(&adc_semaphore);
    break;
  }
}

static uint32_t last_sync_seq = 0;

static void debug_tx_sync_consistency(uint32_t seq, uint32_t ts, float phase) {
  if (!have_last_sync_dbg) {
    have_last_sync_dbg = true;
    last_sync_seq = seq;
    last_sync_ts = ts;
    last_sync_phase = phase;
    return;
  }

  if (seq == last_sync_seq) {
    printk("SYNCDBG duplicate seq=%u ignored\n", seq);
    return;
  }

  uint32_t dt = ts - last_sync_ts;
  float adv = phase_advance_from_ticks(dt);

  float pred_plus = wrap_to_pi(last_sync_phase + adv);
  float pred_minus = wrap_to_pi(last_sync_phase - adv);

  float err_plus = wrap_to_pi(phase - pred_plus);
  float err_minus = wrap_to_pi(phase - pred_minus);

  // printk("SYNCDBG seq=%u prev_seq=%u dt=%u dt_mod=%u "
  //        "prev=%.3f now=%.3f "
  //        "plus_err=%.3f plus_cos=%.3f "
  //        "minus_err=%.3f minus_cos=%.3f\n",
  //        seq, last_sync_seq, dt, dt % 500, last_sync_phase, phase, err_plus,
  //        cosf(err_plus), err_minus, cosf(err_minus));

  last_sync_seq = seq;
  last_sync_ts = ts;
  last_sync_phase = phase;
}

int config_saadc() {
  int err;
  // incredibly useful:
  // https://github.com/zephyrproject-rtos/hal_nordic/tree/master/nrfx/samples/src/nrfx_saadc/advanced_non_blocking_internal_timer

  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST,
              nrfx_saadc_irq_handler, 0, 0);

  irq_enable(NRFX_IRQ_NUMBER_GET(NRF_SAADC));

  err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
  if (err != 0) {
    printk("Error setting up SAADC: %d\n", err);
    k_msleep(1000);
    return err;
  }
  printk("Got passed init SAAADC\n");
  k_msleep(1000);

  nrfx_saadc_channel_t default_channel =
      NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0);
  default_channel.channel_config.acq_time = NRF_SAADC_ACQTIME_3US;
  nrfx_saadc_channel_config(&default_channel);

  printk("Got passed default channel\n");
  k_msleep(1000);

  nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;

  uint32_t channel_mask = nrfx_saadc_channels_configured_get();
  err = nrfx_saadc_advanced_mode_set(channel_mask, NRF_SAADC_RESOLUTION_12BIT,
                                     &adv_config, saadc_handler);

  nrf_saadc_channel_input_set(NRF_SAADC, 0, PIN_X, NRF_SAADC_INPUT_DISABLED);

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
      nrf_timer_event_address_get(adc_timer.p_reg, NRF_TIMER_EVENT_COMPARE0),
      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE),
      &gppi_handle);

  if (err != 0) {
    return err;
  }

  // capture time of finished ADC sampling
  err = nrfx_gppi_conn_alloc(
      nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_STARTED),
      nrf_timer_task_address_get(rx_timer.p_reg, NRF_TIMER_TASK_CAPTURE2),
      &gppi_adc_timestamp);

  if (err != 0) {
    return err;
  }

  err = nrfx_gppi_conn_alloc(
      nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_STARTED),
      nrf_timer_task_address_get(adc_timer.p_reg, NRF_TIMER_TASK_START),
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

static float sign_axis_from_phase(const char *name, float amp,
                                  float measured_phase, uint32_t axis_timestamp,
                                  uint32_t sync_timestamp,
                                  float tx_phase_at_sync,
                                  phase_axis_cal_t *cal) {
  float amp_abs = fabsf(amp);

  /*
   * Below this, phase is meaningless.
   * Your TX-off noise is around 1-12, so start with 20 or 25.
   */
  if (amp_abs < cal->min_amp) {
#if DEBUG_GAIN_CAL
    printk("PHDBG %s LOW amp=%.3f min=%.3f -> signed=0\n", name, amp_abs,
           cal->min_amp);
#endif
    return 0.0f;
  }

  uint32_t dt_ticks = axis_timestamp - sync_timestamp;

  float expected_phase =
      wrap_to_pi(tx_phase_at_sync + phase_advance_from_ticks(dt_ticks) +
                 cal->phase_offset_rad);

  float diff = wrap_to_pi(measured_phase - expected_phase);
  float c = cosf(diff);

  /*
   * Use phase only for sign.
   * Do not multiply by cos, because that creates fake gain mismatch.
   */
  float sign = cal->last_sign;

  if (fabsf(c) > cal->deadband_cos) {
    sign = (c >= 0.0f) ? 1.0f : -1.0f;
    cal->last_sign = sign;
  }

  float signed_amp = amp_abs * sign;

#if DEBUG_GAIN_CAL
  printk("PHDBG %s amp=%.3f dt_mod=%u meas=%.3f exp=%.3f diff=%.3f "
         "cos=%.3f sign=%.0f signed=%.3f\n",
         name, amp_abs, dt_ticks % 500, measured_phase, expected_phase, diff, c,
         sign, signed_amp);
#endif

  return signed_amp;
}
void phase_sync_signs(void) {
  float tx_phase = local_tx_data.tx_phase;
  uint32_t sync_time = local_timestamp;

  local_signal_data.adc_x_amp = sign_axis_from_phase(
      "x", local_signal_data.adc_x_amp, local_signal_data.adc_x_phase,
      adc_timestamp_x, sync_time, tx_phase, &phase_x);

  local_signal_data.adc_y_amp = sign_axis_from_phase(
      "y", local_signal_data.adc_y_amp, local_signal_data.adc_y_phase,
      adc_timestamp_y, sync_time, tx_phase, &phase_y);

  local_signal_data.adc_z_amp = sign_axis_from_phase(
      "z", local_signal_data.adc_z_amp, local_signal_data.adc_z_phase,
      adc_timestamp_z, sync_time, tx_phase, &phase_z);

  update_rx_adc_data(local_signal_data.adc_x_amp, local_signal_data.adc_x_phase,
                     "x");

  update_rx_adc_data(local_signal_data.adc_y_amp, local_signal_data.adc_y_phase,
                     "y");

  update_rx_adc_data(local_signal_data.adc_z_amp, local_signal_data.adc_z_phase,
                     "z");
}

int tx_matched_filter(int16_t *signal_buf, struct cached_sin_cos_t *cached_s_c,
                      char *axis) {
  float cos_accumulation = 0.0f;
  float sin_accumulation = 0.0f;
  float amp = 0.0f;
  float phase = 0.0f;
  int32_t dc_sum = 0;

  const int start = ADC_DISCARD_SAMPLES;
  const int n = CONFIG_RX_SAMPLE_NUMBER - ADC_DISCARD_SAMPLES;

  if (n <= 0) {
    return -1;
  }

  for (int i = start; i < CONFIG_RX_SAMPLE_NUMBER; i++) {
    dc_sum += signal_buf[i];
  }

  float true_dc_offset = (float)dc_sum / (float)n;

  for (int i = start; i < CONFIG_RX_SAMPLE_NUMBER; i++) {
    float ac_wave_signal = (float)signal_buf[i] - true_dc_offset;

    sin_accumulation += cached_s_c->sine[i] * ac_wave_signal;
    cos_accumulation += cached_s_c->cosine[i] * ac_wave_signal;
  }

  amp = sqrtf((sin_accumulation * sin_accumulation) +
              (cos_accumulation * cos_accumulation));

  amp = (amp / (float)n) * 2.0f;
  amp = (amp / 4096.0f) * 3.6f;
  amp = amp * 1000.0f;

  phase = atan2f(-sin_accumulation, cos_accumulation);

  update_rx_adc_data(amp, phase, axis);

  return 0;
}

void switch_channel(uint8_t channel_num) {
  gpio_pin_set_dt(&mux_s0, (channel_num & 0x01) ? 1 : 0);
  gpio_pin_set_dt(&mux_s1, (channel_num & 0x02) ? 1 : 0);
  gpio_pin_set_dt(&mux_s2, (channel_num & 0x04) ? 1 : 0);

  // k_busy_wait(250);
  k_busy_wait(1000);
}

void adc_sample(uint8_t channel_num, nrf_saadc_value_t *buf,
                uint32_t *timestamp_out) {
  switch_channel(channel_num);

  nrfx_timer_disable(&adc_timer);
  nrfx_timer_clear(&adc_timer);

  // k_sem_reset(&adc_semaphore);

  nrfx_err_t err = nrfx_saadc_buffer_set(buf, CONFIG_RX_SAMPLE_NUMBER);
  if (err != 0) {
    printk("CRITICAL ERROR: Buffer set failed with code %d\n", err);
    k_msleep(5000);
    return;
  }

  nrfx_saadc_mode_trigger();

  k_sem_take(&adc_semaphore, K_FOREVER);

  *timestamp_out = nrfx_timer_capture_get(&rx_timer, NRF_TIMER_CC_CHANNEL2);
  // *timestamp_out = nrf_timer_cc_get(rx_timer.p_reg, NRF_TIMER_CC_CHANNEL2);

  nrfx_timer_disable(&adc_timer);
}

int setup_mux_pins() {
  int err;
  err = gpio_pin_configure_dt(&mux_s0, GPIO_OUTPUT_INACTIVE);
  if (err != 0) {
    printk("error setting up mux: %d", err);
    k_msleep(2000);
    return err;
  }
  err = gpio_pin_configure_dt(&mux_s1, GPIO_OUTPUT_INACTIVE);
  if (err != 0) {
    printk("error setting up mux: %d", err);
    k_msleep(2000);
    return err;
  }
  err = gpio_pin_configure_dt(&mux_s2, GPIO_OUTPUT_INACTIVE);
  if (err != 0) {
    printk("error setting up mux: %d", err);
    k_msleep(2000);
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
    struct rx_sync_ref_t sync;

    k_msgq_get(&rx_sync_msgq, &sync, K_FOREVER);

    local_timestamp = sync.timestamp;
    local_tx_data.tx_phase = sync.tx_phase;
    local_tx_data.q0 = sync.q0;
    local_tx_data.q1 = sync.q1;
    local_tx_data.q2 = sync.q2;
    local_tx_data.q3 = sync.q3;

    debug_tx_sync_consistency(sync.seq, sync.timestamp, sync.tx_phase);
    // printk("ADC read sync timestamp=%u\n", local_timestamp);
    // read_tx_data(&local_tx_data);
    //
    // need to implmenet 2 stage change
    // vout4 L1
    adc_sample(0, dummy_buf, &dummy_ts);
    adc_sample(0, sample_buf_x, &adc_timestamp_x);

    adc_sample(3, dummy_buf, &dummy_ts);
    adc_sample(3, sample_buf_y, &adc_timestamp_y);

    adc_sample(5, dummy_buf, &dummy_ts);
    adc_sample(5, sample_buf_z, &adc_timestamp_z);

    // printk("TIMES sync=%u x=%u y=%u z=%u dx=%u dy=%u dz=%u\n",
    // local_timestamp,
    //        adc_timestamp_x, adc_timestamp_y, adc_timestamp_z,
    //        (uint32_t)(adc_timestamp_x - local_timestamp),
    //        (uint32_t)(adc_timestamp_y - local_timestamp),
    //        (uint32_t)(adc_timestamp_z - local_timestamp));

    tx_matched_filter(sample_buf_x, &cached_s_c, "x");
    tx_matched_filter(sample_buf_y, &cached_s_c, "y");
    tx_matched_filter(sample_buf_z, &cached_s_c, "z");
    //
    read_rx_adc_data(&local_signal_data);
    //
    phase_sync_signs();
    //
    read_rx_adc_data(&local_signal_data);
    read_data(&local_data_rx);

#if DEBUG_GAIN_CAL
    printk("CAL,%u,%.6f,%.6f,%.6f,"
           "%.6f,%.6f,%.6f,"
           "%.6f,%.6f,%.6f,%.6f,"
           "%.6f,%.6f,%.6f,%.6f\n",
           sync.seq, local_signal_data.adc_x_amp, local_signal_data.adc_y_amp,
           local_signal_data.adc_z_amp, local_signal_data.adc_x_phase,
           local_signal_data.adc_y_phase, local_signal_data.adc_z_phase,
           local_data_rx.q0, local_data_rx.q1, local_data_rx.q2,
           local_data_rx.q3, local_tx_data.q0, local_tx_data.q1,
           local_tx_data.q2, local_tx_data.q3);
#endif
    //
    float bx = -(local_signal_data.adc_x_amp * GAIN_X);
    float by = -(local_signal_data.adc_y_amp * GAIN_Y);
    float bz = (local_signal_data.adc_z_amp * GAIN_Z);

    float bmag = sqrtf(bx * bx + by * by + bz * bz);

    if (!isfinite(bmag) || bmag < 20.0f) {
      printk("NO_TX Brx %.3f %.3f %.3f | |Brx| %.3f\n", bx, by, bz, bmag);
      continue;
    }
    struct solver_packet_t pos_packet = {
        .bx = bx,
        .by = by,
        .bz = bz,
        .q0 = local_tx_data.q0,
        .q1 = local_tx_data.q1,
        .q2 = local_tx_data.q2,
        .q3 = local_tx_data.q3,
    };
    //
    k_msgq_put(&positioning_queue, &pos_packet, K_NO_WAIT);
  }
}
