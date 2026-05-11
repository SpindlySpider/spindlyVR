#include "adc.h"
#include "data_handle.h"
#include "hal/nrf_saadc.h"
#include "hal/nrf_timer.h"
#include "nrfx_templates_config.h"
#include "orientation_handle.h"
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
#define GAIN_X 1.2184578168707212f
#define GAIN_Y 0.8938119254701037f
#define GAIN_Z 0.9429631550167387f
#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define TWO_PI (2.0f * PI)

extern struct k_msgq rx_sync_msgq;

K_SEM_DEFINE(adc_semaphore, 0, 1);

// used to pass messages to positioning thread
extern struct k_msgq positioning_queue;
extern struct k_sem radio_sync_sem;

// used for calculating phase for sign
static uint32_t local_timestamp;
static uint32_t adc_timestamp_x;
static uint32_t adc_timestamp_y;
static uint32_t adc_timestamp_z;

static float synced_phase;
static float raw_phase;

// needed for passing to pos algo?
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

// number of samples to hold the sign on
static uint8_t sign_hold_number = 4;
// array of each coils pointers
static struct coil_data_storage coils_arr[3] = {
    {&local_signal_data.adc_x_amp, &local_signal_data.adc_x_phase, "x",
     &adc_timestamp_x, 0.0f, 36.0f, 0.15, 1, 0},
    {&local_signal_data.adc_y_amp, &local_signal_data.adc_y_phase, "y",
     &adc_timestamp_y, -1.4f, 36.0f, 0.15, 1, 0},
    {&local_signal_data.adc_z_amp, &local_signal_data.adc_z_phase, "z",
     &adc_timestamp_z, -0.10f, 36.0f, 0.15, 1, 0}};

// the last reference coil, use to determine if the reference coil should be
// changed, integer because it directly relates to the index above
static uint8_t last_reference_coil = 0;
// what is the minimum amount of difference between stronger amp of the coil to
// change to and the current coil
//
static float min_threshold_coil_change = 20.f;

int16_t sample_buf_x[CONFIG_RX_SAMPLE_NUMBER] = {0};
int16_t sample_buf_y[CONFIG_RX_SAMPLE_NUMBER] = {0};
int16_t sample_buf_z[CONFIG_RX_SAMPLE_NUMBER] = {0};

int32_t val_mv;
int32_t err;

// NOTE: store 100(default) signal entries for phase and amp calculation
static float amp_bias;
struct cached_sin_cos_t cached_s_c = {.sine = {0}, .cosine = {0}};

static float wrap_to_pi(float angle) {
  // add PI to shift angle from -pi to 0, 2pi
  angle = fmodf(angle + PI, TWO_PI);
  if (angle < 0.0f) {
    // rotate the angle back to be positive 2 pi range
    angle += TWO_PI;
  }
  // shifts value back to -pi to pi range
  return angle - PI;
}

static float phase_advance_from_ticks(int32_t dt_ticks) {
  // takes int 32 as it could be negative
  const uint32_t ticks_per_period = 500; // 16 MHz / 32 kHz
  // find only the remainder in phase
  int32_t phase_ticks = dt_ticks % ticks_per_period;
  // return how much that remainder would fast forward or rewind
  return ((float)phase_ticks) * (2.0f * PI / 500.0f);
}

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

  if (err != 0) {
    return err;
  }
  printk("Got passed advanced mode set\n");
  k_msleep(1000);

  printk("Got passed buffer set\n");
  k_msleep(1000);

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

static float sign_axis_from_phase(struct coil_data_storage *data,
                                  uint32_t sync_timestamp, float target_phase) {
  // target phase could be TX, or the internal reference coil
  // sync timestamp will either be when tx coil was received or the reference
  // coil timestamp
  float amp_abs = fabsf(*data->amp);
  float phase_offset = 0.0f;

  int32_t dt_ticks = *data->axis_timestamp - sync_timestamp;

  // calculate the expected phase, may need to be rewind or speed up, if
  // referncing internal coils, maybe ignore offset?
  float expected_phase =
      wrap_to_pi(target_phase + phase_advance_from_ticks(dt_ticks));

  if (strcmp(data->axis, coils_arr[last_reference_coil].axis) == 0) {
    // if this is the reference coil, see what the synced and expected phase is
    synced_phase = expected_phase;
    phase_offset = data->phase_offset_rad;
  }

  // NOTE: This conditionally applies the phase_offset radians, if this is the
  // refernce coil, otherwise ignore

  float diff = wrap_to_pi((*data->phase - expected_phase) + phase_offset);
  // finding the difference between the expected phase and the measurement

  // cosine used to determine sign.
  // If the phases are aligned then the diff will be 0 or close to.
  // This will make C postive 1 otherwise if the phase are not
  // in algiment they will be around -pi or +pi (roughly the same angle) and
  // will give -1
  float c = cosf(diff);

  float sign = data->last_sign;

  // if the difference is more positive then swap sign to + otherwise -
  float potential_sign = (c >= 0.0f) ? 1.0f : -1.0f;
  // // deadband to make sure there is enough of a difference to change sign
  if (fabsf(c) > data->deadband_cos && amp_abs > data->min_amp) {
    if (potential_sign != data->last_sign) {
      data->hold_number++;
      if (data->hold_number > sign_hold_number) {
        // sign = (c >= 0.0f) ? 1.0f : -1.0f;
        data->last_sign = potential_sign;
        data->hold_number = 0;
      }
    } else {
      // its the same sign so reset the hold value
      data->hold_number = 0;
    }
  }

  return amp_abs * sign;
  ;
}

void phase_sync_signs(void) {
  float tx_phase = local_tx_data.tx_phase;
  // when the TX was received
  uint32_t tx_sync_time = local_timestamp;

  // find which coil has the largest amplitude
  for (int i = 0; i < 3; i++) {
    // if the difference between this coil (x,y,z) and the last reference coil
    // is greater than the minimum threshold change
    // should be postive since this is before the sign is applied
    if (*coils_arr[i].amp - *coils_arr[last_reference_coil].amp >
        min_threshold_coil_change) {
      // switch the coil, this should ensure that the largest coil is always
      // selected, change the intger
      last_reference_coil = i;
    }
  }
  // from here the last reference coil has the coil we will compare tx phase to

  // get the reference coil data
  struct coil_data_storage data = coils_arr[last_reference_coil];
  // update the reference coils sign first
  *data.amp = sign_axis_from_phase(&data, tx_sync_time, tx_phase);

  // update coils from internal reference coil
  for (int i = 0; i < 3; i++) {
    if (i == last_reference_coil) {
      // skip reference coil!
      continue;
    }
    // update value at coils array amp (local_signal_data.amp)
    // compare this sampled coil to the sample time of the reference coil
    *coils_arr[i].amp =
        sign_axis_from_phase(&coils_arr[i], *data.axis_timestamp, *data.phase);
  }

  // this can probbaly be compressed as well, using a for loop
  update_rx_adc_data(local_signal_data.adc_x_amp, local_signal_data.adc_x_phase,
                     "x");

  update_rx_adc_data(local_signal_data.adc_y_amp, local_signal_data.adc_y_phase,
                     "y");

  update_rx_adc_data(local_signal_data.adc_z_amp, local_signal_data.adc_z_phase,
                     "z");
}

int matched_filter(int16_t *signal_buf, struct cached_sin_cos_t *cached_s_c,
                   char *axis) {

  float cos_accumulation = 0.0f;
  float sin_accumulation = 0.0f;
  float amp = 0.0f;
  float phase = 0.0f;
  int32_t dc_sum = 0;

  // because voltage can be varaible, on the vbias a sum is added to make sure
  // it stays centered
  for (int i = 0; i < CONFIG_RX_SAMPLE_NUMBER; i++) {
    dc_sum += signal_buf[i];
  }

  float true_dc_offset = (float)dc_sum / (float)CONFIG_RX_SAMPLE_NUMBER;

  for (int i = 0; i < CONFIG_RX_SAMPLE_NUMBER; i++) {

    float ac_wave_signal = (float)signal_buf[i] - true_dc_offset;

    sin_accumulation += cached_s_c->sine[i] * ac_wave_signal;
    cos_accumulation += cached_s_c->cosine[i] * ac_wave_signal;
  }

  amp = sqrtf((sin_accumulation * sin_accumulation) +
              (cos_accumulation * cos_accumulation));

  amp = (amp / (float)CONFIG_RX_SAMPLE_NUMBER) * 2.0f;

  // normalise to volts
  amp = (amp / 4096.0f) * 3.6f;
  // change to update to milivolts
  amp = amp * 1000.0f;

  // phase was calculated differently to paper here
  // because of the the way it is entering the coil
  // phase = atan2f(-sin_accumulation, cos_accumulation);
  phase = atan2f(-sin_accumulation, cos_accumulation);

  // update :)
  update_rx_adc_data(amp, phase, axis);

  return 0;
}

void switch_channel(uint8_t channel_num) {
  gpio_pin_set_dt(&mux_s0, (channel_num & 0x01) ? 1 : 0);
  gpio_pin_set_dt(&mux_s1, (channel_num & 0x02) ? 1 : 0);
  gpio_pin_set_dt(&mux_s2, (channel_num & 0x04) ? 1 : 0);

  // if still arcing can reduce this
  k_busy_wait(50);
  // k_busy_wait(1000);
}

void adc_sample(uint8_t channel_num, nrf_saadc_value_t *buf,
                uint32_t *timestamp_out) {
  switch_channel(channel_num);

  nrfx_timer_disable(&adc_timer);
  nrfx_timer_clear(&adc_timer);

  nrfx_err_t err = nrfx_saadc_buffer_set(buf, CONFIG_RX_SAMPLE_NUMBER);
  if (err != 0) {
    printk("CRITICAL ERROR: Buffer set failed with code %d\n", err);
    k_msleep(5000);
    return;
  }

  nrfx_saadc_mode_trigger();

  k_sem_take(&adc_semaphore, K_FOREVER);

  // store when ADC started sampling
  *timestamp_out = nrfx_timer_capture_get(&rx_timer, NRF_TIMER_CC_CHANNEL2);

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

    // read the quaterion orientation when sampling ADC
    read_data(&local_data_rx);
    // read accel data for later positioning
    read_rx_data(&local_sensor_data);

    float rx_q0 = local_data_rx.q0;
    float rx_q1 = local_data_rx.q1;
    float rx_q2 = local_data_rx.q2;
    float rx_q3 = local_data_rx.q3;
    local_timestamp = sync.timestamp;
    local_tx_data.tx_phase = sync.tx_phase;
    local_tx_data.q0 = sync.q0;
    local_tx_data.q1 = sync.q1;
    local_tx_data.q2 = sync.q2;
    local_tx_data.q3 = sync.q3;

    // really these should be done as soon as possible, I wonder if there is a
    // better way to optimise this?
    adc_sample(0, sample_buf_x, &adc_timestamp_x);
    adc_sample(3, sample_buf_y, &adc_timestamp_y);
    adc_sample(5, sample_buf_z, &adc_timestamp_z);

    // ISSUE: need to change this from TX matched filter
    matched_filter(sample_buf_x, &cached_s_c, "x");
    matched_filter(sample_buf_y, &cached_s_c, "y");
    matched_filter(sample_buf_z, &cached_s_c, "z");

    read_rx_adc_data(&local_signal_data);

    // for calibration, before the values become signed
    // printk(
    //     "x amp: %-8.3f | y amp: %-8.3f | z amp: %-8.3f \n",
    //     local_signal_data.adc_x_amp, local_signal_data.adc_y_amp,
    //     local_signal_data.adc_z_amp);

    phase_sync_signs();
    // extracted phase from matched filter
    raw_phase = *coils_arr[last_reference_coil].phase;

    read_rx_adc_data(&local_signal_data);

    read_data(&local_data_rx);

    // Looks at reference coils phase error
    // float phase_err = wrap_to_pi((raw_phase - synced_phase));
    // printk("phase: %6.3f | q0: %6.3f | q1: %6.3f | q2: %6.3f | q3: %6.3f | "
    //        "syncedPhase: %6.3f | phaseErr: %6.2f | %s amp: %5.3f \n",
    //        raw_phase, local_data_rx.q0, local_data_rx.q1, local_data_rx.q2,
    //        local_data_rx.q3, synced_phase, phase_err,
    //        coils_arr[last_reference_coil].axis,
    //        *coils_arr[last_reference_coil].amp);

    // printk("x amp: %-8.3f | y amp: %-8.3f | z amp: %-8.3f | strongest
    // axis%s\n",
    //        local_signal_data.adc_x_amp, local_signal_data.adc_y_amp,
    //        local_signal_data.adc_z_amp, coils_arr[last_reference_coil].axis);

    // printk("first timestamp: %d | last timestamp: %d | difference: %d\n",
    //        adc_timestamp_x, adc_timestamp_z, adc_timestamp_z -
    //        adc_timestamp_x);

    // scale the vectors
    float bx = (local_signal_data.adc_x_amp * GAIN_X);
    float by = (local_signal_data.adc_y_amp * GAIN_Y);
    float bz = (local_signal_data.adc_z_amp * GAIN_Z);

    // printk("x amp: %-8.3f | y amp: %-8.3f | z amp: %-8.3f | strongest axis %s
    // "
    //        "| gain magnitude: %-8.3f\n",
    //        bx, by, bz, coils_arr[last_reference_coil].axis,
    //        sqrtf((bx * bx) + (by * by) + (bz * bz)));

    // TODO: should include the current quaternions as well, so that we are not
    // using stale quaternions when calculating phase?

    struct solver_packet_t pos_packet = {
        .bx = bx,
        .by = by,
        .bz = bz,
        .tx_q0 = local_tx_data.q0,
        .tx_q1 = local_tx_data.q1,
        .tx_q2 = local_tx_data.q2,
        .tx_q3 = local_tx_data.q3,
        .rx_q0 = rx_q0,
        .rx_q1 = rx_q1,
        .rx_q2 = rx_q2,
        .rx_q3 = rx_q3,
        .accel_x = local_sensor_data.accel_x,
        .accel_y = local_sensor_data.accel_y,
        .accel_z = local_sensor_data.accel_z,
    };

    k_msgq_put(&positioning_queue, &pos_packet, K_NO_WAIT);
    // k_msleep(10);
  }
}
