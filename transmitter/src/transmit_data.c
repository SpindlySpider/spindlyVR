#include "adc.h"
#include "data_handle.h"
#include <esb.h>
#include <helpers/nrfx_gppi.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include "hal/nrf_timer.h"
#include <nrfx_timer.h>

// NOTE: 16MHZ / 32KHZ, number of ticks for each wave, will need to be modified
// if the frequency changes
#define TICKS_PER_WAVE 500
// if you modify this make sure to change the bit_depth on the TX ESB config
#define TX_TIMER_BIT_DEPTH 32
#define TWO_PI (2.0f * PI)

extern struct k_sem adc_semaphore;
extern struct k_sem adc_radio_sync_semaphore;

extern struct cached_sin_cos_t cached_s_c;
extern int16_t sample_buf;

static struct data_container_t local_data_container;

static uint32_t tx_seq = 0;
static float last_frame_true_phase = 0.0f;
static volatile bool last_tx_success = false;

K_SEM_DEFINE(tx_sync_sem, 0, 1);

static nrfx_timer_t tx_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(4));

void esb_tx_event_handler(struct esb_evt const *event) {
  switch (event->evt_id) {
  case ESB_EVENT_TX_FAILED:
    printk("TX FAILED! Flushing buffer...\n");
    esb_flush_tx();
    last_tx_success = false;
    k_sem_give(&tx_sync_sem);
    break;
  case ESB_EVENT_TX_SUCCESS:
    last_tx_success = true;
    k_sem_give(&tx_sync_sem);
    break;
  default:
    break;
  }
}

int config_esb(void) {
  int err;
  struct esb_config config = ESB_DEFAULT_CONFIG;
  config.protocol = ESB_PROTOCOL_ESB_DPL;
  // set to 2MBS to support high refresh rate, set up as Tx
  config.bitrate = ESB_BITRATE_2MBPS;
  config.mode = ESB_MODE_PTX;
  // specifically set ramp up time to 40us, so we can calculate TLL on receiver
  config.use_fast_ramp_up = true;
  config.tx_mode = ESB_TXMODE_MANUAL_START;
  // assign event handler
  config.event_handler = esb_tx_event_handler;
  config.selective_auto_ack = true;

  err = esb_init(&config);
  if (err)
    return err;

  // set base address from common config (data_handle.h)
  esb_set_base_address_0(base_addr_0);
  esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

  printk("Transmitter ESB Initialized!\n");
  return 0;
}

int setup_transmitter() {
  int err;
  err = config_esb();
  if (err != 0) {
    printk("error setting up transmitter: %d", err);
    return err;
  }
  return 0;
}

int config_tx_timer() {
  // incredibly useful:
  // https://github.com/zephyrproject-rtos/hal_nordic/tree/master/nrfx/samples/src/nrfx_timer
  int err;

  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER4), IRQ_PRIO_LOWEST,
              nrfx_timer_irq_handler, &tx_timer, 0);

  irq_enable(NRFX_IRQ_NUMBER_GET(NRF_TIMER4));

  uint32_t frequency = NRF_TIMER_BASE_FREQUENCY_GET(tx_timer.p_reg);

  nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(frequency);
  config.bit_width = NRF_TIMER_BIT_WIDTH_32;

  err = nrfx_timer_init(&tx_timer, &config, NULL);

  nrfx_timer_clear(&tx_timer);
  // nrfx_timer_enable(&tx_timer);

  // set wrap value so there are no phase jumps in timestamps
  uint32_t wrap_value = floor(powl(2, TX_TIMER_BIT_DEPTH) / 500) * 500;
  nrfx_timer_extended_compare(&tx_timer, NRF_TIMER_CC_CHANNEL0, wrap_value,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

  nrfx_timer_enable(&tx_timer);

  printk("Started timer\n");
  k_msleep(1000);

  return 0;
}

int config_tx_ppi() {
  // set up generic PPI
  nrfx_gppi_handle_t gppi_adc_timestamp;
  nrfx_gppi_handle_t gppi_phase_timestamp;

  // timestamp when ADC is done
  int err = nrfx_gppi_conn_alloc(
      nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
      nrf_timer_task_address_get(NRF_TIMER4, NRF_TIMER_TASK_CAPTURE1),
      &gppi_adc_timestamp);

  if (err != 0) {
    return err;
  }

  // record the time the radio starts sending phase pulse
  err = nrfx_gppi_conn_alloc(
      nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS),
      nrf_timer_task_address_get(NRF_TIMER4, NRF_TIMER_TASK_CAPTURE2),
      &gppi_phase_timestamp);

  if (err != 0) {
    return err;
  }

  // enable this connection
  nrfx_gppi_conn_enable(gppi_phase_timestamp);
  nrfx_gppi_conn_enable(gppi_adc_timestamp);
  return 0;
}

static float wrap_to_pi(float angle) {
  angle = fmodf(angle + PI, TWO_PI);
  if (angle < 0.0f) {
    angle += TWO_PI;
  }
  return angle - PI;
}

static bool have_tx_phase_dbg = false;
static uint32_t prev_phase_seq = 0;
static uint32_t prev_phase_ts = 0;
static float prev_phase = 0.0f;

static float phase_advance_from_ticks(uint32_t dt_ticks) {
  const uint32_t ticks_per_period = 500; // 16 MHz / 32 kHz
  uint32_t phase_ticks = dt_ticks % ticks_per_period;
  return ((float)phase_ticks) * (2.0f * PI / 500.0f);
}

static void debug_tx_phase_continuity(uint32_t seq, uint32_t radio_ts,
                                      float phase) {
  if (!have_tx_phase_dbg) {
    have_tx_phase_dbg = true;
    prev_phase_seq = seq;
    prev_phase_ts = radio_ts;
    prev_phase = phase;
    return;
  }

  if (seq != prev_phase_seq + 1) {
    printk("TXPHASE skip seq=%u prev=%u\n", seq, prev_phase_seq);
    prev_phase_seq = seq;
    prev_phase_ts = radio_ts;
    prev_phase = phase;
    return;
  }

  uint32_t dt = radio_ts - prev_phase_ts;
  float adv = phase_advance_from_ticks(dt);

  float pred_plus = wrap_to_pi(prev_phase + adv);
  float pred_minus = wrap_to_pi(prev_phase - adv);

  float err_plus = wrap_to_pi(phase - pred_plus);
  float err_minus = wrap_to_pi(phase - pred_minus);

  printk("TXPHASE seq=%u dt=%u mod=%u prev=%.3f now=%.3f "
         "plus_err=%.3f plus_cos=%.3f "
         "minus_err=%.3f minus_cos=%.3f\n",
         seq, dt, dt % 500, prev_phase, phase, err_plus, cosf(err_plus),
         err_minus, cosf(err_minus));

  prev_phase_seq = seq;
  prev_phase_ts = radio_ts;
  prev_phase = phase;
}

void calculate_phase_offset(float *update_phase, uint32_t adc_timestamp,
                            uint32_t packet_seq) {
  uint32_t radio_timestamp =
      nrfx_timer_capture_get(&tx_timer, NRF_TIMER_CC_CHANNEL2);

  uint32_t sample_tx_tick_dif = radio_timestamp - adc_timestamp;
  uint32_t phase_ticks = sample_tx_tick_dif % TICKS_PER_WAVE;

  float phase_diff = (float)phase_ticks * (2.0f * PI / (float)TICKS_PER_WAVE);

  float phase = wrap_to_pi(local_data_container.tx_phase + phase_diff);

  *update_phase = phase;

  debug_tx_phase_continuity(packet_seq, radio_timestamp, phase);

  printk("TXCALC seq=%u adc=%u radio=%u dt=%u mod=%u raw=%.3f ff=%.3f\n",
         packet_seq, adc_timestamp, radio_timestamp, sample_tx_tick_dif,
         sample_tx_tick_dif % 500, local_data_container.tx_phase, phase);
}

void transmit_phase(float phase, uint32_t packet_seq) {
  union esb_packet my_packet = {0};
  struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0);

  read_data(&local_data_container);

  my_packet.data = local_data_container;
  my_packet.data.packet_seq = packet_seq;
  my_packet.data.tx_phase = phase;

  BUILD_ASSERT(sizeof(my_packet.data) <= CONFIG_ESB_MAX_PAYLOAD_LENGTH,
               "ESB payload too large");

  tx_payload.length = sizeof(my_packet.data);
  tx_payload.noack = true;

  memcpy(tx_payload.data, &my_packet.data, tx_payload.length);

  esb_flush_tx();

  int err = esb_write_payload(&tx_payload);
  if (err != 0) {
    printk("ESB write error: %d len=%u\n", err, tx_payload.length);
    return;
  }

  err = esb_start_tx();
  if (err != 0) {
    printk("ESB start error: %d\n", err);
  }

  printk("Broadcasting data: %.2f, %.2f, %.2f, %.2f | phase: %.4f amp:% .4f\n ",
         local_data_container.q0, local_data_container.q1,
         local_data_container.q2, local_data_container.q3,
         local_data_container.tx_phase, local_data_container.tx_amp);
}

void start_transmit_thread(void *, void *, void *) {
  config_tx_timer();

  config_tx_ppi();
  uint32_t adc_timestamp;
  while (1) {
    k_sem_take(&adc_radio_sync_semaphore, K_FOREVER);
    uint32_t now = nrfx_timer_capture(&tx_timer, NRF_TIMER_CC_CHANNEL0);
    printk("TX TIMER4 now=%u mod=%u\n", now, now % 500);

    read_adc_data(&local_data_container, &adc_timestamp);

    uint32_t this_seq = tx_seq++;

    transmit_phase(last_frame_true_phase, this_seq);

    k_sem_take(&tx_sync_sem, K_FOREVER);

    calculate_phase_offset(&last_frame_true_phase, adc_timestamp, this_seq);
    // calculate_phase_offset(&last_frame_true_phase, adc_timestamp);

    k_msleep(1000 / CONFIG_TX_BROADCAST_FREQUENCY);
  }
}
