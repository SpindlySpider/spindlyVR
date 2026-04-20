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

extern struct k_sem adc_semaphore;
extern struct k_sem adc_radio_sync_semaphore;

extern struct cached_sin_cos_t cached_s_c;
static float last_frame_true_phase = 0.0f;
extern int16_t sample_buf;

static struct data_container_t local_data_container;

K_SEM_DEFINE(tx_sync_sem, 0, 1);

static nrfx_timer_t tx_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(4));

void esb_tx_event_handler(struct esb_evt const *event) {
  switch (event->evt_id) {
  case ESB_EVENT_TX_FAILED:
    printk("TX FAILED! Flushing buffer...\n");
    esb_flush_tx();
    k_sem_give(&tx_sync_sem);
    break;
  case ESB_EVENT_TX_SUCCESS:
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

  // set wrap value so there are no phase jumps in timestamps
  uint32_t wrap_value = floor(powl(2, TX_TIMER_BIT_DEPTH) / 500) * 500;
  nrfx_timer_extended_compare(&tx_timer, NRF_TIMER_CC_CHANNEL0, wrap_value,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

  // nrfx_timer_enable(&tx_timer);

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

void calculate_phase_offset(float *update_phase, uint32_t adc_timestamp) {
  // get when phase was timestamped
  // NOTE: if this has issues, move it out of this function and collect this
  // data as soon as ADC and radio have finished and pass it back into here to
  // prevent race conditions

  uint32_t radio_timestamp =
      nrfx_timer_capture_get(&tx_timer, NRF_TIMER_CC_CHANNEL2);

  // fast forward the phase from when the packet was sent
  // find the difference between when the ADC was sampled and when the radio
  // sent the data
  //
  // NOTE: Need to add somthing to account for when ADC timestamp at high number
  // and radio close to 0 on timer because of wrap around.
  uint32_t sample_tx_tick_dif = radio_timestamp - adc_timestamp;

  int32_t phase_ticks = sample_tx_tick_dif % TICKS_PER_WAVE;
  float phase_diff = (float)phase_ticks * (2.0f * PI / (float)TICKS_PER_WAVE);

  // fast forward phase from ADC readings to when it was transmitter to RX
  float phase = fmodf(local_data_container.tx_phase + phase_diff, 2.0f * PI);

  // Safely pass the value back
  *update_phase = phase;
}

void transmit_phase(float *phase) {
  union esb_packet my_packet;
  struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0);

  read_data(&local_data_container);
  // Create empty payload
  my_packet.data = local_data_container;
  my_packet.data.tx_phase = *phase;
  tx_payload.length = sizeof(my_packet);

  // broadcasting so ack is not required
  tx_payload.noack = true;
  // copy packet data into payload
  memcpy(&tx_payload.data, &my_packet.bytes, sizeof(my_packet));
  // empty payload buffer
  esb_flush_tx();

  printk("Broadcasting data: %.2f, %.2f, %.2f, %.2f | phase: %.4f amp:% .4f\n ",
         local_data_container.q0, local_data_container.q1,
         local_data_container.q2, local_data_container.q3, *phase,
         local_data_container.tx_amp);

  int err;
  // broadcast phase
  err = esb_write_payload(&tx_payload);
  switch (err) {
  case 0:
    // success we loaded the packet :))
    break;
  case -12:
    printk("Radio jammed! Error code: %d\n", err);
    break;
  default:
    // another error
    printk("ESB error: %d\n", err);
    break;
  }
  // start transmission
  esb_start_tx();
}

void start_transmit_thread(void *, void *, void *) {
  config_tx_timer();
  config_tx_ppi();
  uint32_t adc_timestamp;
  while (1) {
    // make sure there is a sample ready before transmitting
    k_sem_take(&adc_radio_sync_semaphore, K_FOREVER);
    // update local container to most recent readings
    read_adc_data(&local_data_container, &adc_timestamp);
    // get the ADC timestamp
    // k_sem_reset(&tx_sync_sem);

    transmit_phase(&last_frame_true_phase);
    // make sure radio has fully sent phase data
    k_sem_take(&tx_sync_sem, K_FOREVER);
    // calculate this phase offset
    calculate_phase_offset(&last_frame_true_phase, adc_timestamp);
    // 1000 (ms) / hz to get millisecond sleep time
    k_msleep(1000 / CONFIG_TX_BROADCAST_FREQUENCY);
  }
}
