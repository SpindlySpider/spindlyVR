#include "data_handle.h"
#include <esb.h>
#include <helpers/nrfx_gppi.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include "hal/nrf_radio.h"
#include "hal/nrf_timer.h"
#include <nrfx_timer.h>

static struct data_container_t local_data_container;

// NOTE: time is 40 microseconds because use_fast_ramp_up is set more info here:
// https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html#fast_ramp-up
// radio ramp up time
static int TLL = 40;
static float TAIR = 40;

static nrfx_timer_t tx_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(4));

void esb_tx_event_handler(struct esb_evt const *event) {
  switch (event->evt_id) {
  case ESB_EVENT_TX_FAILED:
    printk("TX FAILED! Flushing buffer...\n");
    esb_flush_tx();
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

  uint32_t frequency = NRF_TIMER_BASE_FREQUENCY_GET(tx_timer.p_reg);

  nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(frequency);

  err = nrfx_timer_init(&tx_timer, &config, NULL);

  nrfx_timer_clear(&tx_timer);

  // convert hz to micro seconds
  uint32_t hz_to_us = (uint32_t)(1000000 / CONFIG_TX_BROADCAST_FREQUENCY);
  uint32_t desired_ticks = nrfx_timer_us_to_ticks(&tx_timer, hz_to_us);

  // set event to transmit every 100hz
  nrfx_timer_extended_compare(&tx_timer, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

  nrfx_timer_enable(&tx_timer);
  printk("Started timer\n");
  k_msleep(1000);

  return 0;
}

int config_tx_ppi() {
  // set up generic PPI
  nrfx_gppi_handle_t gppi_transmit;

  // connect timer event compare0 to broadcast data every 100hz
  int err = nrfx_gppi_conn_alloc(
      nrf_timer_event_address_get(NRF_TIMER4, NRF_TIMER_EVENT_COMPARE0),
      nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_TXEN),
      &gppi_transmit);

  if (err != 0) {
    return err;
  }

  // enable this connection
  nrfx_gppi_conn_enable(gppi_transmit);
  return 0;
}

void calculate_phase_offset(float *update_phase) {
  // get current time / tick
  // nrf_timer_task_trigger(&timer_inst, NRF_TIMER_TASK_CAPTURE1);
  uint32_t current_us = nrfx_timer_capture(&tx_timer, NRF_TIMER_CC_CHANNEL1);
  uint32_t us_until_tx;

  if (current_us <= 10000) {
    us_until_tx = 10000 - current_us;
  } else {
    // woke up late, we calculate for next broadcast event
    us_until_tx = 20000 - current_us;
  }

  // convert the time into seconds
  float seconds_until_tx = (float)us_until_tx / 1000000.0f;

  read_data(&local_data_container);

  float phase = fmodf(
      local_data_container.tx_phase +
          (2.0f * PI * (CONFIG_TX_PWM_FREQUENCY * 1000.0f) * seconds_until_tx),
      2.0f * PI);

  // may cause issue here might need to allow amp to not be passed or need to
  // fetch new amp value
  memcpy(update_phase, &phase, sizeof(phase));
}

void transmit(float *phase) {
  union esb_packet my_packet;
  read_data(&local_data_container);
  my_packet.data = local_data_container;
  memcpy(&my_packet.data.tx_phase, phase, sizeof(phase));
  // my_packet.data.tx_phase = phase;

  // Create empty payload
  struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0);
  tx_payload.length = sizeof(my_packet);
  // broadcasting so ack is not required
  tx_payload.noack = true;
  // copy packet data into payload
  memcpy(&tx_payload.data, &my_packet.bytes, sizeof(my_packet));
  // send data to all listening Rx
  // empty payload buffer
  esb_flush_tx();

  printk("Broadcasting data: %.2f, %.2f, %.2f, %.2f | phase: %.4f amp:% .4f\n ",
         local_data_container.q0, local_data_container.q1,
         local_data_container.q2, local_data_container.q3,
         local_data_container.tx_phase, local_data_container.tx_amp);

  int err;
  err = esb_write_payload(&tx_payload);
  switch (err) {
  case 0:
    // success we sent the packet :))
    break;
  case -12:
    printk("Radio jammed! Error code: %d\n", err);
    break;
  default:
    // another error
    printk("ESB error: %d\n", err);
    break;
  }
}

void start_transmit_thread(void *, void *, void *) {
  config_tx_timer();
  config_tx_ppi();
  while (1) {
    float phase;
    calculate_phase_offset(&phase);
    transmit(&phase);
    // sleep so it does this at 100 hz
    // 1000 (ms) / hz to get millisecond sleep time
    k_msleep(1000 / CONFIG_TX_BROADCAST_FREQUENCY);
  }
}
