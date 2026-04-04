#include "data_handle.h"
#include "timer.h"
#include "zephyr/kernel.h"
#include <esb.h>
#include <hal/nrf_timer.h>
#include <helpers/nrfx_gppi.h>
#include <math.h>
#include <nrfx_timer.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#import "receive_data.h"

void esb_rx_event_handler(struct esb_evt const *event) {
  if (event->evt_id == ESB_EVENT_RX_RECEIVED) {
    // get time stamp of when packet was received
    // timestamp when received

    uint32_t receive_time_us =
        nrfx_timer_capture_get(&rx_timer, NRF_TIMER_CC_CHANNEL1);

    // uint32_t sync_timestamp =
    // nrf_timer_cc_get(NRF_TIMER2, NRF_TIMER_CC_CHANNEL1);

    struct esb_payload rx_payload;
    // read packet from radio
    if (esb_read_rx_payload(&rx_payload) == 0) {
      union esb_packet incoming_data;
      // copy bytes into union struct to convert back to floats
      memcpy(incoming_data.bytes, rx_payload.data, rx_payload.length);

      // calculate phase before updating struct

      // calculate air time, we are using 2MBPS so / by 2,000,000, time taken
      // over the air in seconds
      float TAIR = (sizeof(rx_payload.length) * 8) / 2000000.0f;
      // we are also using fast ramp up time so TLL is 40us
      // more info here:
      // https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html#fast_ramp-up
      // converted to seconds its 0.000040f. (40/1000000)
      float total_time = TAIR + 0.000040f;

      float phase = fmodf(incoming_data.data.tx_phase +
                              2.0f * PI * (20000.0f) * total_time,
                          2.0f * PI);

      update_tx_data(incoming_data.data.q0, incoming_data.data.q1,
                     incoming_data.data.q2, incoming_data.data.q3, phase);

      update_timestamp(receive_time_us);

      // printk("Phase %f arrived at microsecond: %u\n",
      // incoming_data.data.tx_phase, sync_timestamp);

      // printk(
      //     "Broadcasting data: %.2f, %.2f, %.2f, %.2f | phase: %.4f amp:
      //     %.4f\n", incoming_data.data.q0, incoming_data.data.q1,
      //     incoming_data.data.q2, incoming_data.data.q3,
      //     incoming_data.data.tx_phase);
    }
  }
}

int setup_rx_gppi(){
  nrfx_gppi_handle_t gppi_timestamp;

  // when we receive a radio packet save the time we got it (for working out phase sign)
  int err = nrfx_gppi_conn_alloc(
      nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_END),
      nrf_timer_task_address_get(NRF_TIMER4, NRF_TIMER_TASK_CAPTURE1),
      &gppi_timestamp);

  if (err != NRFX_SUCCESS) {
    return err;
  }

  nrfx_gppi_conn_enable(gppi_timestamp);
  return 0;
}

int setup_receiver(void) {
  int err;
  struct esb_config config = ESB_DEFAULT_CONFIG;

  config.protocol = ESB_PROTOCOL_ESB_DPL;
  // setup for 2MBPS and receiver mode
  config.bitrate = ESB_BITRATE_2MBPS;
  config.mode = ESB_MODE_PRX;
  config.event_handler = esb_rx_event_handler;

  err = esb_init(&config);
  if (err != 0) {
    printk("Tracker failed to init ESB: %d\n", err);
    return err;
  }

  esb_set_base_address_0(base_addr_0);
  esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

  // start listening
  esb_start_rx();

  printk("Tracker ESB Listening...\n");
  return 0;
}
