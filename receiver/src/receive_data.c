#include "data_handle.h"
#include "zephyr/kernel.h"
#include <esb.h>
#include <hal/nrf_timer.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#import "receive_data.h"

void esb_rx_event_handler(struct esb_evt const *event) {
  if (event->evt_id == ESB_EVENT_RX_RECEIVED) {
    // get time stamp of when packet was received
    // uint32_t sync_timestamp =
        // nrf_timer_cc_get(NRF_TIMER2, NRF_TIMER_CC_CHANNEL1);

    struct esb_payload rx_payload;
    // read packet from radio
    if (esb_read_rx_payload(&rx_payload) == 0) {
      union esb_packet incoming_data;
      // copy bytes into union struct to convert back to floats
      memcpy(incoming_data.bytes, rx_payload.data, rx_payload.length);
      update_tx_data(incoming_data.data.q0, incoming_data.data.q1,
                     incoming_data.data.q2, incoming_data.data.q3,
                     incoming_data.data.tx_phase);

      // printk("Phase %f arrived at microsecond: %u\n",
             // incoming_data.data.tx_phase, sync_timestamp);

      // printk(
      //     "Broadcasting data: %.2f, %.2f, %.2f, %.2f | phase: %.4f amp: %.4f\n",
      //     incoming_data.data.q0, incoming_data.data.q1, incoming_data.data.q2,
      //     incoming_data.data.q3, incoming_data.data.tx_phase);

      // TODO: phase sync here
    }
  }
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
