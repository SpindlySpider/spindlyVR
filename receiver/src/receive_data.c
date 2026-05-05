#include "data_handle.h"
#include "timer.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include <esb.h>
#include <hal/nrf_timer.h>
#include <helpers/nrfx_gppi.h>
#include <math.h>
#include <nrfx_timer.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include "receive_data.h"

struct rx_ts_entry {
  uint32_t seq;
  uint32_t timestamp;
  bool valid;
};

extern struct k_msgq rx_sync_msgq;
extern struct k_sem radio_sync_sem;

static bool have_prev_packet = false;
static uint32_t prev_packet_seq = 0;
static uint32_t prev_packet_timestamp = 0;
static struct data_container_t prev_packet_data;


void esb_rx_event_handler(struct esb_evt const *event) {
  if (event->evt_id != ESB_EVENT_RX_RECEIVED) {
    return;
  }

  struct esb_payload rx_payload;

  while (esb_read_rx_payload(&rx_payload) == 0) {
    uint32_t current_timestamp =
        nrfx_timer_capture_get(&rx_timer, NRF_TIMER_CC_CHANNEL1);

    union esb_packet incoming_data;
    memset(&incoming_data, 0, sizeof(incoming_data));

    size_t copy_len = MIN(rx_payload.length, sizeof(incoming_data.bytes));
    memcpy(incoming_data.bytes, rx_payload.data, copy_len);

    uint32_t packet_seq = incoming_data.data.packet_seq;

    // check they are consecutive
    bool consecutive =
        have_prev_packet && ((uint32_t)(prev_packet_seq + 1) == packet_seq);

    if (!consecutive) {
      have_prev_packet = true;
      prev_packet_seq = packet_seq;
      prev_packet_timestamp = current_timestamp;
      continue;
    }

    /*
     * Packet N contains phase for packet N-1.
     * Therefore pair incoming tx_phase with previous packet timestamp.
     */
    struct rx_sync_ref_t sync = {
        .seq = prev_packet_seq,
        .timestamp = prev_packet_timestamp,
        .tx_phase = incoming_data.data.tx_phase,
        .q0 = incoming_data.data.q0,
        .q1 = incoming_data.data.q1,
        .q2 = incoming_data.data.q2,
        .q3 = incoming_data.data.q3,
    };

    /*
     * If ADC is behind, drop old sync events.
     * Better to use the newest coherent sync than process stale ones.
     * seems like this always removes the packets?
     * if its using old ones would probbaly be better to have a timer condition on ADC to chhose wether to use them or not
     */
    if (k_msgq_put(&rx_sync_msgq, &sync, K_NO_WAIT) != 0) {
      struct rx_sync_ref_t dummy;
      // removes old packets from queue
      while (k_msgq_get(&rx_sync_msgq, &dummy, K_NO_WAIT) == 0) {
      }
      // puts new packets into queue
      k_msgq_put(&rx_sync_msgq, &sync, K_NO_WAIT);
    }

    prev_packet_seq = packet_seq;
    prev_packet_timestamp = current_timestamp;
    have_prev_packet = true;
  }
}

int setup_rx_gppi(void) {
  nrfx_gppi_handle_t gppi_timestamp;
  // not firing??

  int err = nrfx_gppi_conn_alloc(
      nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_CRCOK),
      nrf_timer_task_address_get(rx_timer.p_reg, NRF_TIMER_TASK_CAPTURE1),
      &gppi_timestamp);

  if (err != 0) {
    printk("RX radio timestamp GPPI failed: %d\n", err);
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

  printk("Setting up receiver gppio\n");
  k_msleep(1000);
  err = setup_rx_gppi();
  if (err != 0) {
    return err;
  }

  // start listening
  esb_start_rx();

  printk("Tracker ESB Listening...\n");
  return 0;
}
