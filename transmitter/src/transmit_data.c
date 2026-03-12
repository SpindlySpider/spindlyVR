#include "data_handle.h"
#include <esb.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

static struct data_container_t local_data_container;

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

void transmit() {
  union esb_packet my_packet;
  read_data(&local_data_container);
  my_packet.data = local_data_container;

  // Create empty payload
  struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0);
  tx_payload.length = sizeof(my_packet);
  // broadcasting so ack is not required
  tx_payload.noack = true;
  // copy packet data into payload
  memcpy(&tx_payload.data, &my_packet.bytes, sizeof(my_packet));
  // send data to all listening Rx
  int err;
  err = esb_write_payload(&tx_payload);
  switch (err) {
  case 0:
    esb_start_tx();
    printk(
        "Broadcasting data: %.2f, %.2f, %.2f, %.2f | phase: %.4f amp: %.4f\n",
        local_data_container.q0, local_data_container.q1,
        local_data_container.q2, local_data_container.q3,
        local_data_container.tx_phase, local_data_container.tx_amp);
    break;
  case -12:
    printk("Radio jammed! Error code: %d\n", err);
    break;
  default:
    // another error
    printk("ESB error: %d", err);
    break;
  }
}

void start_transmit_thread(void *, void *, void *) {
  while (1) {
    transmit();
    // sleep so it does this at 100 hz
    // 1000 (ms) / hz to get millisecond sleep time
    k_msleep(1000 / CONFIG_TX_BROADCAST_FREQUENCY);
  }
}
