#include "data_handle.h"
#import "receive_data.h"
#include "zephyr/bluetooth/assigned_numbers.h"
#include <stdint.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

struct packet_content_t {
  bool found_tx;
  bool found_data;
  uint8_t raw_data[20];
};

static struct data_container_t data_container;

struct bt_le_scan_param scan_param = {
    .type = BT_LE_SCAN_TYPE_ACTIVE,
    // .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
    // .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
    .interval = BT_GAP_SCAN_FAST_INTERVAL_MIN,
    .window = BT_GAP_SCAN_FAST_WINDOW,
};

bool ad_parse_cb(struct bt_data *data, void *user_data) {
  // checks if bluetooth packets are from transmitter and gets data out
  struct packet_content_t *content = user_data;
  if (data->type == BT_DATA_NAME_COMPLETE ||
      data->type == BT_DATA_NAME_SHORTENED) {
    // TODO: make adjustable advertising name for rx and tx (the sVR part)
    if (data->data_len && memcmp(data->data, "sVR", 3) == 0) {
      content->found_data = true;
    }
  }
  if (data->type == BT_DATA_MANUFACTURER_DATA) {
    // check this is using our manufacturer data e.g. 0xff 0xff
    // TODO: make manufacture ID data adjustable
    if (data->data_len == 22) {
      uint16_t manufacture_id =
          data->data[0] | (data->data[1] << 8); // Little-endian ID;
      if (manufacture_id == 0xFFFF) {
        memcpy(content->raw_data, &data->data[2], 20);
        content->found_tx = true;
      }
    }
  }
  return true;
}

static void device_found_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                            struct net_buf_simple *ad) {
  char addr_str[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
  // printk("Device found: %s (RSSI %d), type %u, AD data len %u\n", addr_str,
  //        rssi, type, ad->len);

  // check found device is a transmitter
  struct packet_content_t packet_content = {0};
  bt_data_parse(ad, ad_parse_cb, &packet_content);

  if (packet_content.found_tx && packet_content.found_data) {
    // copy received data into data container
    printk("Found device!!!\n");

    read_data(&data_container);

    memcpy(&data_container.q0, &packet_content.raw_data[0], 4);
    memcpy(&data_container.q1, &packet_content.raw_data[4], 4);
    memcpy(&data_container.q2, &packet_content.raw_data[8], 4);
    memcpy(&data_container.q3, &packet_content.raw_data[12], 4);
    memcpy(&data_container.tx_phase, &packet_content.raw_data[16], 4);

    printk("Receiving data: %.2f, %.2f, %.2f, %.2f | %.4f\n", data_container.q0,
           data_container.q1, data_container.q2, data_container.q3,
           data_container.tx_phase);
  }
}

int receiver_start() {
  int err;

  err = bt_le_scan_start(&scan_param, device_found_cb);
  if (err) {
    printk("Start scanning failed (err %d)\n", err);
    return err;
  }
  printk("Started scanning...\n");

  return 0;
}

int setup_rec_bt() {
  int err;
  err = bt_enable(NULL);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }
}
