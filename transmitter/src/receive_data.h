#ifndef DATA_HANDLE_H
#define DATA_HANDLE_H

#include "data_handle.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

int receiver_start();

static void device_found_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,struct net_buf_simple *ad);

int setup_rec_bt();


bool ad_parse_cb(struct bt_data *data, void *user_data);

#endif
