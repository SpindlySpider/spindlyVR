#include "data_handle.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

// TODO: transmit phase and quaternion data
//

static struct data_container_t data_container;
// floats are 4 bytes so 2 bytes for manufacture ID, 20 bytes for quaternion and
// phase data = 22 bytes total
uint8_t mfg_data[22] = { 0xff, 0xff };

// NOTE: this may be an issue, check if data container dynamically updates
static struct bt_data ad[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "sVR", 3)};

int init_transmit() {
  // TODO: move code here
  int err;

  printk("Starting Broadcast Phase and Orientation\n");

  // enable bluetooth
  err = bt_enable(NULL);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }

  printk("Bluetooth initialized\n");

  // Create identity to broadcast from
  // NOTE: may need to set irk param?
  err = bt_id_create(NULL, NULL);
  // Check if ID creation failed or passed
  if (err == 0) {
    printk("Bluetooth identity creation failed (err %d)\n", err);
    return 0;
  } else {
    printk("Using bluetooth identity (ID %d)\n", err);
  }

  return 0;
}

void create_payload() {
  // update local container
  read_data(&data_container);
  // cast data into bytes (first 2 bytes are manufacture specific data)
  memcpy(&mfg_data[2], &data_container.q0, 4);
  memcpy(&mfg_data[6], &data_container.q1, 4);
  memcpy(&mfg_data[10], &data_container.q2, 4);
  memcpy(&mfg_data[14], &data_container.q3, 4);
  memcpy(&mfg_data[18], &data_container.tx_phase, 4);
}

int transmit() {
  // maybe access quaternion and phase data and then broadcast that?
  // TODO: single broadcast
  int err;

  create_payload();
  // prints quaternion and phase values
  printk("Broadcasting data: %.2f, %.2f, %.2f, %.2f | %.4f\n",
         data_container.q0, data_container.q1, data_container.q2,
         data_container.q3, data_container.tx_phase);

  // broadcast
  // TODO: turn ad back into an array so we can put more options in the array
  err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), NULL, 0);

  if (err) {
    printk("Advertising failed to start (err %d)\n", err);
    return 0;
  }

  k_msleep(1000);
  // NOTE: can change this so that we update broadcast message instead of stop
  // and start
  err = bt_le_adv_stop();
  if (err) {
    printk("Advertising failed to stop (err %d)\n", err);
    return 0;
  }
  return 0;
}
