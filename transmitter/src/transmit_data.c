#include "data_handle.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

// TODO: transmit phase and quaternion data

// NOTE: first 2 items are manufacturer specific data
//
// static uint8_t tx_buffer[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static struct data_container_t data_container;

// static const struct bt_data ad[] = {
//     BT_DATA(BT_DATA_MANUFACTURER_DATA, tx_buffer, sizeof(tx_buffer)),
// };

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

  printk("Bluetooth initialized\n");

  return 0;
}

struct bt_data create_payload(){
  // update local container
  read_data(&data_container);
  // cast data into bytes (first bytes are manufacture specific data)
  uint8_t tx_buffer[] = {
    0xff,
    0xff,
    (uint8_t)data_container.q0,
    (uint8_t)data_container.q1,
    (uint8_t)data_container.q2,
    (uint8_t)data_container.q3,
    (uint8_t)data_container.tx_phase,
  };

  struct bt_data ad =  BT_DATA(BT_DATA_MANUFACTURER_DATA, tx_buffer, sizeof(tx_buffer)); 
  return ad;
}

int transmit() {
  // maybe access quaternion and phase data and then broadcast that?
  // TODO: single broadcast
  int err;
  // construct payload
  struct bt_data ad = create_payload();

  // prints quaternion and phase values
  printk("Broadcasting data: %.2f, %.2f, %.2f, %.2f | %.4f\n",
         data_container.q0, data_container.q1, data_container.q2, data_container.q3, data_container.tx_phase);

  // broadcast
  err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, &ad, sizeof(ad), NULL, 0);

  if (err) {
    printk("Advertising failed to start (err %d)\n", err);
    return 0;
  }

  k_msleep(1000);
  // NOTE: can change this so that we update broadcast message instead of stop and start
  err = bt_le_adv_stop();
  if (err) {
    printk("Advertising failed to stop (err %d)\n", err);
    return 0;
  }
  return 0;
}
