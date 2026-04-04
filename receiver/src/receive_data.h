#ifndef DATA_HANDLE_H
#define DATA_HANDLE_H

#include "data_handle.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

int setup_receiver();

void esb_rx_event_handler(struct esb_evt const *event);

int setup_rx_gppi();

#endif
