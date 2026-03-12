#ifndef DATA_HANDLE_H
#define DATA_HANDLE_H

#include <zephyr/kernel.h>
struct data_container_t {
    // Quaternion orientation
    float q0,q1,q2,q3;

    // transmitted to Rx so it knows its signed magnetic field
    float tx_amp, tx_phase;
};

// used for quick conversion of data to bytes
union esb_packet {
    struct data_container_t data;
    uint8_t bytes[sizeof(struct data_container_t)];
};

// ESB radio address (make sure your Rx and Tx share the same)
static uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
static uint8_t addr_prefix[8] = {0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9};


void update_orientation_data(float q0, float q1, float q2, float q3);


void update_signal_data(float amp, float phase);

void read_data(struct data_container_t *data_destination);

#endif
