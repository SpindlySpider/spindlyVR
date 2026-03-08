#ifndef DATA_HANDLE_H
#define DATA_HANDLE_H

#include <zephyr/kernel.h>
struct data_container_t {
    // Quaternion orientation
    float q0,q1,q2,q3;

    // transmitted to Rx so it knows its signed magnetic field
    float tx_amp, tx_phase;
};


void update_orientation_data(float q0, float q1, float q2, float q3);


void update_signal_data(float amp, float phase);

void read_data(struct data_container_t *data_destination);

#endif
