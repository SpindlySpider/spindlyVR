#ifndef ORIENTATION_HANDLE_H
#define ORIENTATION_HANDLE_H

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

struct sensor_data_struct {
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

int setup_sensors();

int read_sensors();

void run_orientation_loop(void *, void *, void *);

void update_madgwick();

int convert_raw(struct sensor_data_struct *data);

#endif
