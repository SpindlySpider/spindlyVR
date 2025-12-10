#ifndef IMU_H
#define IMU_H
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include "orientation_handle.h"


float sensor_val_to_float(const struct sensor_value *val);

int setup_imu();
int read_imu(struct sensor_data_struct *data);

#endif
