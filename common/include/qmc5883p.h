#ifndef QMC5883P_H
#define QMC5883P_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdint.h>

struct qmc_data {
    int16_t x;
    int16_t y;
    int16_t z;
};

int setup_qmc5883p(const struct device *i2c_dev);
int qmc_write_reg(const struct device *i2c_dev, uint8_t reg, uint8_t value);
int qmc_read_sensor_data(const struct device *i2c_dev, struct qmc_data *data);
int qmc_calibration_routine(const struct device *i2c_dev);

#endif
