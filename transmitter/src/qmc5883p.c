// I would like to credit Granddyser for their QMC5883P implementation for
// ardunio I have used it as a reference to create this driver. their github can
// be found here: https://github.com/Granddyser

#include "qmc5883p.h"
#include "zephyr/logging/log_core.h"
#include <sys/errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#define CHIP_ID 0x00
#define CTRL_REG_1 0x0A
#define CTRL_REG_2 0x0B
#define START_REG 0x01
// #define I2C_NODE DT_NODELABEL(i2c0)
#define QMC5883P_ADDR 0x2c

// configuration
#define MODE 0x03
// mode: 0 suspend | 0x1 normal | 0x2  single | 0x3 continuous
#define ODR 0x03
// output data rate: 0x0 10hz | 0x1 50hz | 0x2 100hz | 0x3 200hz
#define OSR1 0x00
// over sample ratio: 0x0 8 | 0x1 4 | 0x2 2 | 0x3 1
#define OSR2 0x00
// down sampling rate: 0x0 1 | 0x1 2 | 0x2 4| 0x3 8

// bitwise shift and or
#define CONFIG ((OSR2 << 6) | (OSR1 << 4) | (ODR << 2) | MODE)

// #define MAG_OFFSET_X 0
// #define MAG_OFFSET_Y 0
// #define MAG_OFFSET_Z 0
// #define MAG_SCALE_X 1
// #define MAG_SCALE_Y 1
// #define MAG_SCALE_Z 1
// #define MAG_OFFSET_X  286.50
// #define MAG_OFFSET_Y  -69.00
// #define MAG_OFFSET_Z  -24.00
// #define MAG_SCALE_X   1.0117
// #define MAG_SCALE_Y   0.9971
// #define MAG_SCALE_Z   0.9914


// #define MAG_OFFSET_X  371.50
// #define MAG_OFFSET_Y  -69.50
// #define MAG_OFFSET_Z  -32.50
// #define MAG_SCALE_X   1.1358
// #define MAG_SCALE_Y   0.9372
// #define MAG_SCALE_Z   0.9500

#define MAG_OFFSET_X  277.00
#define MAG_OFFSET_Y  -63.00
#define MAG_OFFSET_Z  -30.00
#define MAG_SCALE_X   1.0000
#define MAG_SCALE_Y   1.0019
#define MAG_SCALE_Z   0.9981

LOG_MODULE_REGISTER(qmc5883p, LOG_LEVEL_INF);
// device pointer
// static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

int qmc_write_reg(const struct device *i2c_dev, uint8_t reg, uint8_t value) {
  // make buffer to store data
  uint8_t buffer[2];
  buffer[0] = reg;
  buffer[1] = value;
  // send data over i2c bus
  int ret = i2c_write(i2c_dev, buffer, 2, QMC5883P_ADDR);
  if (ret != 0) {
    LOG_ERR("Error writing to register 0x%02X\n", reg);
  }
  return ret;
}

int setup_qmc5883p(const struct device *i2c_dev) {
  // set payload here to start configutation after establishing i2c;
  // check device is ready
  if (!device_is_ready(i2c_dev)) {
    LOG_ERR("I2C bus is not ready\n");
    return ENODEV;
  }
  // check that the chip is correct
  uint8_t buffer[1];
  i2c_reg_read_byte(i2c_dev, QMC5883P_ADDR, 0x0, buffer);
  if (buffer[0] != 0x80) {
    // this is not the device
    LOG_ERR("this is not a QMC5883p device is %d", buffer[0]);
  }
  LOG_INF("QMC5883p device is %d", buffer[0]);

  // soft reset qmc5883p
  int ret = qmc_write_reg(i2c_dev, CTRL_REG_2, 0x80);
  if (ret != 0) {
    LOG_ERR("Could not soft reset qmc5883p, exiting with error: %d\n", ret);
    return ret;
  }
  // Wait for reset to complete (10ms is usually plenty)
  k_msleep(10);

  // 2. Enable Set/Reset Period (CRITICAL STEP)
  // Write 0x01 to CTRL_REG_2 (0x0B) to turn on the measurement loop
  ret = qmc_write_reg(i2c_dev, CTRL_REG_2, 0x01);
  if (ret != 0) {
    LOG_ERR("Could not set FBR/Set-Reset, error: %d\n", ret);
    return ret;
  }

  // set config
  ret = qmc_write_reg(i2c_dev, CTRL_REG_1, CONFIG);
  if (ret != 0) {
    LOG_ERR("Could not config qmc5883p, exiting with error: %d\n", ret);
    return ret;
  }
  LOG_INF("successfully setup qmc5883p continuing with configuration 0x%02X",
          CONFIG);
  return ret;
}

int qmc_read_sensor_data(const struct device *i2c_dev,
                         struct qmc_data *buffer) {
  // buffer should array of size 3 for xyz
  int ret;
  uint8_t raw_data[6];

  // read 6 bytes from start reg, this is defined in QMC5883p datasheet
  ret = i2c_burst_read(i2c_dev, QMC5883P_ADDR, START_REG, raw_data, 6);

  if (ret == 0) {
    // bit shift and  bitwise or to create full signed value from registers
    int16_t x = (raw_data[1] << 8) | raw_data[0];
    int16_t y = (raw_data[3] << 8) | raw_data[2];
    int16_t z = (raw_data[5] << 8) | raw_data[4];
    buffer->x = (x - MAG_OFFSET_X) * MAG_SCALE_X;
    buffer->y = (y - MAG_OFFSET_Y) * MAG_SCALE_Y;
    buffer->z = (z - MAG_OFFSET_Z) * MAG_SCALE_Z;
  } else {
    // failed to get register values set buffer to 0 and warn
    buffer->x = 0;
    buffer->y = 0;
    buffer->z = 0;
    LOG_ERR("could not read values from qmc5883p");
  }
  return ret;
}

int qmc_calibration_routine(const struct device *i2c_dev) {
  printk("you are now running the calibration routine, this will find offsets "
         "for hard and soft metals\n");
  printk("please rotate the device in a figure of 8 pattern to expose to "
         "magnetic fields in vicinity\n");
  struct qmc_data data;
  // Start with opposite extremes
  int16_t min_x = 32000, max_x = -32000;
  int16_t min_y = 32000, max_y = -32000;
  int16_t min_z = 32000, max_z = -32000;

  for (int i = 0; i < 6000; i++) {
    if (qmc_read_sensor_data(i2c_dev, &data) == 0) {
      // Update Min/Max for X
      if (data.x < min_x)
        min_x = data.x;
      if (data.x > max_x)
        max_x = data.x;
      // Update Min/Max for Y
      if (data.y < min_y)
        min_y = data.y;
      if (data.y > max_y)
        max_y = data.y;
      // Update Min/Max for Z
      if (data.z < min_z)
        min_z = data.z;
      if (data.z > max_z)
        max_z = data.z;
    }
    if (i % 100 == 0)
      printk("."); // Progress dot
    k_msleep(10);
  }

  // --- 1. HARD IRON (OFFSETS) ---
  float off_x = (max_x + min_x) / 2.0f;
  float off_y = (max_y + min_y) / 2.0f;
  float off_z = (max_z + min_z) / 2.0f;

  // --- 2. SOFT IRON (SCALING) ---
  float scale_x = (max_x - min_x) / 2.0f;
  float scale_y = (max_y - min_y) / 2.0f;
  float scale_z = (max_z - min_z) / 2.0f;

  float avg_radius = (scale_x + scale_y + scale_z) / 3.0f;

  float scale_factor_x = avg_radius / scale_x;
  float scale_factor_y = avg_radius / scale_y;
  float scale_factor_z = avg_radius / scale_z;

  printk("\n\n--- CALIBRATION RESULTS ---\n");
  printk("Copy these macros into your code:\n\n");

  // Print Hard Iron Offsets
  printk("#define MAG_OFFSET_X  %.2f\n", off_x);
  printk("#define MAG_OFFSET_Y  %.2f\n", off_y);
  printk("#define MAG_OFFSET_Z  %.2f\n", off_z);

  // Print Soft Iron Scale Factors
  printk("#define MAG_SCALE_X   %.4f\n", scale_factor_x);
  printk("#define MAG_SCALE_Y   %.4f\n", scale_factor_y);
  printk("#define MAG_SCALE_Z   %.4f\n", scale_factor_z);
  return 0;
}
