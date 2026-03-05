#include "orientation_handle.h"
#include "imu.h"
#include "lib/MadgwickAHRS.h"
#include "qmc5883p.h"
#include "zephyr/device.h"
#include "zephyr/sys/util_macro.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// define constants
#define I2C DT_NODELABEL(i2c0)
#define GRAVITY 9.81f

static const struct device *i2c_dev = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(qmc_5883p)));

// purpose of this file is to convert all sensor data into quaternions using
// Madgwick filter

struct qmc_data mag_data;
struct sensor_data_struct raw_sensor_data;

int setup_sensors() {
  int ret;
  ret = setup_imu();
  if (ret != 0) {
    return ret;
  }
  ret = setup_qmc5883p(i2c_dev);
  ret = setup_imu();
  if (ret != 0) {
    return ret;
  }
  return 0;
}

int read_sensors() {
  int ret;

  ret = read_imu(&raw_sensor_data);
  if (ret != 0) {
    return ret;
  }
  ret = qmc_read_sensor_data(i2c_dev, &mag_data);
  if (ret != 0) {
    return ret;
  }

  raw_sensor_data.mag_x = mag_data.x;
  raw_sensor_data.mag_y = mag_data.y;
  raw_sensor_data.mag_z = mag_data.z;

  if (IS_ENABLED(CONFIG_DEBUG_SHOW_SENSOR_DATA)) {
    printk("acc | x:%.2f | y:%.2f | z:%.2f |\ngyro | x:%.2f | y:%.2f | z:%.2f "
           "\nmag | x%d | y%d | z%d \n",
           raw_sensor_data.accel_x, raw_sensor_data.accel_y,
           raw_sensor_data.accel_z, raw_sensor_data.gyro_x,
           raw_sensor_data.gyro_y, raw_sensor_data.gyro_z,
           raw_sensor_data.mag_x, raw_sensor_data.mag_y, raw_sensor_data.mag_z);
  }
  return 0;
}

int convert_raw(struct sensor_data_struct *data) {
  data->accel_x /= GRAVITY;
  data->accel_y /= GRAVITY;
  data->accel_z /= GRAVITY;
  return 0;
}

void update_madgwick(struct sensor_data_struct *data) {
  MadgwickAHRSupdate(data->gyro_x, -data->gyro_z, data->gyro_y, data->accel_x,
                     -data->accel_z, data->accel_y, data->mag_x, -data->mag_z,
                     -data->mag_y);
  // MadgwickAHRSupdateIMU(data->gyro_x, -data->gyro_z, data->gyro_y,
  // data->accel_x, -data->accel_z, data->accel_y);
}

void run_orientation_loop() {
  setup_sensors();
  static int startup_counter = 0;

  while (1) {
    read_sensors();
    convert_raw(&raw_sensor_data);
    if (!IS_ENABLED(CONFIG_DEBUG_SHOW_SENSOR_DATA)) {
      update_madgwick(&raw_sensor_data);

      if (startup_counter <= 1000) {
        // fix to true north
        startup_counter++;
        beta = 5.0f;
      } else {
        beta = 0.8f;
      }

      static int print_counter = 0;
      if (print_counter++ >= 5) {

        // q0 = W (Scalar), q1 = X, q2 = Y, q3 = Z
        printk("%.4f,%.4f,%.4f,%.4f\n", (double)q0, (double)q1, (double)q2,
               (double)q3);

        print_counter = 0;
      }
      // update thread structure
      k_msleep(5);
    } else {
      k_msleep(500);
    }
  }
}
