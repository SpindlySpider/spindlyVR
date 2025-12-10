#include "orientation_handle.h"
#include "imu.h"
#include "lib/MadgwickAHRS.h"
#include "qmc5883p.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// define constants
#define I2C DT_NODELABEL(i2c0)
#define GYRO_SENSITIVITY_2000 16.4f
#define ACCEL_SENSITIVITY_2G 16384.0f
#define DEG_TO_RAD 0.01745329f

static const struct device *i2c_dev = DEVICE_DT_GET(I2C);

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

  // printk("acc | x:%.2f | y:%.2f | z:%.2f |\ngyro | x:%.2f | y:%.2f | z:%.2f \n "
  //        "mag | x%d | y%d | z%d \n",
  //        raw_sensor_data.accel_x, raw_sensor_data.accel_y,
  //        raw_sensor_data.accel_z, raw_sensor_data.gyro_x,
  //        raw_sensor_data.gyro_y, raw_sensor_data.gyro_z, raw_sensor_data.mag_x,
  //        raw_sensor_data.mag_y, raw_sensor_data.mag_z);

  return 0;
}

int convert_raw(struct sensor_data_struct *data) {
  // convert accelerometer to G's
  data->accel_x = data->accel_x / ACCEL_SENSITIVITY_2G;
  data->accel_y = data->accel_y / ACCEL_SENSITIVITY_2G;
  data->accel_z = data->accel_z / ACCEL_SENSITIVITY_2G;

  data->gyro_x = (data->gyro_x / GYRO_SENSITIVITY_2000) * DEG_TO_RAD;
  data->gyro_y = (data->gyro_y / GYRO_SENSITIVITY_2000) * DEG_TO_RAD;
  data->gyro_z = (data->gyro_z / GYRO_SENSITIVITY_2000) * DEG_TO_RAD;
  return 0;
}

void update_madgwick(struct sensor_data_struct *data) {
  MadgwickAHRSupdate(data->gyro_x, data->gyro_y, data->gyro_z, data->accel_x,
                     data->accel_y, data->accel_z, data->mag_x, data->mag_y,
                     data->mag_z);
}

void run_orientation_loop() {
  setup_sensors();
  while (1) {
    read_sensors();
    convert_raw(&raw_sensor_data);
    update_madgwick(&raw_sensor_data);

    static int print_counter = 0;
    if (print_counter++ >= 100) {

      // q0 = W (Scalar), q1 = X, q2 = Y, q3 = Z
      printk("%.4f,%.4f,%.4f,%.4f\n", (double)q0,
             (double)q1, (double)q2, (double)q3);

      print_counter = 0;
    }

    k_msleep(10);
  }
}
