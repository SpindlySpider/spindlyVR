#include "orientation_handle.h"
#include "MadgwickAHRS.h"
#include "data_handle.h"
#include "imu.h"
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

static const struct device *i2c_dev =
    DEVICE_DT_GET(DT_BUS(DT_NODELABEL(qmc_5883p)));

// purpose of this file is to convert all sensor data into quaternions using
// Madgwick filter

static struct qmc_data mag_data;
static struct sensor_data_struct raw_sensor_data;
// gyro bias for each axis
float x_gb, y_gb, z_gb;

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
  raw_sensor_data.gyro_x -= x_gb;
  raw_sensor_data.gyro_y -= y_gb;
  raw_sensor_data.gyro_z -= z_gb;
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
  MadgwickAHRSupdate(data->gyro_x, data->gyro_y, data->gyro_z, data->accel_x,
                     data->accel_y, data->accel_z,data->mag_x, -data->mag_y,
                     data->mag_z);
  // MadgwickAHRSupdateIMU(data->gyro_x, data->gyro_y, data->gyro_z, data->accel_x,
  //                       data->accel_y, data->accel_z);
}

void calibrate_gyro() {
  int samples = 500;
  float sum_x, sum_y, sum_z;
  // allow sensor settle after initation
  printk("starting gyro calibration\n");
  k_msleep(1000);
  for (int i = 0; i < samples; i++) {
    read_sensors();
    // printk("gyro x: %4.3f | gyro y: %4.3f | gyro z: %4.3f\n",
    //        raw_sensor_data.gyro_x, raw_sensor_data.gyro_y,
    //        raw_sensor_data.gyro_z);
    sum_x += raw_sensor_data.gyro_x;
    sum_y += raw_sensor_data.gyro_y;
    sum_z += raw_sensor_data.gyro_z;
    // 100 hz 5 seconds
    k_msleep(10);
    // k_msleep(300);
  }
  x_gb = sum_x / samples;
  y_gb = sum_y / samples;
  z_gb = sum_z / samples;
  printk("Finished gyro calibration!\n");
}

void run_orientation_loop(void *, void *, void *) {
  setup_sensors();
  int startup_counter = 0;
  int print_counter = 0;
  bool finished_startup = false;
  // NOTE: if there are performance issue can move the start up function out of
  // while loop to stop beta reassignment each loop

  // calibrate gyro
  calibrate_gyro();

  while (1) {
    read_sensors();
    // ISSUE: Zepyhyr converts to m2 so we unconvert back to g
    convert_raw(&raw_sensor_data);
    update_madgwick(&raw_sensor_data);

    if (!finished_startup && startup_counter <= 1000) {
      // NOTE: fix to true north, start super sensitive and then reduce beta
      startup_counter++;
      // beta = 5.0f;
    } else {
      beta = 0.3f;
    }

    if (IS_ENABLED(CONFIG_DEBUG_SHOW_QUATERNIONS_DATA)) {
      if (print_counter++ >= 5) {
        // q0 = W (Scalar), q1 = X, q2 = Y, q3 = Z
        printk("%.4f,%.4f,%.4f,%.4f\n", (double)q0, (double)q1, (double)q2,
               (double)q3);

        print_counter = 0;
      }
    }

    // update data container
    update_orientation_data(q0, q1, q2, q3);

    if (IS_ENABLED(CONFIG_IS_RECEIVER)) {
      update_rx_accel_gyro_data(raw_sensor_data.accel_x,
                                raw_sensor_data.accel_y,
                                raw_sensor_data.accel_z, raw_sensor_data.gyro_x,
                                raw_sensor_data.gyro_y, raw_sensor_data.gyro_z);
    }
    // TODO: if receiver update accelermeter & gyro data
    // for dynamic gyro callibration & accel for ghost position differntation

    k_msleep(1000 / CONFIG_TX_BROADCAST_FREQUENCY);
  }
}
