#include "imu.h"
#include "orientation_handle.h"
#include "zephyr/dsp/types.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

#define IMU DT_NODELABEL(mpu6500)

static const struct device *imu_dev = DEVICE_DT_GET(IMU);

struct sensor_value accel[3];
struct sensor_value gyro[3];

float sensor_val_to_float(const struct sensor_value *val) {
  return (float)val->val1 + ((float)val->val2 / 1000000.0f);
}

int setup_imu() {
  if (!device_is_ready(imu_dev)) {
    return -1;
  }
  return 0;
}

int read_imu(struct sensor_data_struct *data) {
  int ret = sensor_sample_fetch(imu_dev);
  if (ret < 0) {
    printk("fetching IMU data failed (Error: %d)\n", ret);
  }

  sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
  float64_t ax = sensor_value_to_float(&accel[0]);
  float64_t ay = sensor_value_to_float(&accel[1]);
  float64_t az = sensor_value_to_float(&accel[2]);
  
  float64_t gx = sensor_value_to_float(&gyro[0]);
  float64_t gy = sensor_value_to_float(&gyro[1]);
  float64_t gz = sensor_value_to_float(&gyro[2]);
  //
  // float64_t ax = &accel[0];
  // float ay = &accel[1];
  // float az = &accel[2];
  //
  // float gx = &gyro[0];
  // float gy = &gyro[1];
  // float gz = &gyro[2];



  // printk("A: %.2f %.2f %.2f | G: %.2f %.2f %.2f\n", ax, ay, az, gx, gy, gz);
  data->accel_x = ax;
  data->accel_y = ay;
  data->accel_z = az;
  data->gyro_x = gx;
  data->gyro_y = gy;
  data->gyro_z = gz;
  return 0;
}
