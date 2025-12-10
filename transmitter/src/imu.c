#include "imu.h"
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>

#define IMU DT_NODELABEL(mpu6500)

static const struct device *imu_dev = DEVICE_DT_GET(IMU);

struct sensor_value accel[3];
struct sensor_value gyro[3];

float sensor_val_to_float(const struct sensor_value *val) {
  return (float)val->val1 + ((float)val->val2 / 1000000.0f);
}

int setup_imu() {
  if (!device_is_ready(imu_dev)) {
    return 0;
  }
  return 1;
}

int read_imu() {
  int ret = sensor_sample_fetch(imu_dev);
  if (ret < 0) {
    printk("fetching IMU data failed (Error: %d)\n", ret);
  }

  sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, gyro);
  float ax = sensor_val_to_float(&accel[0]);
  float ay = sensor_val_to_float(&accel[1]);
  float az = sensor_val_to_float(&accel[2]);

  float gx = sensor_val_to_float(&gyro[0]);
  float gy = sensor_val_to_float(&gyro[1]);
  float gz = sensor_val_to_float(&gyro[2]);

  printk("A: %.2f %.2f %.2f | G: %.2f %.2f %.2f\n", ax, ay, az, gx, gy, gz);
  return 0;
}
