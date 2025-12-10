#include "qmc5883p.h"
#include "pwm.h"
#include "adc.h"
#include "imu.h"
#include "zephyr/device.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// define constants
#define I2C DT_NODELABEL(i2c0)

int32_t _err;

static const struct device *i2c_dev = DEVICE_DT_GET(I2C);

int main(void) {

  printk("Starting...\n");

  printk("setting up ADC...\n");
  _err = setup_adc();
  if (_err == 0) {
    return 0;
  }
  printk("setting up PWM...\n");
  _err = setup_pwm();
  if (_err == 0) {
    return 0;
  }

  setup_imu();
  setup_qmc5883p(i2c_dev);
  struct qmc_data mag_data ;

  printk("All pins are ready!\n");

  // qmc_calibration_routine(i2c_dev);

  while (1) {
    _err = read_adc();
    if (_err == 0) {
      return 0;
    }
    read_imu();
    qmc_read_sensor_data(i2c_dev,&mag_data);
    printk("mag data: x:%d y:%d z:%d\n",mag_data.x,mag_data.y,mag_data.z);
    k_msleep(500);
  }

  return 0;
}
