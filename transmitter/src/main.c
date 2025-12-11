#include "pwm.h"
#include "adc.h"
#include "orientation_handle.h"
#include "qmc5883p.h"
#include "zephyr/device.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

//TODO: move device tree defs into here so they are configurable

// define constants
#define I2C DT_NODELABEL(i2c0)

int32_t _err;

static const struct device *i2c_dev = DEVICE_DT_GET(I2C);

int main(void) {

  // printk("Starting...\n");

  // printk("setting up ADC...\n");
  _err = setup_adc();
  if (_err == 0) {
    return 0;
  }
  // printk("setting up PWM...\n");
  _err = setup_pwm();
  if (_err != 0) {
    return 0;
  }
  
  _err = setup_sensors();
  if (_err != 0) {
    return _err;
  }

  // printk("All pins are ready!\n");

  // qmc_calibration_routine(i2c_dev);
  // run_orientation_loop();

  // while (1) {
    // _err = read_adc();
    // if (_err == 0) {
      // return 0;
    // }
    // read_sensors();
    // printk("mag data: x:%d y:%d z:%d\n",mag_data.x,mag_data.y,mag_data.z);
    // k_msleep(500);
  // }

  return 0;
}
