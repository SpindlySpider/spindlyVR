#include "adc.h"
#include "orientation_handle.h"
#include "pwm.h"
#include "qmc5883p.h"
#include "transmit_data.h"

#include "zephyr/device.h"
#include "zephyr/sys/util_macro.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

// #define THREAD_STACK_SIZE 2048
#define THREAD_STACK_SIZE 4096
#define THREAD_PRIORITY 7

// K_THREAD_STACK_DEFINE(adc_stack, THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(adc_stack, 8192);
K_THREAD_STACK_DEFINE(transmit_stack, THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(orientation_stack, THREAD_STACK_SIZE);

struct k_thread adc_thread_data;
struct k_thread transmit_thread_data;
struct k_thread orientation_thread_data;

int32_t _err;

void start_precision_clock(void) {
  const struct device *const clk = DEVICE_DT_GET_ONE(nordic_nrf_clock);
  if (!device_is_ready(clk)) {
    printk("Clock device not ready\n");
    return;
  }

  /* Start the high-frequency crystal oscillator (HFXO) */
  clock_control_on(clk, CLOCK_CONTROL_NRF_SUBSYS_HF);

  printk("HFXO precision clock started.\n");
}

int main(void) {

  k_msleep(2000);
  printk("Starting...\n");
  k_msleep(2000);

  printk("Start precison clock...\n");
  start_precision_clock();
  k_msleep(1000);

  printk("setting up PWM...\n");
  _err = setup_pwm();
  if (_err != 0) {
    return 0;
  }

  printk("setting up Sensors...\n");
  _err = setup_sensors();
  if (_err != 0) {
    return _err;
  }

  printk("All pins are ready!\n");

  printk("setting up transmitter...\n");
  _err = setup_transmitter();
  if (_err != 0) {
    return 0;
  }

  if (IS_ENABLED(CONFIG_RUN_QMC_CALIBRATION)) {
    static const struct device *i2c_dev =
        DEVICE_DT_GET(DT_BUS(DT_NODELABEL(qmc_5883p)));
    qmc_calibration_routine(i2c_dev);
    // exit from function
    return 0;
  }

  k_thread_create(&adc_thread_data, adc_stack, K_THREAD_STACK_SIZEOF(adc_stack),
                  start_adc_thread, NULL, NULL, NULL, THREAD_PRIORITY, 0,
                  K_NO_WAIT);

  k_thread_create(&transmit_thread_data, transmit_stack,
                  K_THREAD_STACK_SIZEOF(transmit_stack), start_transmit_thread,
                  NULL, NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);

  k_thread_create(&orientation_thread_data, orientation_stack,
                  K_THREAD_STACK_SIZEOF(orientation_stack),
                  run_orientation_loop, NULL, NULL, NULL, THREAD_PRIORITY, 0,
                  K_NO_WAIT);
  while (1) {
    k_sleep(K_FOREVER);
  }
  return 0;
}
