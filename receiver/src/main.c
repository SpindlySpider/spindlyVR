// #include "adc.h"
#include "orientation_handle.h"
#include "qmc5883p.h"
#include "receive_data.h"

#include "zephyr/device.h"
#include "zephyr/sys/util_macro.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY 7

K_THREAD_STACK_DEFINE(adc_stack, THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(receive_stack, THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(orientation_stack, THREAD_STACK_SIZE);

struct k_thread adc_thread_data;
struct k_thread transmit_thread_data;
struct k_thread orientation_thread_data;

int32_t _err;

int main(void) {

  k_msleep(1000);
  printk("Starting...\n");
  k_msleep(1000);
  //
  // printk("setting up ADC...\n");
  //
  // printk("setting up Sensors...\n");
  // _err = setup_sensors();
  // if (_err != 0) {
  //   return _err;
  // }
  //
  // printk("All pins are ready!\n");
  //
  // printk("setting up BlueTooth...\n");
  // // _err = init_transmit();
  // // if (_err != 0) {
  // //   return 0;
  // // }


  // set this up last, or atleast before ADC so timer is available
  printk("Setting up reciever\n");
  k_msleep(1000);
  setup_receiver();

  // if (IS_ENABLED(CONFIG_RUN_QMC_CALIBRATION)) {
  //   static const struct device *i2c_dev =
  //       DEVICE_DT_GET(DT_BUS(DT_NODELABEL(qmc_5883p)));
  //   qmc_calibration_routine(i2c_dev);
  //   // exit from function
  //   return 0;
  // }
  //
  // // k_thread_create(&adc_thread_data, adc_stack, K_THREAD_STACK_SIZEOF(adc_stack),
  // //                 start_adc_thread, NULL, NULL, NULL, THREAD_PRIORITY, 0,
  // //                 K_NO_WAIT);
  //
  // k_thread_create(&orientation_thread_data, orientation_stack,
  //                 K_THREAD_STACK_SIZEOF(orientation_stack),
  //                 run_orientation_loop, NULL, NULL, NULL, THREAD_PRIORITY, 0,
  //                 K_NO_WAIT);
  // while (1) {
  //   k_sleep(K_FOREVER);
  // }
  return 0;
}
