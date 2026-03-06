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

int32_t _err;

int main(void) {

  printk("Starting...\n");

  // NOTE: disabled other setups to test bluetooth
  //
  // printk("setting up ADC...\n");
  // _err = setup_adc();
  // if (_err == 0) {
  //   return 0;
  // }
  // printk("setting up PWM...\n");
  // _err = setup_pwm();
  // if (_err != 0) {
  //   return 0;
  // }
  //
  // printk("setting up Sensors...\n");
  // _err = setup_sensors();
  // if (_err != 0) {
  //   return _err;
  // }

  printk("All pins are ready!\n");


  printk("setting up BlueTooth...\n");
  _err = init_transmit();
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

  // TODO: need to multi thread here
  // One thread for orientation retrieval
  // Another for getting amplitude and phase of tx
  // and finally one for broadcasting data to other devices
  //
  // get orientation

  // NOTE: uncomment once finished with BT tests
  // run_orientation_loop();

  while (1){
    transmit();
  }


  // get amp and phase of tx - to allow for dynamic broadcasting at 32khz?
  // broadcast info - broadcast info to reciever nodes as specified in paper
  // repeat - we can adjust to specific htz

  return 0;
}
