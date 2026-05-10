#include "pwm.h"
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define COIL_1 DT_NODELABEL(coil_1)
#define COIL_2 DT_NODELABEL(coil_2)

#define PERIOD PWM_KHZ(32)
#define DUTY_CYCLE 0.33f
#define PULSE ((uint32_t)(PERIOD * DUTY_CYCLE))
// #define PULSE ( PERIOD / 2 )

// Get PWM pins to drive transmitter
static const struct pwm_dt_spec pwms[] = {PWM_DT_SPEC_GET(COIL_1),
                                          PWM_DT_SPEC_GET(COIL_2)};

int setup_pwm() {
  // check transmission coils are ready
  for (int i = 0; i < 2; i++) {
    // go through all pin indexes
    if (!pwm_is_ready_dt(&pwms[i])) {
      // issue with this
      printk("Pin %d is not ready\n", i);
      return -1;
    }
  }
  int err;

  printk("setting pwm to 32 khz\n");
  // setting just one for now to not fry board - direct driving the tx antenna
  printk("PERIOD=%lu, PULSE=%lu\n", (unsigned long)PERIOD, (unsigned long)PULSE);
  // printk("PERIOD=%lu, PULSE=%lu\n", (unsigned long)PERIOD, (unsigned long) 0);
  err = pwm_set_dt(&pwms[0], PERIOD, PULSE);
  // err = pwm_set_dt(&pwms[0], PERIOD,0);
  if (err) {
    // issue with this
    printk("Could not setup PWM\n");
    return -1;
  }
  pwm_set_dt(&pwms[1], PERIOD, PULSE);
  // pwm_set_dt(&pwms[1], PERIOD, 0);
  return 0;
}
