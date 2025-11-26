#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PWM_PERIOD_NS 20000000
#define PWM_PULSE_WIDTH 1400000

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_stepper));

int main(void) {
  if (!pwm_is_ready_dt(&pwm_led0)) {
    printk("Error: PWM device %s is not ready\n", pwm_led0.dev->name);
    return 0;
  }
  int err = pwm_set_dt(&pwm_led0,PWM_PERIOD_NS, PWM_PULSE_WIDTH);
  if (err){
    printk("Error in pmw_set_dt(), err %d",err);
    return 0;
  }

  return 0;
}
