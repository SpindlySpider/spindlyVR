#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// define coils
#define COIL_1 DT_NODELABEL(coil_1)
#define COIL_2 DT_NODELABEL(coil_2)
#define COIL_3 DT_NODELABEL(coil_3)
#define COIL_4 DT_NODELABEL(coil_4)

// constant for sleep time
#define SLEEP_TIME 500
// constant of number of turns on coil
#define NUM_TURNS 300


// Create array of gpio pins
static const struct gpio_dt_spec coils[] = {
    GPIO_DT_SPEC_GET(COIL_1, gpios), GPIO_DT_SPEC_GET(COIL_3, gpios),
    GPIO_DT_SPEC_GET(COIL_2, gpios), GPIO_DT_SPEC_GET(COIL_4, gpios)};

int main(void) {
  printk("Starting...\n");
  // check that all pins are ready
  printk("Checking GPIO pins are ready...\n");
  int num_coils = 4;
  printk("You are driving %d coils",num_coils);
  for (int i = 0; i < num_coils-1; i++) {
    // go through all pin indexes
    if (!gpio_is_ready_dt(&coils[i])) {
      // issue with this coil pin
      printk("Pin %d is not ready\n", i);
      return 0;
    }
  }
  printk("All pins are ready! starting driving GPIO...\n");

  // count number of steps so we can count the full rotation.
  int num_rotations = 0;
  int step = 0;
  int current_pin = 0;
  while (1) {
    printk("Current step is %d the current pin we are driving is %d\n",step,current_pin);
    // cycle through pins and turn current on and the rest off
    for (int i = 0; i < num_coils-1; i++) {
      // go through all pin indexes
      if (i == current_pin) {
        // drive pin
        gpio_pin_set_dt(&coils[i], 1);
      } else {
        // set all other pins off
        gpio_pin_set_dt(&coils[i], 0);
      }
    }

    current_pin++;
    step++;
    if (current_pin >= num_coils-1) {
      current_pin = 0;
    }
    if(step == 200){
      // number of steps for a max rotation
      num_rotations++;
      step = 0;
    }
    if (NUM_TURNS == num_rotations){
      printk("Finished rotating %d times\n",NUM_TURNS);
      return 0;
    }

    k_msleep(SLEEP_TIME);
  }
  return 0;
}
