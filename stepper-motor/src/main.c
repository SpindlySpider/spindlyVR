#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// define coils
#define COIL_1 DT_NODELABEL(coil_1)
#define COIL_2 DT_NODELABEL(coil_2)
#define COIL_3 DT_NODELABEL(coil_3)
#define COIL_4 DT_NODELABEL(coil_4)

// constant for sleep time
#define SLEEP_TIME 100
// constant of number of turns on coil
#define NUM_TURNS 300

// Create array of gpio pins
static const struct gpio_dt_spec coils[] = {
    GPIO_DT_SPEC_GET(COIL_1, gpios), GPIO_DT_SPEC_GET(COIL_2, gpios),
    GPIO_DT_SPEC_GET(COIL_3, gpios), GPIO_DT_SPEC_GET(COIL_4, gpios)};

const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

void set_pins(int pin1, int pin2, int pin3, int pin4) {
  // args: takes pins 1-4 with which are activated
      gpio_pin_set_dt(&coils[0], pin1);
      gpio_pin_set_dt(&coils[1], pin2);
      gpio_pin_set_dt(&coils[2], pin3);
      gpio_pin_set_dt(&coils[3], pin4);
}

int main(void) {

  /* Configure Candidates */
  gpio_pin_configure(gpio0, 17, GPIO_OUTPUT_ACTIVE);
  gpio_pin_configure(gpio0, 20, GPIO_OUTPUT_ACTIVE);
  gpio_pin_configure(gpio0, 22, GPIO_OUTPUT_ACTIVE);
  gpio_pin_configure(gpio0, 24, GPIO_OUTPUT_ACTIVE);

  printk("Starting...\n");
  // check that all pins are ready
  printk("Checking GPIO pins are ready...\n");
  int num_coils = 4;
  printk("You are driving %d coils", num_coils);
  for (int i = 0; i < num_coils; i++) {
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
    printk("Current step is %d the current pin we are driving is %d and we are "
           "on rotation %d\n",
           step, current_pin, num_rotations);
    // cycle through pins and turn current on and the rest off
    switch (current_pin) {
    case 0:
        set_pins(1,0,0,0);
      break;
    case 1:
        set_pins(1,0,1,0);
      break;
    case 2:
        set_pins(0,0,1,0);
      break;
    case 3:
        set_pins(0,1,1,0);
      break;
    case 4:
        set_pins(0,1,0,0);
      break;
    case 5:
        set_pins(0,1,0,1);
      break;
    case 6:
        set_pins(0,0,0,1);
      break;
    case 7:
        set_pins(1,0,0,1);
      break;
    }

    current_pin++;
    step++;
    if (current_pin >= 7) {
      current_pin = 0;
    }
    if (step == 200) {
      // number of steps for a max rotation
      num_rotations++;
      step = 0;
    }
    if (NUM_TURNS == num_rotations) {
      printk("Finished rotating %d times\n", NUM_TURNS);
      return 0;
    }

    k_msleep(SLEEP_TIME);
  }

  return 0;
}
