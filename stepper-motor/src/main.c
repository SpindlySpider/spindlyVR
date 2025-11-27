#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// define coils
#define COIL_1 DT_NODELABEL(coil_1)
#define COIL_2 DT_NODELABEL(coil_2)
#define COIL_3 DT_NODELABEL(coil_3)
#define COIL_4 DT_NODELABEL(coil_4)

static const struct gpio_dt_spec coils[] = {
    GPIO_DT_SPEC_GET(COIL_1, gpios), GPIO_DT_SPEC_GET(COIL_3, gpios),
    GPIO_DT_SPEC_GET(COIL_2, gpios), GPIO_DT_SPEC_GET(COIL_4, gpios)};

int main(void) { 
  printk("Somthing out in console");
  return 0
}
