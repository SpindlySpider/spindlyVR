#ifndef ADC_H
#define ADC_H

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/adc.h>

struct cached_sin_cos_t {
  double sine[CONFIG_TX_SAMPLE_NUMBER];
  double cosine[CONFIG_TX_SAMPLE_NUMBER];
};

int read_adc();

int setup_adc();

#endif
