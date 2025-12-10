#ifndef ADC_H
#define ADC_H

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/adc.h>

int read_adc();

int setup_adc();

#endif
