#ifndef ADC_H
#define ADC_H

// #include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

struct cached_sin_cos_t {
  double sine[CONFIG_TX_SAMPLE_NUMBER];
  double cosine[CONFIG_TX_SAMPLE_NUMBER];
};

int read_adc();

int setup_adc();

int tx_matched_filter(int16_t *signal_buf, struct cached_sin_cos_t *cached_s_c);

int setup_sin_cos_cache(struct cached_sin_cos_t *cached_s_c);

void start_adc_thread(void *, void *, void *);

#endif
