#ifndef ADC_H
#define ADC_H

#include <nrfx_saadc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static void saadc_handler(nrfx_saadc_evt_t const *p_event);

int config_saadc();

int config_timer();

int config_ppi();

int tx_matched_filter(int16_t *signal_buf, struct cached_sin_cos_t *cached_s_c);

void start_adc_thread(void *, void *, void *);

#endif
