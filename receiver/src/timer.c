// timer file to manage timer for broadcasting data and working out phase sync
//
#include "data_handle.h"
#include "nrfx_templates_config.h"
#include "zephyr/kernel.h"
#include <esb.h>
#include <hal/nrf_timer.h>
#include <math.h>
#include <nrfx_timer.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#import "receive_data.h"

// used for ADC timing
static nrfx_timer_t adc_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(3));
// used to time position broadcast to other nodes, and workout phase sync
static nrfx_timer_t rx_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(4));

int setup_timers() {
  int err;
  // setup adc timer
  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER3), IRQ_PRIO_LOWEST,
              nrfx_timer_irq_handler, &adc_timer, 0);

  uint32_t frequency = NRF_TIMER_BASE_FREQUENCY_GET(adc_timer.p_reg);
  // nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(frequency);
  nrfx_timer_config_t config =
      NRFX_TIMER_DEFAULT_CONFIG(NRF_TIMER_BASE_FREQUENCY_16MHZ);

  err = nrfx_timer_init(&adc_timer, &config, NULL);

  if (err != 0) {
    printk("ADC timer init failed: %d\n", err);
    return err;
  }

  uint32_t adc_target_hz = CONFIG_RX_ADC_FREQUENCY * 1000;
  uint32_t adc_desired_ticks = frequency / adc_target_hz;

  // ADC Timer is allowed to use CLEAR_MASK because it is a metronome
  nrfx_timer_extended_compare(&adc_timer, NRF_TIMER_CC_CHANNEL0,
                              adc_desired_ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

  nrfx_timer_clear(&adc_timer);
  nrfx_timer_enable(&adc_timer);

  // set up receiver timer
  // for ADC and radio
  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER4), IRQ_PRIO_LOWEST,
              nrfx_timer_irq_handler, &rx_timer, 0);

  config.bit_width = NRF_TIMER_BIT_WIDTH_32;
  err = nrfx_timer_init(&rx_timer, &config, NULL);
  if (err != 0) {
    printk("RX timer init failed: %d\n", err);
    return err;
  }

  // Reset Timer 4 every 10ms (160,000 ticks) to keep math numbers small
  // uint32_t rx_loop_ticks = 160000;
  // nrfx_timer_extended_compare(&rx_timer, NRF_TIMER_CC_CHANNEL0,
  // rx_loop_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

  nrfx_timer_clear(&rx_timer);
  nrfx_timer_enable(&rx_timer);

  return 0;
}
