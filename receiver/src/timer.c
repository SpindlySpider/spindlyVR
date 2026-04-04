// timer file to manage timer for broadcasting data and working out phase sync
//
#include "data_handle.h"
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
  // setup adc and broadcast timer
  // incredibly useful:
  // https://github.com/zephyrproject-rtos/hal_nordic/tree/master/nrfx/samples/src/nrfx_timer
  int err;

  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER3), IRQ_PRIO_LOWEST,
              nrfx_timer_irq_handler, &adc_timer, 0);

  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER4), IRQ_PRIO_LOWEST,
              nrfx_timer_irq_handler, &rx_timer, 0);

  uint32_t frequency = NRF_TIMER_BASE_FREQUENCY_GET(adc_timer.p_reg);

  nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(frequency);

  err = nrfx_timer_init(&adc_timer, &config, NULL);
  err = nrfx_timer_init(&rx_timer, &config, NULL);

  nrfx_timer_clear(&adc_timer);
  nrfx_timer_clear(&rx_timer);

  // convert kilohertz to micro seconds
  uint32_t khz_to_us = (uint32_t)(16000 / CONFIG_RX_ADC_FREQUENCY);

  // uint32_t hz_to_us = (uint32_t)(CONFIG_RX_BROADCAST_FREQUENCY);
  //
  // // uint32_t adc_desired_ticks = nrfx_timer_us_to_ticks(&adc_timer,
  // khz_to_us); uint32_t rx_desired_ticks = nrfx_timer_us_to_ticks(&rx_timer,
  // hz_to_us);

  // uint32_t adc_sample_period_us = 10;
  // uint32_t adc_desired_ticks =
  //     nrfx_timer_us_to_ticks(&adc_timer, adc_sample_period_us);

  uint32_t adc_target_hz = CONFIG_RX_ADC_FREQUENCY * 1000;
  uint32_t adc_desired_ticks = 16000000 / adc_target_hz;

  nrfx_timer_extended_compare(&adc_timer, NRF_TIMER_CC_CHANNEL0,
                              adc_desired_ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

  // convert hz to microseconds
  uint32_t hz_to_us = 1000000/(uint32_t)(CONFIG_RX_BROADCAST_FREQUENCY);
  uint32_t rx_desired_ticks = nrfx_timer_us_to_ticks(&rx_timer, hz_to_us);

  // nrfx_timer_extended_compare(&adc_timer, NRF_TIMER_CC_CHANNEL0,
  //                             // adc_desired_ticks,
  //                             khz_to_us, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
  //                             true);

  // nrfx_timer_extended_compare(&adc_timer, NRF_TIMER_CC_CHANNEL0,
  //                             adc_desired_ticks,
  //                             NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  // broadcast for x hz
  nrfx_timer_extended_compare(&rx_timer, NRF_TIMER_CC_CHANNEL0,
                              rx_desired_ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

  nrfx_timer_extended_compare(&rx_timer, NRF_TIMER_CC_CHANNEL1,
                              rx_desired_ticks,
                              NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, false);

  return 0;
}
