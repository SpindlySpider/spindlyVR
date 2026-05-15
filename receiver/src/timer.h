#ifndef TIMER_H
#define TIMER_H
#include <hal/nrf_timer.h>
#include <nrfx_timer.h>

// used to get time for phase sync of amplitude and phase, also used to broadcast messages to receiver dongle plugged into pc
static nrfx_timer_t rx_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(4));

static nrfx_timer_t adc_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(3));

int setup_timers();

#endif
