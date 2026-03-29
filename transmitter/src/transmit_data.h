#ifndef DATA_HANDLE_H
#define DATA_HANDLE_H

void esb_tx_event_handler(struct esb_evt const *event);
int config_esb(void);
int setup_transmitter();
int transmit(float *phase);
void start_transmit_thread(void *, void *, void *);

#endif
