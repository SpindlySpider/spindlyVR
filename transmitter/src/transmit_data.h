#ifndef DATA_HANDLE_H
#define DATA_HANDLE_H

int init_transmit();
int transmit();
struct bt_data create_payload();
void start_transmit_thread(void *, void *, void *);

#endif
