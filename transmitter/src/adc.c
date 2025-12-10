
#include "adc.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/adc.h>

int16_t buf;
int32_t val_mv;
int32_t err;

struct adc_sequence sequence = {
    .buffer = &buf,
    /* buffer size in bytes, not number of samples */
    .buffer_size = sizeof(buf),
};

// Get ADC transmission amplitude pin
static const struct adc_dt_spec tx_adc = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

int read_adc() {
  // NOTE: may need to reset buffer variable here, if program crashes in future
  // check this
  err = adc_read(tx_adc.dev, &sequence);
  if (err < 0) {
    printk("Could not read (%d)", err);
    return 0;
  }
  val_mv = (int32_t)buf;
  err = adc_raw_to_millivolts_dt(&tx_adc, &val_mv);

  if (err < 0) {
    printk(" (value in mV not available)\n");
  } else {
    printk(" = %d mV\n", val_mv);
  }
  return 1;
}

int setup_adc() {
  if (!adc_is_ready_dt(&tx_adc)) {
    // issue with this ADC pin
    printk("transmittor ADC is not ready\n");
    return 0;
  }

  err = adc_channel_setup_dt(&tx_adc);
  if (err < 0) {
    // LOG_ERR("Could not setup channel #%d (%d)", 0, err);
    printk("Could not setup channel #%d (%d)", 0, err);
    return 0;
  }

  // set up adc sequence
  err = adc_sequence_init_dt(&tx_adc, &sequence);
  if (err < 0) {
    printk("Could not initalize sequnce");
    return 0;
  }

  return 1;
}

