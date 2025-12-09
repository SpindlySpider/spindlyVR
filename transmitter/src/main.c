#include "zephyr/device.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// define constants
#define COIL_1 DT_NODELABEL(coil_1)
#define COIL_2 DT_NODELABEL(coil_2)
#define IMU DT_NODELABEL(mpu6500)

#define PERIOD PWM_KHZ(32)
#define DUTY_CYCLE 0.5
#define PULSE PERIOD *DUTY_CYCLE

int16_t buf;
int32_t val_mv;
int32_t err;


float sensor_val_to_float(const struct sensor_value *val)
{
    return (float)val->val1 + ((float)val->val2 / 1000000.0f);
}

// TODO: get logs working rather than just printk

struct adc_sequence sequence = {
    .buffer = &buf,
    /* buffer size in bytes, not number of samples */
    .buffer_size = sizeof(buf),
};

// Get ADC transmission amplitude pin
static const struct adc_dt_spec tx_adc = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
// Get PWM pins to drive transmitter
static const struct pwm_dt_spec pwms[] = {PWM_DT_SPEC_GET(COIL_1),
                                          PWM_DT_SPEC_GET(COIL_2)};
// Get I2C for IMU
static const struct device *imu_dev = DEVICE_DT_GET(IMU);

struct sensor_value accel[3];
struct sensor_value gyro[3];
struct sensor_value mag[3];

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

int setup_pwm() {
  // check transmission coils are ready
  for (int i = 0; i < 2; i++) {
    // go through all pin indexes
    if (!pwm_is_ready_dt(&pwms[i])) {
      // issue with this
      printk("Pin %d is not ready\n", i);
      return 0;
    }
  }

  printk("setting pwm to 32 khz\n");
  // setting just one for now to not fry board - direct driving the tx antenna
  pwm_set_dt(&pwms[0], PERIOD, PULSE);
  return 1;
}

int setup_imu() {
  if (!device_is_ready(imu_dev)) {
    return 0;
  }
  return 1;
}

int read_imu() {
  int ret = sensor_sample_fetch(imu_dev);
  if (ret < 0) {
    printk("fetching IMU data failed (Error: %d)\n", ret);
  }

  // sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
  // sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, gyro);
  // float ax = sensor_val_to_float(&accel[0]);
  // float ay = sensor_val_to_float(&accel[1]);
  // float az = sensor_val_to_float(&accel[2]);

  // float gx = sensor_val_to_float(&gyro[0]);
  // float gy = sensor_val_to_float(&gyro[1]);
  // float gz = sensor_val_to_float(&gyro[2]);

  // printk("A: %.2f %.2f %.2f | G: %.2f %.2f %.2f\n", ax, ay, az, gx, gy, gz);
  //
  //
  sensor_channel_get(imu_dev, SENSOR_CHAN_MAGN_XYZ, mag);
  float mx = sensor_val_to_float(&mag[0]);
  float my = sensor_val_to_float(&mag[1]);
  float mz = sensor_val_to_float(&mag[2]);

  printk("mag x:%d y:%d z:%d\n", mx,my,mz);

  return 0;
}

int main(void) {

  printk("Starting...\n");

  printk("setting up ADC...\n");
  err = setup_adc();
  if (err == 0) {
    return 0;
  }
  printk("setting up PWM...\n");
  err = setup_pwm();
  if (err == 0) {
    return 0;
  }

  setup_imu();

  printk("All pins are ready!\n");
  while (1) {
    // err = read_adc();
    if (err == 0) {
      return 0;
    }
    // read_imu();
    k_msleep(500);
  }

  return 0;
}
