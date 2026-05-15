#include "data_handle.h"
// #include "zephyr/dt-bindings/gpio/gpio.h"
#include "hal/nrf_timer.h"
#include "nrfx_timer.h"
#include "position.h"
#include "timer.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include <math.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>

K_MSGQ_DEFINE(positioning_queue, sizeof(struct solver_packet_t), 10, 4);

// precompute this
static const float FIVE_DIVIDE_TWO = 5.0f / 2.0f;
// since accel in g use 1.0f instead of 9.80665f
static const float GRAVITY = 1.0f;
static const float GRAVITY_MS = 9.80665f;

// found with the kalman noise finder script
static const float X_VP = 0.005295973033866432;
static const float Y_VP = 0.0040412814941998025;
static const float Z_VP = 0.0024122114372555487;
static const float X_VA = 0.0021756922347726873;
static const float Y_VA = 0.0014266803384266505;
static const float Z_VA = 0.0033356483850830957;

// define kalman estimation per axis
static struct kalman_state_t kalman_x = {0, 0, 1, 0, 0, 1};
static struct kalman_state_t kalman_y = {0, 0, 1, 0, 0, 1};
static struct kalman_state_t kalman_z = {0, 0, 1, 0, 0, 1};
// timestamp for kalman calculation
static int64_t kalman_timestamp = 0;

// Calibrate magnetic moment
static float cal_magnetic_moment = 104.1666666f;

// used to tell if the offset should be recalibrated
static uint8_t calibrate_orientation = 0;
static struct quaternion_t rx_offset_q = {1, 0.0001f, 0.0001f, 0.0001f};

static const struct gpio_dt_spec cal_button_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(cal_button_pin), gpios);

static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins) {
  // button pressed, set the next postion to reset q offset
  printk("recalibration button pressed\n");
  calibrate_orientation = 1;
}

int config_calibration_button() {
  // https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/drivers/gpio/button_interrupt
  uint8_t err;
  err = device_is_ready(cal_button_pin.port);
  // is bool if 1 then is ready
  if (err != 1) {
    printk("Button setup error: %d\n", err);
    k_msleep(1000);
    return err;
  }

  // this func is used to setup the calibration button to recenter offset
  // quaternions between RX and TX
  gpio_pin_configure_dt(&cal_button_pin, GPIO_INPUT | GPIO_PULL_UP);
  printk("setup config button\n");
  k_msleep(1000);

  // on button press gen interupt
  gpio_pin_interrupt_configure(cal_button_pin.port, cal_button_pin.pin,
                               GPIO_INT_EDGE_FALLING);
  printk("setup button inturput\n");
  k_msleep(1000);

  // add button callback
  gpio_init_callback(&button_cb_data, button_pressed, BIT(cal_button_pin.pin));
  gpio_add_callback(cal_button_pin.port, &button_cb_data);
  printk("setup button callback: done!\n");
  k_msleep(1000);
  return 0;
}

float kalman_update(struct kalman_state_t *kalman, float p_m, float a_m,
                    float dt, float vp, float va) {
  // TODO: this does not need to be a monolothic function like this
  //
  // p_m is position measured, a_m is acceleration measured
  // predict next system state
  float dt_squared = dt * dt;
  // float va_squared = va * va;
  kalman->p_est =
      kalman->p_est + (kalman->v_est * dt) + ((a_m * (dt_squared)) / 2);
  kalman->v_est = kalman->v_est + a_m * dt;

  // predict covariance matrix
  float new_C0 = kalman->C0 + ((kalman->C1 + kalman->C2) * dt) +
                 (kalman->C3 * dt_squared) + (va * dt_squared);
  float new_C1 = kalman->C1 + (kalman->C3 * dt) + (va * (dt_squared * dt));
  float new_C2 = kalman->C2 + (kalman->C3 * dt) + (va * (dt_squared * dt));
  float new_C3 = kalman->C3 + (va * dt_squared);
  kalman->C0 = new_C0;
  kalman->C1 = new_C1;
  kalman->C2 = new_C2;
  kalman->C3 = new_C3;

  // calculate kalman gain
  float K_denominator = kalman->C0 + ((kalman->C1 + kalman->C2) * dt) +
                        (kalman->C3 * dt_squared) + (vp * vp);
  float K0 = (kalman->C0 + (kalman->C1 * dt)) / K_denominator;
  float K1 = (kalman->C2 + (kalman->C3 * dt)) / K_denominator;

  // update state estimate
  kalman->p_est =
      kalman->p_est + (K0 * (p_m - kalman->p_est - (kalman->v_est * dt)));
  kalman->v_est =
      kalman->v_est + (K1 * (p_m - kalman->p_est - (kalman->v_est * dt)));

  // update covariance estimation
  kalman->C0 = new_C0 - (K0 * (new_C0 + (new_C2 * dt)));
  kalman->C1 = new_C1 - (K0 * (new_C1 + (new_C3 * dt)));
  kalman->C2 = new_C2 - (K1 * (new_C0 + (new_C2 * dt)));
  kalman->C3 = new_C3 - (K1 * (new_C1 + (new_C3 * dt)));
  return kalman->p_est;
}

struct vector_t rotate_vector(struct vector_t raw_pos, struct quaternion_t rx_q,
                              struct quaternion_t inverse_rx_q) {
  // ensure normalise the quaterion

  // multiple tx quaternion by inverse of RX quaterionion to find the relative
  // orientation to rotate vectors

  // struct quaternion_t q_realtive = multiply_quaternion(inverse_rx_q, tx_q);
  //
  // struct quaternion_t inverse_q_relative = inverse_quaternion(&q_realtive);
  //
  struct quaternion_t mag_vector = {
      .w = 0.0f, .x = raw_pos.x, .y = raw_pos.y, .z = raw_pos.z};

  // rotate magnetic vectors (rotation via shoemakers paper q^-1 * V * q)
  mag_vector = multiply_quaternion(inverse_rx_q, mag_vector);
  mag_vector = multiply_quaternion(mag_vector, rx_q);

  // rotated RX vector
  struct vector_t rx_pos_rot = {mag_vector.x, mag_vector.y, mag_vector.z};
  return rx_pos_rot;
}

struct vector_t calculate_pos(float bx, float by, float bz, float a_x,
                              float a_y, float a_z) {
  // cache values so this is slightly less expensive
  // also this is entirely taken from research paper
  float sqrt_bx_by = sqrtf((bx * bx) + (by * by));

  float c1 = bz / sqrt_bx_by;
  // calculate with +/-
  float c2_part1 = (3 * c1 / 4);
  float c2_part2 = (sqrtf((9 * (c1 * c1)) + 8) / 4);

  float c2_plus = c2_part1 + c2_part2;
  float c2_minus = c2_part1 - c2_part2;

  // TODO: decide which +/- here or calculate both -/+ and decide at the end?
  // not sure this is the right place since (5) has +/- but (6)(c2) does not
  // only has + but it says only considering first quadrant?
  float c2 = c2_plus;

  float x0_denominator =
      (4 * PI * powf(1 + (c2 * c2), FIVE_DIVIDE_TWO) * sqrt_bx_by);
  float x0_numerator = 3 * cal_magnetic_moment * c2;
  // cube root 1/3
  float x0 = cbrtf(x0_numerator / x0_denominator);

  // need to apply sign from rotated vector

  float zp = c2 * x0;

  float xp = x0 / sqrtf(1 + powf((by / bx), 2.0f));
  float yp = sqrtf((x0 * x0) - (xp * xp));

  // will need to apply zp, sign after ? might not since the + - will probbaly
  // produce the correct results
  // TODO: use gyroscope to keep sign if angular veloctiy is high, otherwise
  // changes quadrant

  float sign_z = (zp >= 0.0f) ? 1.0f : -1.0f;
  xp = (bx * sign_z >= 0.0f) ? xp : -xp;
  yp = (by * sign_z >= 0.0f) ? yp : -yp;

  // TODO: add rejection outliers, if position jumps alot

  // printk(
  //     " c1: %-7.3f | c2: %-7.3f | x0: %-7.3f | x0_n: %-7.3f | x0_d:
  //     %-7.3f\n", c1, c2, x0, x0_numerator, x0_denominator);
  // print values

  // print values for debug and calibration

  // printk("X: %-10.8f | Y: %-10.8f | Z: %-10.8f | accel X: %-10.8f | accel Y:"
  //        " %-10.8f | accel Z: %-10.8f\n",
  //        xp, yp, zp, a_x, a_y, a_z);
  // TODO: add kalman filter activation here in another func
  // int64_t now_ms = k_uptime_get();
  // float dt =
  // (kalman_timestamp == 0) ? 0.0025f : (now_ms - kalman_timestamp) / 1000.0f;
  // kalman_timestamp = now_ms;
  float dt = 0.01f;

  float xp_filtered = kalman_update(&kalman_x, xp, a_x, dt, X_VP, X_VA);
  float yp_filtered = kalman_update(&kalman_y, yp, a_y, dt, Y_VP, Y_VA);
  float zp_filtered = kalman_update(&kalman_z, zp, a_z, dt, Z_VP, Z_VA);

  // printk("Kalman X: %.4f | Y: %.4f | Z: %.4f\n", xp_filtered, yp_filtered,
  //        zp_filtered);
  struct vector_t smooth = {xp_filtered, yp_filtered, zp_filtered};
  return smooth;
}

void calculate_linear_acceleration(float *a_x, float *a_y, float *a_z,
                                   struct quaternion_t rx_q,
                                   struct quaternion_t inverse_rx_q) {
  // calculated for all axis for later kalman filter, for position estimation
  // only, only z axis is required
  struct quaternion_t gravity_q = {.w = 0, .x = 0, .y = 0, .z = -GRAVITY};
  // rotate sensor frame to world frame and then remove gravity
  // ISSUE: this will cause errors if this is not the correct convention.

  // orginal
  gravity_q = multiply_quaternion(inverse_rx_q, gravity_q);
  gravity_q = multiply_quaternion(gravity_q, rx_q);

  // changed
  // gravity_q = multiply_quaternion(rx_q, gravity_q);
  // gravity_q = multiply_quaternion(gravity_q, inverse_rx_q);
  // Remove gravity from the accelerometer data and convert to m/s^2
  // orginally a + grav.axis
  *a_x = (*a_x + gravity_q.x) * GRAVITY_MS;
  *a_y = (*a_y + gravity_q.y) * GRAVITY_MS;
  *a_z = (*a_z + gravity_q.z) * GRAVITY_MS;
}

void recalibrate_q_offset(struct quaternion_t inverse_rx_q,
                          struct quaternion_t tx_q) {
  rx_offset_q = multiply_quaternion(tx_q, inverse_rx_q);
  normalise_quaternion(&rx_offset_q);
}

void start_positioning_thread(void *, void *, void *) {
  struct solver_packet_t incoming_data;
  config_calibration_button();
  // setup calibration button config here

  while (1) {
    // TODO: maybe dont wait on queue to free up, just keep guessing using accel
    // data and if a position item comes in use that to stablize
    k_msgq_get(&positioning_queue, &incoming_data, K_FOREVER);

    struct vector_t rx_pos_raw = {incoming_data.bx, incoming_data.by,
                                  incoming_data.bz};

    // NOTE: forced postive to see how position does, if this fixes most
    // positoning stuff then phase sync sign needs fixing

    // struct vector_t rx_pos_raw = {
    //     fabsf(incoming_data.bx), fabsf(incoming_data.by),
    //     fabsf(incoming_data.bz)};

    // printk(" RAW X: %-7.3f | RAW Y: %-7.3f | RAW Z: %-7.3f\n", rx_pos_raw.x,
    //        rx_pos_raw.y, rx_pos_raw.z);

    struct quaternion_t q_tx_frame = {
        .w = incoming_data.tx_q0,
        .x = incoming_data.tx_q1,
        .y = incoming_data.tx_q2,
        .z = incoming_data.tx_q3,
    };

    struct quaternion_t q_rx_frame = {
        .w = incoming_data.rx_q0,
        .x = incoming_data.rx_q1,
        .y = incoming_data.rx_q2,
        .z = incoming_data.rx_q3,
    };

    // acceleration raw values
    float a_x = incoming_data.accel_x;
    float a_y = incoming_data.accel_y;
    float a_z = incoming_data.accel_z;

    normalise_quaternion(&q_rx_frame);
    // find inverse of rx quaternion
    struct quaternion_t inverse_rx_q = inverse_quaternion(&q_rx_frame);

    calculate_linear_acceleration(&a_x, &a_y, &a_z, q_rx_frame, inverse_rx_q);

    // NOTE: I recon the RX and TX are not algined properly
    // this was correct, switching to 6 axis magdwick with manual calibration
    // printk("tx_q0: %4.2f | tx_q1: %4.2f | tx_q2: %4.2f | tx_q3: %4.2f |
    // rx_q0: "
    //        "%4.2f | rx_q1: %4.2f | rx_q2: %4.2f | rx_q3: %4.2f\n",
    //        q_tx_frame.w, q_tx_frame.x, q_tx_frame.y, q_tx_frame.z,
    //        q_rx_frame.w, q_rx_frame.x, q_rx_frame.y, q_rx_frame.z);

    // normalise
    normalise_quaternion(&q_tx_frame);

    if (calibrate_orientation == 1) {
      rx_offset_q = inverse_rx_q;
      normalise_quaternion(&rx_offset_q);
      printk("CAL: raw q=(%.4f, %.4f, %.4f, %.4f)\n", q_rx_frame.w,
             q_rx_frame.x, q_rx_frame.y, q_rx_frame.z);
      printk("CAL: offset q=(%.4f, %.4f, %.4f, %.4f)\n", rx_offset_q.w,
             rx_offset_q.x, rx_offset_q.y, rx_offset_q.z);

      struct quaternion_t check = multiply_quaternion(rx_offset_q, q_rx_frame);
      printk("CAL: check (offset*raw) = (%.4f, %.4f, %.4f, %.4f) [should be "
             "near 1,0,0,0]\n",
             check.w, check.x, check.y, check.z);

      calibrate_orientation = 0;
    }

    // printk("correction q | q0: %6.4f | q1: %6.4f | q2: %6.4f | q3: %6.4f\n",
    //        rx_offset_q.w, rx_offset_q.x, rx_offset_q.y, rx_offset_q.z);
    // apply the offset

    // orginally
    // q_rx_frame = multiply_quaternion(rx_offset_q, q_rx_frame); // changed
    // q_rx_frame = multiply_quaternion(q_rx_frame, q_rx_frame); // changed
    q_rx_frame = multiply_quaternion(rx_offset_q, q_rx_frame);
    // q_rx_frame = multiply_quaternion(q_rx_frame,rx_offset_q);
    normalise_quaternion(&q_rx_frame);

    // printk("After offset: q_rx_frame = %.4f, %.4f, %.4f, %.4f\n",
    // q_rx_frame.w,
    //        q_rx_frame.x, q_rx_frame.y, q_rx_frame.z);

    inverse_rx_q = inverse_quaternion(&q_rx_frame);

    // rotated vector
    struct vector_t rot_vector;

    // struct vector_t rot_vector = {incoming_data.bx, incoming_data.by,
    //                               incoming_data.bz};
    //
    // rot_vector = rotate_vector(rx_pos_raw, q_tx_frame, inverse_rx_q);
    // update to only use internal reference
    rot_vector = rotate_vector(rx_pos_raw, q_rx_frame, inverse_rx_q);
    // printk("bx: %5.3f | by: %5.3f | bz: %5.3f | rot_bx: %5.3f | rot_by:
    // %5.3f"
    //        "| rot_bz: %5.3f\n",
    //        rx_pos_raw.x, rx_pos_raw.y, rx_pos_raw.z, rot_vector.x,
    //        rot_vector.y, rot_vector.z);

    // rot_vector = rx_pos_raw;

    // printk(" ROT X: %-7.3f | ROT Y: %-7.3f | ROT Z: %-7.3f\n", rot_vector.x,
    //        rot_vector.y, rot_vector.z);

    // estimate position from rotated vector
    struct vector_t smooth;

    smooth =
        calculate_pos(rot_vector.x, rot_vector.y, rot_vector.z, a_x, a_y, a_z);

    printk("X: %5.4f | Y: %5.4f | Z: %5.4f | q0: %5.4f | q1: %5.4f | "
           "q2: %5.4f | q3: %5.4f  \n",
           smooth.x, smooth.y, smooth.z, incoming_data.rx_q0,
           incoming_data.rx_q1, incoming_data.rx_q2, incoming_data.rx_q3);
  }
}
