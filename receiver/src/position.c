#include "data_handle.h"
#include <math.h>
// using static K value here which will need to be adjusted for each Rx, this is
// because 3*m*s/4*PI is constant and can be substituted for as a single value.
// further more it
//
static int calibration_value = 1;
static struct rx_data_signal_t rx_local_data;

void calculate_pos() {
  read_tx_data(&rx_local_data);
  float bx = rx_local_data.adc_x_amp;
  float by = rx_local_data.adc_y_amp;
  float bz = rx_local_data.adc_z_amp;

  // cache XY magnitude
  float bxy_mag = sqrtf(powf(bx, 2) + powf(by, 2));

  // prevent divide by zero
  if (bxy_mag < 0.0001f) {
    bxy_mag = 0.0001f;
  }

  float c1 = bz / bxy_mag;
  float c2 = ((3.0f * c1) / 4.0f) + (sqrtf(9.0f * powf(c1, 2) + 8.0f) / 4.0f);

  float denominator = powf(1.0f + powf(c2, 2), 2.5f) * bxy_mag;
  float x0 = cbrtf((calibration_value * c2) / denominator);

  float zp = c2 * x0;

  float xp_mag = x0 / sqrtf(1.0f + powf((by / bx), 2));
  float yp_mag = sqrtf(powf(x0, 2) - powf(xp_mag, 2));

  // 3. QUADRANT UNPACKING (The Magic Trick)
  // sign(x) = sign(Bx) * sign(z)
  // sign(y) = sign(By) * sign(z)

  float xp_final = xp_mag;
  float yp_final = yp_mag;

  // We use standard C signbit logic. If (Bx is negative) XOR (Z is negative)...
  if ((bx < 0.0f) != (zp < 0.0f)) {
    xp_final = -xp_mag;
  }

  if ((by < 0.0f) != (zp < 0.0f)) {
    yp_final = -yp_mag;
  }

  // Done! You now have true 3D spatial coordinates!
  // will need to store these in data structure and transmit to pc dongle
  printk("Final Position: X: %.3f, Y: %.3f, Z: %.3f\n", xp_final, yp_final, zp);
}
