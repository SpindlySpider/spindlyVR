#ifndef POSITION_H
#define POSITION_H

struct quaternion_t {
  float w, x, y, z;
};

struct vector_t {
  float x,y,z;
};

void calculate_pos(float bx, float by, float bz);

void start_positioning_thread(void *, void *, void *);

void rotate_vector_by_quaternion(float q0, float q1, float q2, float q3,
                                 float *bx, float *by, float *bz);

void align_to_tx_frame(float *bx, float *by, float *bz);

#endif
