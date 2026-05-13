#ifndef POSITION_H
#define POSITION_H

struct kalman_state_t {
    float p_est;       // estimated position
    float v_est;       // estimated velocity
    float C0, C1, C2, C3;  // covariance matrix
};

struct vector_t calculate_pos(float bx, float by, float bz, float a_x, float a_y,
                   float a_z);

void start_positioning_thread(void *, void *, void *);

void rotate_vector_by_quaternion(float q0, float q1, float q2, float q3,
                                 float *bx, float *by, float *bz);

void align_to_tx_frame(float *bx, float *by, float *bz);

#endif
