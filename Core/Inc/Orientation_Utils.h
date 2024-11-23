#ifndef H_ORIENTATION_UTILS
#define H_ORIENTATION_UTILS

#include "Ring_Buffer.h"
#include "MPU_6050_Utils.h"
#include "math.h"

#define FREQ   10.0
#define PERIOD (float)1/FREQ
#define ALPHA 0.98


#define RAD_TO_DEG(x) x*(180.0/3.14)
#define DEG_TO_RAD(x) x*(3.14/180.0)


void get_accel_angles(ring_buffer_s* pRingBuffer,float* accel_pitch, float* accel_roll);

float compfilter_tick(ring_buffer_s* pRingBuffer,float* comp_pitch, float* comp_roll);

void ring_buffer_pitch_roll_rotation(ring_buffer_s* pRingBuffer, float theta, float phi);







#endif