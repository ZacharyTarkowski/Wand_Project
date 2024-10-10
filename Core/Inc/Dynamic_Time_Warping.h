#ifndef H_DYNAMIC_TIME_WARPING
#define H_DYNAMIC_TIME_WARPING

#include "Ring_Buffer.h"
#include <stdlib.h>

#define MIN(a,b) ((a) < (b)  ? (a) : (b) )

#define MAX_INT 0xFFFFFFFFFF

#define DTW_LENGTH RING_BUFFER_SIZE * RING_BUFFER_SIZE * MPU_6050_NUM_DIMS

//static u32 dtw_buf[MPU_6050_NUM_DIMS][RING_BUFFER_SIZE][RING_BUFFER_SIZE];

typedef struct DTW_Result
{
    s32 x_accel_result;
    s32 y_accel_result;
    s32 z_accel_result;
    s32 x_gyro_result;
    s32 y_gyro_result;
    s32 z_gyro_result;
} DTW_Result;

typedef struct dtw_array_data
{
    u32 x_accel_value;
    u32 y_accel_value;
    u32 z_accel_value;

    u32 x_gyro_value;
    u32 y_gyro_value;
    u32 z_gyro_value;

} dtw_array_data;


DTW_Result DTW_Distance( buffer_element* s, buffer_element* t, u32 s_length, u32 t_length);
















#endif //H_DYNAMIC_TIME_WARPING