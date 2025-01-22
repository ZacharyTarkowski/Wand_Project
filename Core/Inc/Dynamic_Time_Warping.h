#ifndef H_DYNAMIC_TIME_WARPING
#define H_DYNAMIC_TIME_WARPING

#include "General_Utils.h"
#include "Ring_Buffer.h"
#include "dd_dtw.h"

#include <stdlib.h>
#include <math.h>

#define MIN(a,b) ((a) < (b)  ? (a) : (b) )
#define MAX(a,b) ((a) > (b)  ? (a) : (b) )

//Highest signed integer. Still undecided on using UINTS, but 
//0xFFFFFFFF causes problems for signed addition and 0x7FFFFFFF is sufficiently high enough for this data anyway
#define INFINITY 0x7FFFFFFF

#define DTW_WINDOW_PERCENTAGE 10

typedef struct DTW_Result
{
    int32_t x_accel_result;
    int32_t y_accel_result;
    int32_t z_accel_result;
    int32_t x_gyro_result;
    int32_t y_gyro_result;
    int32_t z_gyro_result;
} DTW_Result;


HAL_StatusTypeDef dtw_init();
DTW_Result DTW_Distance(ring_buffer_s* s, ring_buffer_s* t);
void print_dtw_result(DTW_Result* data);
void free_dtw_buf();
















#endif //H_DYNAMIC_TIME_WARPING