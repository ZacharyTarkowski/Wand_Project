#ifndef H_DYNAMIC_TIME_WARPING
#define H_DYNAMIC_TIME_WARPING

#include "Ring_Buffer.h"

#define MIN(a,b) ((a) < (b)  ? (a) : (b) )

#define MAX_INT 0xFFFF

#define DTW_LENGTH RING_BUFFER_SIZE * RING_BUFFER_SIZE * MPU_6050_NUM_DIMS

//static u32 dtw_buf[MPU_6050_NUM_DIMS][RING_BUFFER_SIZE][RING_BUFFER_SIZE];

u32 DTW_Distance( buffer_element* s, buffer_element* t);
















#endif //H_DYNAMIC_TIME_WARPING