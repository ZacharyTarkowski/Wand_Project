/*
 * Ring_Buffer.h
 *
 *  Created on: Sep 16, 2024
 *      Author: zacht
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include "General_Utils.h"

//fixme
#include "MPU_6050_Utils.h"

//put this in a user config
#define RING_BUFFER_MAX_SIZE 512

//#define RING_BUFFER_ERROR_TYPE HAL_StatusTypeDef
#define buffer_element float

#if 0
#define ringBufPrint ("%f.2, ")
#else
#define ringBufPrint ("%d, ")
#endif

typedef enum 
{
    RING_BUFFER_SUCCESS = HAL_OK,
    RING_BUFFER_FAIL = HAL_ERROR
}RING_BUFFER_ERROR_TYPE;

//rainy day linkedlist
//typedef struct buffer_element
//{
//	buffer_element* head;
//	buffer_element* tail;
//
//	buffer_data data;
//
//}buffer_element;



#define buffer_element_type "MPU_6050_DATA_s"

typedef struct ring_buffer_s
{
    buffer_element** buffer;
    u32 write_index;
    u32 num_dims;
    u32 dim_size;
    void (*print_function)(buffer_element* pData);
} ring_buffer_s;


//extern u32 write_index[2];
//static buffer_element ring_buffer[RING_BUFFER_MAX_SIZE];

//extern buffer_element* ring_buffer[2];


// #define print_function void (*print_function)(buffer_element*) ;

static void (*ring_buffer_print_function)(buffer_element* pData);


RING_BUFFER_ERROR_TYPE ring_buffer_init(ring_buffer_s* pRingBuffer, buffer_element* initialData, u32 initial_data_size, u32 num_dims, u32 dim_size);

RING_BUFFER_ERROR_TYPE ring_buffer_write_element(ring_buffer_s* pRingBuffer, buffer_element* data);
RING_BUFFER_ERROR_TYPE ring_buffer_read_element(ring_buffer_s* pRingBuffer, u32 index, buffer_element* data);

void ring_buffer_print_all_elements(ring_buffer_s* pRingBuffer);
void ring_buffer_print_to_write_index(ring_buffer_s* pRingBuffer);
void ring_buffer_print_element(ring_buffer_s* pRingBuffer, u32 index);

void ring_buffer_clear(ring_buffer_s* pRingBuffer);

//todo fix this?
RING_BUFFER_ERROR_TYPE ring_buffer_MPU6050_read_and_store(Mpu_6050_handle_s* handle, ring_buffer_s* pRingBuffer);

RING_BUFFER_ERROR_TYPE ring_buffer_MPU6050_parse_data_buffer(ring_buffer_s* pRingBuffer, s16* data);

void ring_buffer_clear(ring_buffer_s* pRingBuffer);
void ring_buffer_destroy(ring_buffer_s* pRingBuffer);
void ring_buffer_MPU6050_apply_mean_centering(ring_buffer_s* pRingBuffer);



#endif /* RING_BUFFER_H_ */
