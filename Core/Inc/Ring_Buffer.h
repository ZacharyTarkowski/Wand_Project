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
#define RING_BUFFER_SIZE 128

//#define RING_BUFFER_ERROR_TYPE HAL_StatusTypeDef
#define buffer_element Mpu_6050_data_s

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
    buffer_element* ring_buffer;
    u32* write_ptr;
} ring_buffer_s;

extern u32 write_index[2];
//static buffer_element ring_buffer[RING_BUFFER_SIZE];

extern buffer_element* ring_buffer[2];


// #define print_function void (*print_function)(buffer_element*) ;

static void (*ring_buffer_print_function)(buffer_element* pData);


void ring_buffer_init(u32 size);

RING_BUFFER_ERROR_TYPE ring_buffer_write_element(buffer_element* data, u8 buf_sel);
RING_BUFFER_ERROR_TYPE ring_buffer_read_element(u32 index, buffer_element* data, u8 buf_sel);

void ring_buffer_print_all_elements(u8 buf_sel);
void ring_buffer_print_to_write_index(u8 buf_sel);
void ring_buffer_print_element(u32 index,u8 buf_sel);

void ring_buffer_clear(u8 buf_sel);
RING_BUFFER_ERROR_TYPE ring_buffer_MPU6050_read_and_store(Mpu_6050_handle_s* handle, u8 buf_sel);

u32 ring_buffer_get_index();

void ring_buffer_destroy();


#endif /* RING_BUFFER_H_ */
