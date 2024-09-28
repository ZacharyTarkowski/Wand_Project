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
#define RING_BUFFER_SIZE 1024
#define RING_BUFFER_ERROR_TYPE HAL_StatusTypeDef
#define buffer_element Mpu_6050_data_s

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

static u32 write_index;
//static buffer_element ring_buffer[RING_BUFFER_SIZE];
static buffer_element* ring_buffer;

//void* g_ring_buffer_print_function;


void ring_buffer_init(u32 size);

RING_BUFFER_ERROR_TYPE ring_buffer_write_element(buffer_element* data);
RING_BUFFER_ERROR_TYPE ring_buffer_read_element();
void ring_buffer_print_all_elements();
void ring_buffer_print_to_write_index();
void ring_buffer_print_element(u32 index);

void ring_buffer_clear();
RING_BUFFER_ERROR_TYPE ring_buffer_MPU6050_read_and_store(Mpu_6050_handle_s* handle);

u32 ring_buffer_get_index();

void ring_buffer_destroy();


#endif /* RING_BUFFER_H_ */
