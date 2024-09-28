/*
 * Ring_Buffer.c
 *
 *  Created on: Sep 16, 2024
 *      Author: zacht
 */
#include "Ring_Buffer.h"

void ring_buffer_init(u32 size)
{
	ring_buffer = malloc(size * sizeof(buffer_element));
}

RING_BUFFER_ERROR_TYPE ring_buffer_MPU6050_read_and_store(Mpu_6050_handle_s* handle)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	u8 num_samples = 0;

	Mpu_6050_data_s tmp_data;

	status = MPU6050_get_fifo_data(handle, &num_samples);

	if(status == HAL_OK)
	{
		for(int i = 0; i<num_samples; i++)
		{
			memset(&tmp_data,0,sizeof(Mpu_6050_data_s));
			MPU6050_read_fifo_data(i, &tmp_data);
			ring_buffer_write_element(&tmp_data);
		}
	}

	//fixme
	return status;
}

RING_BUFFER_ERROR_TYPE ring_buffer_write_element(buffer_element* data)
{
	memcpy(&ring_buffer[write_index++], data, sizeof(buffer_element));

	if(write_index == RING_BUFFER_SIZE)
	{
		write_index = 0;
		uart_println("ring buffer rollover");
	}

	//return 0;
}

RING_BUFFER_ERROR_TYPE ring_buffer_read_element(u32 index, buffer_element* data)
{
	*data = ring_buffer[index];

	//return 0;
}

void ring_buffer_print_all_elements()
{
	for(int i = 0; i< RING_BUFFER_SIZE; i++)
	{
		//fixme
		buffer_element data;
		ring_buffer_read_element(i, &data);
		MPU6050_print_data(&data);
	}
}

void ring_buffer_print_to_write_index()
{
	for(int i = 0; i< write_index; i++)
	{
		//fixme
		buffer_element data;
		ring_buffer_read_element(i, &data);
		MPU6050_print_data(&data);
	}
}

void ring_buffer_print_element(u32 index)
{
	buffer_element data;
	ring_buffer_read_element(index, &data);
	MPU6050_print_data(&data);
}

//RING_BUFFER_ERROR_TYPE ring_buffer_set_print_function(size_t* fp_buffer_element_print)
//{
//	g_ring_buffer_print_function = fp_buffer_element_print;
//}


void ring_buffer_clear()
{
	memset(ring_buffer,0x0,sizeof(ring_buffer));
	write_index = 0;
}

void ring_buffer_destroy()
{
	free(ring_buffer);
}
