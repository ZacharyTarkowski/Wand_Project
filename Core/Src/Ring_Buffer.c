/*
 * Ring_Buffer.c
 *
 *  Created on: Sep 16, 2024
 *      Author: zacht
 */
#include "Ring_Buffer.h"

buffer_element* ring_buffer[2];
u32 write_index[2];

void ring_buffer_init(u32 size)
{
	ring_buffer_print_function = &MPU6050_print_data;

	//fixme numringbuffers
	for(int i = 0; i< 2; i++)
	{
		ring_buffer[i] =  malloc(size * sizeof(buffer_element));
	}
}

RING_BUFFER_ERROR_TYPE ring_buffer_MPU6050_read_and_store(Mpu_6050_handle_s* handle, u8 buf_sel)
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
			ring_buffer_write_element(&tmp_data, buf_sel);
		}
	}

	//fixme
	return status;
}

RING_BUFFER_ERROR_TYPE ring_buffer_write_element(buffer_element* data, u8 buf_sel)
{
	if(data !=0)
	{
		memcpy(&ring_buffer[buf_sel][write_index[buf_sel]++], data, sizeof(buffer_element));

		if(write_index[buf_sel] >= RING_BUFFER_SIZE)
		{
			write_index[buf_sel] = 0;
			uart_println("ring buffer rollover");
		}

		return RING_BUFFER_SUCCESS;
	}
	else
	{
		return RING_BUFFER_FAIL;
	}
}

RING_BUFFER_ERROR_TYPE ring_buffer_read_element(u32 index, buffer_element* data, u8 buf_sel)
{
	*data = ring_buffer[buf_sel][index];

	//return 0;
}

void ring_buffer_print_all_elements(u8 buf_sel)
{
	for(int i = 0; i< RING_BUFFER_SIZE; i++)
	{
		//fixme
		buffer_element data;
		ring_buffer_read_element(i, &data,buf_sel);
		ring_buffer_print_function(&data);
	}
}

void ring_buffer_print_to_write_index(u8 buf_sel)
{
	for(int i = 0; i< write_index[buf_sel]; i++)
	{
		//fixme
		buffer_element data;
		ring_buffer_read_element(i, &data,buf_sel);
		ring_buffer_print_function(&data);
	}
}

void ring_buffer_print_element(u32 index,u8 buf_sel)
{
	buffer_element data;
	ring_buffer_read_element(index, &data,buf_sel);
	ring_buffer_print_function(&data);
}

//prospective print function given by a function pointer init
//RING_BUFFER_ERROR_TYPE ring_buffer_set_print_function(size_t* fp_buffer_element_print)
//{
//	g_ring_buffer_print_function = fp_buffer_element_print;
//}


void ring_buffer_clear(u8 buf_sel)
{
	memset(ring_buffer[buf_sel],0x0,sizeof(ring_buffer));
	write_index[buf_sel] = 0;
}

void ring_buffer_destroy()
{
	free(ring_buffer);
}
