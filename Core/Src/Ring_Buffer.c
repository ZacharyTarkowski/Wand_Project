/*
 * Ring_Buffer.c
 *
 *  Created on: Sep 16, 2024
 *      Author: zacht
 */
#include "Ring_Buffer.h"

//buffer_element* ring_buffer[2];
//u32 write_index[2];

RING_BUFFER_ERROR_TYPE ring_buffer_init(ring_buffer_s* pRingBuffer, u32 size)
{
	RING_BUFFER_ERROR_TYPE status = RING_BUFFER_SUCCESS;

	pRingBuffer->buffer = malloc(size * sizeof(buffer_element));
	
	if(pRingBuffer->buffer == 0)
	{
		uart_println("Failed ring buffer init!");
		status = RING_BUFFER_FAIL;
	}
	else
	{
		pRingBuffer->print_function = &MPU6050_print_data;
		pRingBuffer->size = size;
		pRingBuffer->write_index = 0;
	}

	return status;
}

//fixme probably need this out of ring buffer source
RING_BUFFER_ERROR_TYPE ring_buffer_MPU6050_read_and_store(Mpu_6050_handle_s* handle, ring_buffer_s* pRingBuffer)
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
			ring_buffer_write_element(pRingBuffer, &tmp_data);
		}
	}

	//fixme
	return status;
}

RING_BUFFER_ERROR_TYPE ring_buffer_write_element(ring_buffer_s* pRingBuffer, buffer_element* data)
{
	RING_BUFFER_ERROR_TYPE status = RING_BUFFER_FAIL;
	
	if(data !=0)
	{
		memcpy(&pRingBuffer->buffer[pRingBuffer->write_index++], data, sizeof(buffer_element));

		if(pRingBuffer->write_index >= pRingBuffer->size)
		{
			pRingBuffer->write_index = 0;
			uart_println("ring buffer rollover");
		}

		status =  RING_BUFFER_SUCCESS;
	}
	else
	{
		status =  RING_BUFFER_FAIL;
	}

	return status;
}

RING_BUFFER_ERROR_TYPE ring_buffer_read_element(ring_buffer_s* pRingBuffer, u32 index, buffer_element* data)
{
	RING_BUFFER_ERROR_TYPE status = RING_BUFFER_FAIL;

	if(index < pRingBuffer->size)
	{
		*data = pRingBuffer->buffer[index];
		status = RING_BUFFER_SUCCESS;
	}

	return status;
}

void ring_buffer_print_all_elements(ring_buffer_s* pRingBuffer)
{
	for(int i = 0; i< pRingBuffer->size; i++)
	{
		//fixme
		buffer_element data;
		ring_buffer_read_element(pRingBuffer, i, &data);
		pRingBuffer->print_function(&data);
	}
}

void ring_buffer_print_to_write_index(ring_buffer_s* pRingBuffer)
{
	for(int i = 0; i< pRingBuffer->write_index; i++)
	{
		//fixme
		buffer_element data;
		ring_buffer_read_element(pRingBuffer, i, &data);
		pRingBuffer->print_function(&data);
	}
}

void ring_buffer_print_element(ring_buffer_s* pRingBuffer, u32 index)
{
	if(index < pRingBuffer->size)
	{
		buffer_element data;
		ring_buffer_read_element(pRingBuffer, index, &data);
		pRingBuffer->print_function(&data);
	}
}

void ring_buffer_clear(ring_buffer_s* pRingBuffer)
{
	memset(pRingBuffer->buffer,0x0,pRingBuffer->size * sizeof(buffer_element));
	pRingBuffer->write_index = 0;
}

void ring_buffer_destroy(ring_buffer_s* pRingBuffer)
{
	free(pRingBuffer->buffer);
}

RING_BUFFER_ERROR_TYPE ring_buffer_MPU6050_parse_data_buffer(ring_buffer_s* pRingBuffer, s16* data)
{
	RING_BUFFER_ERROR_TYPE status = RING_BUFFER_FAIL;
	if(data != 0)
	{
		buffer_element tmp_data;
		for(int i = 0; i< pRingBuffer->size; i++)
		{
			memset(&tmp_data,0,sizeof(buffer_element));
			
			MPU6050_utility_data_buffer_to_struct(&data[i*(MPU_6050_NUM_DIMS)], &tmp_data);

			ring_buffer_write_element(pRingBuffer, &tmp_data);
		}
		status = RING_BUFFER_SUCCESS;
	}
	return status;
}


