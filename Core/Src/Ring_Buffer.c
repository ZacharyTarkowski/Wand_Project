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

//todo add in the rest of the operations
//todo this also doesnt account for whether or not the vectors are there 

//params : performs a = a (op) b
void MPU6050_data_operation(Mpu_6050_data_s* a, Mpu_6050_data_s* b, operation_e operation)
{
	switch(operation)
	{
		case ADD:
			a->x_accel_data = a->x_accel_data + b->x_accel_data;
			a->y_accel_data = a->y_accel_data + b->y_accel_data;
			a->z_accel_data = a->z_accel_data + b->z_accel_data;
			a->x_gyro_data  = a->x_gyro_data  + b->x_gyro_data ;  
			a->y_gyro_data  = a->y_gyro_data  + b->y_gyro_data ;
			a->z_gyro_data  = a->z_gyro_data  + b->z_gyro_data ;
		break;

		case SUB:
			a->x_accel_data = a->x_accel_data - b->x_accel_data;
			a->y_accel_data = a->y_accel_data - b->y_accel_data;
			a->z_accel_data = a->z_accel_data - b->z_accel_data;
			a->x_gyro_data  = a->x_gyro_data  - b->x_gyro_data ;  
			a->y_gyro_data  = a->y_gyro_data  - b->y_gyro_data ;
			a->z_gyro_data  = a->z_gyro_data  - b->z_gyro_data ;
		break;

		case MULTIPLY:
			a->x_accel_data = a->x_accel_data * b->x_accel_data;
			a->y_accel_data = a->y_accel_data * b->y_accel_data;
			a->z_accel_data = a->z_accel_data * b->z_accel_data;
			a->x_gyro_data  = a->x_gyro_data  * b->x_gyro_data ;  
			a->y_gyro_data  = a->y_gyro_data  * b->y_gyro_data ;
			a->z_gyro_data  = a->z_gyro_data  * b->z_gyro_data ;
		break;

		case DIVIDE:
			a->x_accel_data = a->x_accel_data / b->x_accel_data;
			a->y_accel_data = a->y_accel_data / b->y_accel_data;
			a->z_accel_data = a->z_accel_data / b->z_accel_data;
			a->x_gyro_data  = a->x_gyro_data  / b->x_gyro_data ;  
			a->y_gyro_data  = a->y_gyro_data  / b->y_gyro_data ;
			a->z_gyro_data  = a->z_gyro_data  / b->z_gyro_data ;
		break;

	default:
		uart_println("Unsupported operation on MPU6050 data");
		break;
	}
}


void ring_buffer_MPU6050_apply_vector(ring_buffer_s* pRingBuffer, buffer_element* vector, operation_e operation)
{
	for(u32 i = 0; i< pRingBuffer->size; i++)
	{
		MPU6050_data_operation(&pRingBuffer->buffer[i], vector, SUB);
	}
}

void ring_buffer_MPU6050_apply_mean_centering(ring_buffer_s* pRingBuffer)
{
	if(pRingBuffer->write_index != 0)
	{
		u32 mean_x_accel_data = 0;
		u32 mean_y_accel_data = 0;
		u32 mean_z_accel_data = 0;
		u32 mean_x_gyro_data  = 0;
		u32 mean_y_gyro_data  = 0;
		u32 mean_z_gyro_data  = 0;

		for(u32 i; i<pRingBuffer->write_index; i++)
		{
			mean_x_accel_data += pRingBuffer->buffer[i].x_accel_data;
			mean_y_accel_data += pRingBuffer->buffer[i].y_accel_data;
			mean_z_accel_data += pRingBuffer->buffer[i].z_accel_data;
			mean_x_gyro_data  += pRingBuffer->buffer[i].x_gyro_data ;
			mean_y_gyro_data  += pRingBuffer->buffer[i].y_gyro_data ;
			mean_z_gyro_data  += pRingBuffer->buffer[i].z_gyro_data ;
		}

		mean_x_accel_data = mean_x_accel_data / pRingBuffer->write_index;
		mean_y_accel_data = mean_y_accel_data / pRingBuffer->write_index;
		mean_z_accel_data = mean_z_accel_data / pRingBuffer->write_index;
		mean_x_gyro_data  = mean_x_gyro_data  / pRingBuffer->write_index;
		mean_y_gyro_data  = mean_y_gyro_data  / pRingBuffer->write_index;
		mean_z_gyro_data  = mean_z_gyro_data  / pRingBuffer->write_index;

		for(u32 i; i<pRingBuffer->write_index; i++)
		{
			pRingBuffer->buffer[i].x_accel_data -= mean_x_accel_data;
			pRingBuffer->buffer[i].y_accel_data -= mean_y_accel_data;
			pRingBuffer->buffer[i].z_accel_data -= mean_z_accel_data;
			pRingBuffer->buffer[i].x_gyro_data  -= mean_x_gyro_data  ;
			pRingBuffer->buffer[i].y_gyro_data  -= mean_y_gyro_data  ;
			pRingBuffer->buffer[i].z_gyro_data  -= mean_z_gyro_data  ;
		}
	}
}


