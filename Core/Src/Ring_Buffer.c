/*
 * Ring_Buffer.c
 *
 *  Created on: Sep 16, 2024
 *      Author: zacht
 * 
 * Iteratively good and bad ideas for data movement.
 * Archive of attempts:
 * Attempt 1:
 * 	Buffer elements were structs. Ring buffers were structs with a malloc'd pointer to the buffer element struct.
 * 
 * Attempt 2:
 * 	This attempt will be basically making each ring buffer a 2d matrix, malloc'd on init.
 * 
 * 
 * 
 * 
 */
#include "Ring_Buffer.h"

RING_BUFFER_ERROR_TYPE ring_buffer_init(ring_buffer_s* pRingBuffer, buffer_element* initialData, u32 initial_data_size, u32 num_dims, u32 dim_size)
{
	RING_BUFFER_ERROR_TYPE status = RING_BUFFER_FAIL;

	if(num_dims != 0 && dim_size != 0)
	{
		pRingBuffer->num_dims = num_dims;
		pRingBuffer->dim_size = dim_size;
		pRingBuffer->write_index = 0;
		pRingBuffer->read_index = 0;
		pRingBuffer->rollover_count = 0;

		pRingBuffer->print_function = &MPU6050_print_data;
		status = RING_BUFFER_SUCCESS;

	}

	if(status == RING_BUFFER_SUCCESS )
	{
		pRingBuffer->buffer = malloc(num_dims * sizeof(buffer_element*));
	}

	if(pRingBuffer->buffer == 0)
	{
		status = RING_BUFFER_FAIL;
	}

	if(status == RING_BUFFER_SUCCESS )
	{
		for(u32 i = 0; i< num_dims; i++)
		{
			pRingBuffer->buffer[i] = malloc(dim_size * sizeof(buffer_element));

			if(pRingBuffer->buffer[i] == 0)
			{
				status = RING_BUFFER_FAIL;
			}
		}
	}

	if(status == RING_BUFFER_SUCCESS && initialData != 0)
	{
		pRingBuffer->write_index = (initial_data_size / num_dims) / sizeof(buffer_element);

		for(u32 i = 0; i< pRingBuffer->write_index; i++)
		{
			for(u32 j = 0; j< pRingBuffer->num_dims; j++)
			{
				pRingBuffer->buffer[j][i] = initialData[(i*num_dims) + j];
			}		
		}
	
	}

	if(status != HAL_OK)
	{
		uart_println("Failed ring buffer init!");
		ring_buffer_destroy(pRingBuffer);
	}

	return status;
}

//fixme probably need this out of ring buffer source
RING_BUFFER_ERROR_TYPE ring_buffer_MPU6050_read_and_store(Mpu_6050_handle_s* handle, ring_buffer_s* pRingBuffer)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	u8 num_samples = 0;

	Mpu_6050_data_s tmp_data;

	buffer_element tmpBuf[pRingBuffer->num_dims];

	status = MPU6050_get_fifo_data(handle, &num_samples);

	if(status == HAL_OK)
	{
		for(int i = 0; i<num_samples; i++)
		{
			memset(&tmp_data,0,sizeof(Mpu_6050_data_s));
			MPU6050_read_fifo_data(i, &tmp_data);

			tmpBuf[0] = (buffer_element)tmp_data.x_accel_data;
			tmpBuf[1] = (buffer_element)tmp_data.y_accel_data;
			tmpBuf[2] = (buffer_element)tmp_data.z_accel_data;
			tmpBuf[3] = (buffer_element)tmp_data.x_gyro_data;
			tmpBuf[4] = (buffer_element)tmp_data.y_gyro_data;
			tmpBuf[5] = (buffer_element)tmp_data.z_gyro_data;
			
			ring_buffer_write_element(pRingBuffer, &tmpBuf);
		}
	}
	else
	{
		uart_println("Breakpoint");
	}

	//fixme
	return status;
}

//fixme probably need this out of ring buffer source
RING_BUFFER_ERROR_TYPE ring_buffer_MPU6050_get_accel_sample(Mpu_6050_handle_s* handle, ring_buffer_s* pRingBuffer)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	u8 num_samples = 0;

	s16 tmp_buf[3];
	buffer_element ring_tmp_buf[pRingBuffer->num_dims];

	status = MPU6050_get_accel_sample(handle, tmp_buf);

	if(status == HAL_OK)
	{

		ring_tmp_buf[0] = (buffer_element)tmp_buf[0];
		ring_tmp_buf[1] = (buffer_element)tmp_buf[1];
		ring_tmp_buf[2] = (buffer_element)tmp_buf[2];

		for(int i = 3; i< pRingBuffer->num_dims; i++)
		{
			ring_tmp_buf[i] = 0;
		}
		
		ring_buffer_write_element(pRingBuffer, &ring_tmp_buf);
	}
	else
	{
		uart_println("Breakpoint");
	}

	//fixme
	return status;
}

//call by using something like buffer_element name[pRingBuffer->num_dims] = {1,2,3};
RING_BUFFER_ERROR_TYPE ring_buffer_write_element(ring_buffer_s* pRingBuffer, buffer_element* data)
{
	RING_BUFFER_ERROR_TYPE status = RING_BUFFER_FAIL;
	
	if(data !=0)
	{
		for(u32 i = 0; i< pRingBuffer->num_dims; i++)
		{
			memcpy(&pRingBuffer->buffer[i][pRingBuffer->write_index], &data[i], sizeof(buffer_element));
		}
		pRingBuffer->write_index++;

		//bad, read index doesnt need to exist. it is either 0 (no rollovers yet) or write_index after a rollover
		if(pRingBuffer->rollover_count > 0)
		{
			pRingBuffer->read_index++;
		}

		if(pRingBuffer->write_index >= pRingBuffer->dim_size)
		{
			pRingBuffer->write_index = 0;
			pRingBuffer->read_index = 0;
			pRingBuffer->rollover_count++;
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

	if(index < pRingBuffer->dim_size)
	{
		for(u32 i = 0; i< pRingBuffer->num_dims; i++)
		{
			data[i] = pRingBuffer->buffer[i][index];
		}
		status = RING_BUFFER_SUCCESS;
	}

	return status;
}

buffer_element ring_buffer_read(ring_buffer_s* pRingBuffer, u32 dim, u32 index)
{

	if(index < pRingBuffer->dim_size)
	{
		if(pRingBuffer->read_index + index < pRingBuffer->dim_size)
		{
			return pRingBuffer->buffer[dim][pRingBuffer->read_index + index];
		}
		else 
		{
			return pRingBuffer->buffer[dim][index - (pRingBuffer->dim_size - pRingBuffer->read_index)];
		}
	}

}

//not a ring buffer function, used to read an array at an index and wrap around
//starts at an arbitray index and returns the element 
buffer_element read_buffer_wraparound(buffer_element* data, u32 start_index, u32 max_index, u32 index )
{
	if(start_index < max_index)
	{
		if(start_index + index < max_index)
		{
			return data[start_index + index];
		}
		else 
		{
			return data[index - (max_index - start_index)];
		}
	}

}

void ring_buffer_print_all_elements(ring_buffer_s* pRingBuffer)
{
	for(u32 i = 0; i< pRingBuffer->dim_size; i++)
	{
		for(u32 j = 0; j< pRingBuffer->num_dims; j++)
		{
			uart_printf(ringBufPrint,pRingBuffer->buffer[j][i]);
		}
		uart_println("");
	}
	
}

void ring_buffer_print_all_elements_from_read_index(ring_buffer_s* pRingBuffer)
{
	for(u32 i = 0; i< pRingBuffer->dim_size; i++)
	{
		for(u32 j = 0; j< pRingBuffer->num_dims; j++)
		{
			uart_printf(ringBufPrint, ring_buffer_read(pRingBuffer, j, i));
		}
		uart_println("");
	}
	
}

void ring_buffer_print_to_write_index(ring_buffer_s* pRingBuffer)
{
	for(u32 i = 0; i< pRingBuffer->write_index; i++)
	{
		for(u32 j = 0; j< pRingBuffer->num_dims; j++)
		{
			uart_printf(ringBufPrint,(int32_t)pRingBuffer->buffer[j][i]);
		}
		uart_println("");
	}
	
}

void ring_buffer_print_element(ring_buffer_s* pRingBuffer, u32 index)
{
	for(u32 i = 0; i< pRingBuffer->num_dims; i++)
	{
		uart_printf(ringBufPrint,pRingBuffer->buffer[i][index]);
	}
	uart_println("");
}

void ring_buffer_clear(ring_buffer_s* pRingBuffer)
{
	for(u32 i = 0; i< pRingBuffer->num_dims; i++)
	{
		memset(pRingBuffer->buffer[i], 0x0, pRingBuffer->dim_size * sizeof(buffer_element));
	}
	pRingBuffer->write_index = 0;
	pRingBuffer->rollover_count = 0;
}

void ring_buffer_destroy(ring_buffer_s* pRingBuffer)
{
	for(u32 i = 0; i<pRingBuffer->num_dims; i++)
	{
		free(pRingBuffer->buffer[i]);
	}
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

void ring_buffer_MPU6050_apply_mean_centering(ring_buffer_s* pRingBuffer)
{
	if(pRingBuffer->write_index != 0)
	{
		
		buffer_element mean_buffer[pRingBuffer->num_dims];
		memset(&mean_buffer, 0x0, pRingBuffer->num_dims * sizeof(buffer_element)); //fixme size

		for(u32 i = 0; i< pRingBuffer->num_dims; i++)
		{
			for(u32 j = 0; j< pRingBuffer->write_index; j++)
			{
				mean_buffer[i] += (buffer_element)pRingBuffer->buffer[i][j];
			}

			mean_buffer[i] = mean_buffer[i] / (buffer_element)pRingBuffer->write_index;
		}

		for(u32 i = 0; i< pRingBuffer->num_dims; i++)
		{
			for(u32 j = 0; j< pRingBuffer->write_index; j++)
			{
				pRingBuffer->buffer[i][j] -= (buffer_element)mean_buffer[i];
			}
		}
		
	}
}


