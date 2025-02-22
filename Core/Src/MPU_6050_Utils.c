/*
 * MPU6050erometer_Utils.c
 *
 *  Created on: Aug 21, 2024
 *      Author: zacht
 */


#include "MPU_6050_Utils.h"

/* Master Todo : MPU6050 Utils
 *	some cool sleep stuff can be done, look at power mgmt 1 for more. auto sleep and wakeup for sampling
 *	should make a user config file for user defines
 *
 *	do some fancy calcs with i2c clock speed to determine if sample rates too high for the i2c to go through
 *	fifo overflow?
 */

//#define DEBUG_MPU6050_REGISTER_READ_SUCCESS
#define DEBUG_MPU6050_REGISTER_READ_FAIL

//#define DEBUG_MPU6050_REGISTER_WRITE_SUCCESS
#define DEBUG_MPU6050_REGISTER_WRITE_FAIL

#define MPU6050_PRINT_LEVEL_INFO

u8 g_fifo_num_samples = 0;
u8 g_fifo_read_buffer[MPU6050_FIFO_SIZE] = {0};

const reg reg_name_list[256] =
{
		{ MPU6050_REG_ADDR_SELF_TEST_X , "SELF_TEST_X"},
		{ MPU6050_REG_ADDR_SELF_TEST_Y , "SELF_TEST_Y"},
		{ MPU6050_REG_ADDR_SELF_TEST_Z , "SELF_TEST_Z"},
		{ MPU6050_REG_ADDR_SELF_TEST_A , "SELF_TEST_A"},
		{ MPU6050_REG_ADDR_SMPLRT_DIV , "SMPLRT_DIV"},
		{ MPU6050_REG_ADDR_CONFIG , "CONFIG"},
		{ MPU6050_REG_ADDR_GYRO_CONFIG , "GYRO_CONFIG"},
		{ MPU6050_REG_ADDR_MPU6050_CONFIG , "MPU6050_CONFIG"},
		{ MPU6050_REG_ADDR_FIFO_EN , "FIFO_EN"},
		{ MPU6050_REG_ADDR_I2C_MST_CTRL , "I2C_MST_CTRL"},
		{ MPU6050_REG_ADDR_I2C_SLV0_ADDR , "I2C_SLV0_ADDR"},
		{ MPU6050_REG_ADDR_I2C_SLV0_REG , "I2C_SLV0_REG"},
		{ MPU6050_REG_ADDR_I2C_SLV0_CTRL , "I2C_SLV0_CTRL"},
		{ MPU6050_REG_ADDR_I2C_SLV1_REG , "I2C_SLV1_REG"},
		{ MPU6050_REG_ADDR_I2C_SLV1_CTRL , "I2C_SLV1_CTRL"},
		{ MPU6050_REG_ADDR_I2C_SLV2_ADDR , "I2C_SLV2_ADDR"},
		{ MPU6050_REG_ADDR_I2C_SLV2_REG , "I2C_SLV2_REG"},
		{ MPU6050_REG_ADDR_I2C_SLV2_CTRL , "I2C_SLV2_CTRL"},
		{ MPU6050_REG_ADDR_I2C_SLV4_REG , "I2C_SLV4_REG"},
		{ MPU6050_REG_ADDR_I2C_SLV4_DO , "I2C_SLV4_DO"},
		{ MPU6050_REG_ADDR_I2C_SLV4_CTRL , "I2C_SLV4_CTRL"},
		{ MPU6050_REG_ADDR_I2C_SLV4_DI , "I2C_SLV4_DI"},
		{ MPU6050_REG_ADDR_I2C_MST_STATUS , "I2C_MST_STATUS"},
		{ MPU6050_REG_ADDR_INT_PIN_CFG , "INT_PIN_CFG"},
		{ MPU6050_REG_ADDR_INT_ENABLE , "INT_ENABLE"},
		{ MPU6050_REG_ADDR_INT_STATUS , "INT_STATUS"},
		{ MPU6050_REG_ADDR_MPU6050_XOUT_H , "MPU6050_XOUT_H"},
		{ MPU6050_REG_ADDR_MPU6050_XOUT_L , "MPU6050_XOUT_L"},
		{ MPU6050_REG_ADDR_MPU6050_YOUT_H , "MPU6050_YOUT_H"},
		{ MPU6050_REG_ADDR_MPU6050_YOUT_L , "MPU6050_YOUT_L"},
		{ MPU6050_REG_ADDR_MPU6050_ZOUT_H , "MPU6050_ZOUT_H"},
		{ MPU6050_REG_ADDR_MPU6050_ZOUT_L , "MPU6050_ZOUT_L"},
		{ MPU6050_REG_ADDR_TEMP_OUT_H , "TEMP_OUT_H"},
		{ MPU6050_REG_ADDR_TEMP_OUT_L , "TEMP_OUT_L"},
		{ MPU6050_REG_ADDR_GYRO_XOUT_H , "GYRO_XOUT_H"},
		{ MPU6050_REG_ADDR_GYRO_XOUT_L , "GYRO_XOUT_L"},
		{ MPU6050_REG_ADDR_GYRO_YOUT_H , "GYRO_YOUT_H"},
		{ MPU6050_REG_ADDR_GYRO_YOUT_L , "GYRO_YOUT_L"},
		{ MPU6050_REG_ADDR_GYRO_ZOUT_H , "GYRO_ZOUT_H"},
		{ MPU6050_REG_ADDR_GYRO_ZOUT_L , "GYRO_ZOUT_L"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_00 , "EXT_SENS_DATA_00"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_01 , "EXT_SENS_DATA_01"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_02 , "EXT_SENS_DATA_02"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_03 , "EXT_SENS_DATA_03"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_04 , "EXT_SENS_DATA_04"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_05 , "EXT_SENS_DATA_05"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_06 , "EXT_SENS_DATA_06"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_07 , "EXT_SENS_DATA_07"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_08 , "EXT_SENS_DATA_08"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_09 , "EXT_SENS_DATA_09"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_10 , "EXT_SENS_DATA_10"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_11 , "EXT_SENS_DATA_11"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_12 , "EXT_SENS_DATA_12"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_13 , "EXT_SENS_DATA_13"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_14 , "EXT_SENS_DATA_14"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_15 , "EXT_SENS_DATA_15"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_16 , "EXT_SENS_DATA_16"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_17 , "EXT_SENS_DATA_17"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_18 , "EXT_SENS_DATA_18"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_19 , "EXT_SENS_DATA_19"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_20 , "EXT_SENS_DATA_20"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_21 , "EXT_SENS_DATA_21"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_22 , "EXT_SENS_DATA_22"},
		{ MPU6050_REG_ADDR_EXT_SENS_DATA_23 , "EXT_SENS_DATA_23"},
		{ MPU6050_REG_ADDR_I2C_SLV0_DO , "I2C_SLV0_DO"},
		{ MPU6050_REG_ADDR_I2C_SLV1_DO , "I2C_SLV1_DO"},
		{ MPU6050_REG_ADDR_I2C_SLV2_DO , "I2C_SLV2_DO"},
		{ MPU6050_REG_ADDR_I2C_SLV3_DO , "I2C_SLV3_DO"},
		{ MPU6050_REG_ADDR_USER_CTRL , "USER_CTRL"},
		{ MPU6050_REG_ADDR_PWR_MGMT_1 , "PWR_MGMT_1"},
		{ MPU6050_REG_ADDR_PWR_MGMT_2 , "PWR_MGMT_2"},
		{ MPU6050_REG_ADDR_FIFO_COUNTH , "FIFO_COUNTH"},
		{ MPU6050_REG_ADDR_FIFO_COUNTL , "FIFO_COUNTL"},
		{ MPU6050_REG_ADDR_FIFO_R_W , "FIFO_R_W"},
		{ MPU6050_REG_ADDR_WHO_AM_I , "WHO_AM_I"}
};

char* get_reg_name(u8 addr)
{
	for(int i = 0; i < 256; i++)
	{
		if(reg_name_list[i].addr == addr)
		{
			return reg_name_list[i].name;
		}
	}

	return "Bad Reg Addr";
}

HAL_StatusTypeDef MPU6050_read_reg(Mpu_6050_handle_s* handle, u8 address, u8* pData)
{
	HAL_StatusTypeDef status = HAL_ERROR;

	status = HAL_I2C_Master_Transmit(handle->i2c_handle, (handle->i2c_address << 1) | 0x00, &address, 1, MPU6050_TIMEOUT);
	status |= HAL_I2C_Master_Receive(handle->i2c_handle, (handle->i2c_address << 1) | 0x01, pData, 1, MPU6050_TIMEOUT);

#ifdef DEBUG_MPU6050_REGISTER_READ_SUCCESS
	if(status == HAL_OK)
	{
		uart_println("Read %02x from %s", *pData, get_reg_name(address));
	}
#endif

#ifdef DEBUG_MPU6050_REGISTER_READ_FAIL
	if(status != HAL_OK)
	{
		uart_println("Failed to read %s, %x", get_reg_name(address),address);
	}
#endif



	return status;
}

HAL_StatusTypeDef MPU6050_write_reg(Mpu_6050_handle_s* handle, u8 address, u8 data)
{
	HAL_StatusTypeDef status = HAL_ERROR;

	u8 buf[2];
	buf[0] = address;
	buf[1] = data;
	status = HAL_I2C_Master_Transmit(handle->i2c_handle, (handle->i2c_address << 1) | 0x00, buf, 2, MPU6050_TIMEOUT);

#ifdef DEBUG_MPU6050_REGISTER_WRITE_SUCCESS
	if(status == HAL_OK)
		{
			uart_println("Wrote %02x to %s", data, get_reg_name(address));
		}
#endif

#ifdef DEBUG_MPU6050_REGISTER_WRITE_FAIL
	if(status != HAL_OK)
		{
			uart_println("Failed to write %s, %x", get_reg_name(address),address);
		}
#endif

	return status;
}

u8 MPU6050_calculate_fifo_read_size(u8 fifo_config)
{
	u8 size = 0;
	if(MPU_6050_TEMP_FIFO_EN_MASK & fifo_config)
	{
		size += 2;
	}

	if(MPU_6050_XG_FIFO_EN_MASK & fifo_config)
	{
		size += 2;
	}

	if(MPU_6050_YG_FIFO_EN_MASK & fifo_config)
	{
		size += 2;
	}

	if(MPU_6050_ZG_FIFO_EN_MASK & fifo_config)
	{
		size += 2;
	}

	if(MPU_6050_ACCEL_FIFO_EN_MASK & fifo_config)
	{
		size += 6;
	}

	return size;
}

//todo : make inputs to this just config enables, and the frequency instead of knowing what reg values are wanted
HAL_StatusTypeDef MPU6050_init(Mpu_6050_handle_s* handle, I2C_HandleTypeDef* i2c_handle, u8 i2c_address, u8 sample_rate_divider, u8 int_config, u8 user_config, u8 config_reg)
{
	HAL_StatusTypeDef status = HAL_ERROR;


	if(i2c_handle != 0 && i2c_address != 0)
	{
		handle->i2c_handle = i2c_handle;
		handle->i2c_address = i2c_address;
		status = HAL_OK;
	}
	else
	{
		status = HAL_ERROR;
	}

	if(status == HAL_OK)
	{
		u8 address = MPU6050_REG_ADDR_WHO_AM_I;
		u8 data = 0;

		status = MPU6050_read_reg(handle, address, &data);

		if(status == HAL_OK)
		{
			if(data == WHO_AM_I_VALUE)
			{
				status = HAL_OK;
			}
			else
			{
				status = HAL_ERROR;
			}
		}
	}

	if(status == HAL_OK)
	{
		u8 data = 0;

		//set data int enable
		status = MPU6050_write_reg(handle, MPU6050_REG_ADDR_INT_ENABLE, INT_ENABLE_MASK );

		//clear sleep
		status |= MPU6050_write_reg(handle, MPU6050_REG_ADDR_PWR_MGMT_1, 0x00);

		//set sample rate divider
		status |= MPU6050_write_reg(handle, MPU6050_REG_ADDR_SMPLRT_DIV, sample_rate_divider);

		//set gyro config
		//status |= MPU6050_write_reg(handle, MPU6050_REG_ADDR_SMPLRT_DIV, sample_rate_divider);

		//set int config
		//todo make inconfig several values / masks

		
		//u8 int_config = 0x30;
		status |= MPU6050_write_reg(handle, MPU6050_REG_ADDR_INT_PIN_CFG, int_config);

		//u8 user_config = 0x40;
		status |= MPU6050_write_reg(handle, MPU6050_REG_ADDR_USER_CTRL, user_config);

		status |= MPU6050_write_reg(handle, MPU6050_REG_ADDR_CONFIG, config_reg);

		u8 fifo_config = MPU_6050_FIFO_CONFIG_USER_DEFINED;
		handle->fifo_config = fifo_config;
		g_fifo_num_samples = MPU6050_FIFO_SIZE / sizeof(Mpu_6050_data_s);

		status |= MPU6050_write_reg(handle, MPU6050_REG_ADDR_FIFO_EN, fifo_config);
	}

	#ifdef MPU6050_PRINT_LEVEL_INFO

		u8 dlpf_config = config_reg & DLPF_CFG_MASK;
		u32 gyro_base_freq = ( dlpf_config == 0 || dlpf_config == 0x7 )  ? 8000 : 1000;
		u32 accel_base_freq = 1000;

		u32 gyro_sample_rate = gyro_base_freq / sample_rate_divider;
		u32 accel_sample_rate = accel_base_freq / sample_rate_divider;

		uart_println("Accel Sample Rate : %d Hz, Bandwith : %d Hz", accel_sample_rate, accel_dplf_to_bandwidth_lut[dlpf_config]);
		uart_println("Gyro Sample Rate : %d Hz, Bandwidth : %d Hz", gyro_sample_rate, gyro_dplf_to_bandwidth_lut[dlpf_config]);


		
	#endif

	if(status == HAL_OK)
	{
		uart_println("MPU6050 Init Success");
	}
	else
	{
		uart_println("MPU6050 Init Failed");
	}

	return status;
}



HAL_StatusTypeDef MPU6050_get_fifo_data(Mpu_6050_handle_s* handle, u32* num_samples)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	u8 fifo_count[2] = {0};
	u16 fifo_count_full = 0;
	u8 address = MPU6050_REG_ADDR_FIFO_R_W;

	status = MPU6050_read_reg(handle, MPU6050_REG_ADDR_FIFO_COUNTH, &fifo_count[0] );
	status |= MPU6050_read_reg(handle, MPU6050_REG_ADDR_FIFO_COUNTL, &fifo_count[1] );
	fifo_count_full = (u16)((fifo_count[0]<<8) | fifo_count[1]);

	if(status == HAL_OK && fifo_count_full > 0)
	{
		status = HAL_I2C_Master_Transmit(handle->i2c_handle, (handle->i2c_address << 1) | 0x00, &address, 1, MPU6050_TIMEOUT);
		status |= HAL_I2C_Master_Receive(handle->i2c_handle, (handle->i2c_address << 1) | 0x01, g_fifo_read_buffer, fifo_count_full, MPU6050_TIMEOUT);

		if(status == HAL_OK)
		{
			//put data into struct
			*num_samples = fifo_count_full / sizeof(Mpu_6050_data_s);
		}
	}
	else if(status == HAL_OK && fifo_count_full == 0)
	{
		status = HAL_OK;
	}

	return status;
}

void MPU6050_parse_fifo_data(u8* fifoData, Mpu_6050_data_s* pData)
{
	u32 index = 0;
#if MPU_6050_TEMP_EN
	pData->temp_data = (s16)((fifoData[index]<<8) |  fifoData[index+1]);
	index += 2;
#endif

#if MPU_6050_ACCEL_EN
	pData->x_accel_data = (s16)((fifoData[index]<<8) |  fifoData[index+1]);
	index += 2;
	pData->y_accel_data = (s16)((fifoData[index]<<8) |  fifoData[index+1]);
	index += 2;
	pData->z_accel_data = (s16)((fifoData[index]<<8) |  fifoData[index+1]);
	index += 2;
#endif

#if MPU_6050_XG_EN
	pData->x_gyro_data = (s16)((fifoData[index]<<8) |  fifoData[index+1]);
	index += 2;
#endif

#if MPU_6050_YG_EN
	pData->y_gyro_data = (s16)((fifoData[index]<<8) |  fifoData[index+1]);
	index += 2;
#endif

#if MPU_6050_ZG_EN
	pData->z_gyro_data = (s16)((fifoData[index]<<8) |  fifoData[index+1]);
	index += 2;
#endif

}

void MPU6050_utility_data_buffer_to_struct(s16* data, Mpu_6050_data_s* pData)
{
	u32 index = 0;
#if MPU_6050_TEMP_EN
	pData->temp_data = (s16)((fifoData[index]<<8) |  fifoData[index+1]);
	index += 1;
#endif

#if MPU_6050_ACCEL_EN
	pData->x_accel_data = data[index];
	index += 1;
	pData->y_accel_data = data[index];
	index += 1;
	pData->z_accel_data = data[index];
	index += 1;
#endif

#if MPU_6050_XG_EN
	pData->x_gyro_data = data[index];
	index += 1;
#endif

#if MPU_6050_YG_EN
	pData->y_gyro_data = data[index];
	index += 1;
#endif

#if MPU_6050_ZG_EN
	pData->z_gyro_data = data[index];
	index += 1;
#endif

}

HAL_StatusTypeDef MPU6050_get_raw_data(Mpu_6050_handle_s* handle, u8* buf)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	u8 num_samples = 0;

	status = MPU6050_get_fifo_data(handle, &num_samples);

	if(status == HAL_OK)
	{
		for(int i = 0; i<num_samples; i++)
		{
			Mpu_6050_data_s tmp_data;
			MPU6050_parse_fifo_data(&g_fifo_read_buffer[i], &tmp_data);
			//ring_buffer_write_element(tmp_data);

			//fixme
			//MPU6050_print_data(&tmp_data);
			memcpy(&buf[i*sizeof(Mpu_6050_data_s)], &tmp_data, sizeof(Mpu_6050_data_s));
		}
	}

	return status;
}

//read the accelerometer data registers directly, using register fallthrough
HAL_StatusTypeDef MPU6050_get_accel_sample(Mpu_6050_handle_s* handle, s16* buf)
{
	HAL_StatusTypeDef status = HAL_ERROR;

	//start at xout_h, read 6 registers from there using fallthrough
	u8 start_address = MPU6050_REG_ADDR_MPU6050_XOUT_H;
	u8 num_reg_to_read = 6;

	u8 tmp_buf[num_reg_to_read];

	status = HAL_I2C_Master_Transmit(handle->i2c_handle, (handle->i2c_address << 1) | 0x00, &start_address, 1, MPU6050_TIMEOUT);
	status |= HAL_I2C_Master_Receive(handle->i2c_handle, (handle->i2c_address << 1) | 0x01, tmp_buf, num_reg_to_read, MPU6050_TIMEOUT);

	//xout
	buf[0] = (s16)((tmp_buf[0]<<8) |  tmp_buf[1]);
	//yout
	buf[1] = (s16)((tmp_buf[2]<<8) |  tmp_buf[3]);
	//zout
	buf[2] = (s16)((tmp_buf[4]<<8) |  tmp_buf[5]);

	return status;
}

void MPU6050_print_data(Mpu_6050_data_s* pData)
{

#if MPU_6050_TEMP_EN
	uart_println("%d", pData->temp_data);
#endif

#if MPU_6050_ACCEL_EN
	MPU6050_PRINT_FUNCTION( MPU6050_PRINT_FMT, (MPU6050_PRINT_DATA_TYPE)pData->x_accel_data);
	MPU6050_PRINT_FUNCTION( MPU6050_PRINT_FMT, (MPU6050_PRINT_DATA_TYPE)pData->y_accel_data);
	MPU6050_PRINT_FUNCTION( MPU6050_PRINT_FMT, (MPU6050_PRINT_DATA_TYPE)pData->z_accel_data);
#endif

#if MPU_6050_XG_EN
	MPU6050_PRINT_FUNCTION( MPU6050_PRINT_FMT, (MPU6050_PRINT_DATA_TYPE)pData->x_gyro_data);
#endif

#if MPU_6050_YG_EN
	MPU6050_PRINT_FUNCTION( MPU6050_PRINT_FMT, (MPU6050_PRINT_DATA_TYPE)pData->y_gyro_data);
#endif

#if MPU_6050_ZG_EN
	MPU6050_PRINT_FUNCTION( MPU6050_PRINT_FMT, (MPU6050_PRINT_DATA_TYPE)pData->z_gyro_data);
#endif

	MPU6050_PRINT_LAST_LINE;
}


//todo pretty sure I can use mask division
void MPU6050_condition_data(Mpu_6050_data_s* pData)
{

#if MPU_6050_TEMP_EN
	//todo not entirely sure if temp data is from a 16 bit adc
	pData->temp_data = (u16)(pData->temp_data * MPU6050_VOLTAGE / MPU6050_ADC_DEPTH);
#endif

#if MPU_6050_ACCEL_EN
	pData->x_accel_data = (u16)(pData->x_accel_data * MPU6050_VOLTAGE / MPU6050_ADC_DEPTH);
	pData->y_accel_data = (u16)(pData->y_accel_data * MPU6050_VOLTAGE / MPU6050_ADC_DEPTH);
	pData->z_accel_data = (u16)(pData->z_accel_data * MPU6050_VOLTAGE / MPU6050_ADC_DEPTH);
#endif

#if MPU_6050_XG_EN
	pData->x_gyro_data = (u16)(pData->x_gyro_data * MPU6050_VOLTAGE / MPU6050_ADC_DEPTH);
#endif

#if MPU_6050_YG_EN
	pData->y_gyro_data = (u16)(pData->y_gyro_data * MPU6050_VOLTAGE / MPU6050_ADC_DEPTH);
#endif

#if MPU_6050_ZG_EN
	pData->z_gyro_data = (u16)(pData->z_gyro_data * MPU6050_VOLTAGE / MPU6050_ADC_DEPTH);
#endif

}

void MPU6050_read_fifo_data(u32 index, Mpu_6050_data_s* data)
{
	MPU6050_parse_fifo_data(&g_fifo_read_buffer[index*sizeof(Mpu_6050_data_s)], data);
}

HAL_StatusTypeDef MPU6050_reset_fifo(Mpu_6050_handle_s* handle)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	u8 user_control;
	status = MPU6050_read_reg(handle, MPU6050_REG_ADDR_USER_CTRL, &user_control);
	user_control |= MPU6050_FIFO_RESET_BIT_MASK;
	status |= MPU6050_write_reg(handle, MPU6050_REG_ADDR_USER_CTRL, user_control);

	return status;
}

