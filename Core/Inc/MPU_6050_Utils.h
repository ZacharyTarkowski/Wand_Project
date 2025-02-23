/*
 * MPU6050erometer_Utils.h
 *
 *  Created on: Aug 21, 2024
 *      Author: zacht
 */

#ifndef SRC_MPU6050EROMETER_UTILS_H_
#define SRC_MPU6050EROMETER_UTILS_H_

#include "General_Utils.h"

#define MPU6050_REG_ADDR_SELF_TEST_X 0x0D
#define MPU6050_REG_ADDR_SELF_TEST_Y 0x0E
#define MPU6050_REG_ADDR_SELF_TEST_Z 0x0F
#define MPU6050_REG_ADDR_SELF_TEST_A 0x10
#define MPU6050_REG_ADDR_SMPLRT_DIV 0x19
#define MPU6050_REG_ADDR_CONFIG 0x1A
#define MPU6050_REG_ADDR_GYRO_CONFIG 0x1B
#define MPU6050_REG_ADDR_MPU6050_CONFIG 0x1C
#define MPU6050_REG_ADDR_FIFO_EN 0x23
#define MPU6050_REG_ADDR_I2C_MST_CTRL 0x24
#define MPU6050_REG_ADDR_I2C_SLV0_ADDR 0x25
#define MPU6050_REG_ADDR_I2C_SLV0_REG 0x26
#define MPU6050_REG_ADDR_I2C_SLV0_CTRL 0x27
#define MPU6050_REG_ADDR_I2C_SLV1_REG 0x29
#define MPU6050_REG_ADDR_I2C_SLV1_CTRL 0x2A
#define MPU6050_REG_ADDR_I2C_SLV2_ADDR 0x2B
#define MPU6050_REG_ADDR_I2C_SLV2_REG 0x2C
#define MPU6050_REG_ADDR_I2C_SLV2_CTRL 0x2D
#define MPU6050_REG_ADDR_I2C_SLV4_REG 0x32
#define MPU6050_REG_ADDR_I2C_SLV4_DO 0x33
#define MPU6050_REG_ADDR_I2C_SLV4_CTRL 0x34
#define MPU6050_REG_ADDR_I2C_SLV4_DI 0x35
#define MPU6050_REG_ADDR_I2C_MST_STATUS 0x36
#define MPU6050_REG_ADDR_INT_PIN_CFG 0x37
#define MPU6050_REG_ADDR_INT_ENABLE 0x38
#define MPU6050_REG_ADDR_INT_STATUS 0x3A
#define MPU6050_REG_ADDR_MPU6050_XOUT_H 0x3B
#define MPU6050_REG_ADDR_MPU6050_XOUT_L 0x3C
#define MPU6050_REG_ADDR_MPU6050_YOUT_H 0x3D
#define MPU6050_REG_ADDR_MPU6050_YOUT_L 0x3E
#define MPU6050_REG_ADDR_MPU6050_ZOUT_H 0x3F
#define MPU6050_REG_ADDR_MPU6050_ZOUT_L 0x40
#define MPU6050_REG_ADDR_TEMP_OUT_H 0x41
#define MPU6050_REG_ADDR_TEMP_OUT_L 0x42
#define MPU6050_REG_ADDR_GYRO_XOUT_H 0x43
#define MPU6050_REG_ADDR_GYRO_XOUT_L 0x44
#define MPU6050_REG_ADDR_GYRO_YOUT_H 0x45
#define MPU6050_REG_ADDR_GYRO_YOUT_L 0x46
#define MPU6050_REG_ADDR_GYRO_ZOUT_H 0x47
#define MPU6050_REG_ADDR_GYRO_ZOUT_L 0x48
#define MPU6050_REG_ADDR_EXT_SENS_DATA_00 0x49
#define MPU6050_REG_ADDR_EXT_SENS_DATA_01 0x4A
#define MPU6050_REG_ADDR_EXT_SENS_DATA_02 0x4B
#define MPU6050_REG_ADDR_EXT_SENS_DATA_03 0x4C
#define MPU6050_REG_ADDR_EXT_SENS_DATA_04 0x4D
#define MPU6050_REG_ADDR_EXT_SENS_DATA_05 0x4E
#define MPU6050_REG_ADDR_EXT_SENS_DATA_06 0x4F
#define MPU6050_REG_ADDR_EXT_SENS_DATA_07 0x50
#define MPU6050_REG_ADDR_EXT_SENS_DATA_08 0x51
#define MPU6050_REG_ADDR_EXT_SENS_DATA_09 0x52
#define MPU6050_REG_ADDR_EXT_SENS_DATA_10 0x53
#define MPU6050_REG_ADDR_EXT_SENS_DATA_11 0x54
#define MPU6050_REG_ADDR_EXT_SENS_DATA_12 0x55
#define MPU6050_REG_ADDR_EXT_SENS_DATA_13 0x56
#define MPU6050_REG_ADDR_EXT_SENS_DATA_14 0x57
#define MPU6050_REG_ADDR_EXT_SENS_DATA_15 0x58
#define MPU6050_REG_ADDR_EXT_SENS_DATA_16 0x59
#define MPU6050_REG_ADDR_EXT_SENS_DATA_17 0x5A
#define MPU6050_REG_ADDR_EXT_SENS_DATA_18 0x5B
#define MPU6050_REG_ADDR_EXT_SENS_DATA_19 0x5C
#define MPU6050_REG_ADDR_EXT_SENS_DATA_20 0x5D
#define MPU6050_REG_ADDR_EXT_SENS_DATA_21 0x5E
#define MPU6050_REG_ADDR_EXT_SENS_DATA_22 0x5F
#define MPU6050_REG_ADDR_EXT_SENS_DATA_23 0x60
#define MPU6050_REG_ADDR_I2C_SLV0_DO 0x63
#define MPU6050_REG_ADDR_I2C_SLV1_DO 0x64
#define MPU6050_REG_ADDR_I2C_SLV2_DO 0x65
#define MPU6050_REG_ADDR_I2C_SLV3_DO 0x66
#define MPU6050_REG_ADDR_USER_CTRL 0x6A
#define MPU6050_REG_ADDR_PWR_MGMT_1 0x6B
#define MPU6050_REG_ADDR_PWR_MGMT_2 0x6C
#define MPU6050_REG_ADDR_FIFO_COUNTH 0x72
#define MPU6050_REG_ADDR_FIFO_COUNTL 0x73
#define MPU6050_REG_ADDR_FIFO_R_W 0x74
#define MPU6050_REG_ADDR_WHO_AM_I 0x75

#define WHO_AM_I_VALUE 0x68
#define INT_ENABLE_MASK 0x01
#define MPU6050_FIFO_RESET_BIT_MASK (1<<2);

#define MPU6050_FIFO_SIZE 1024

#define MPU6050_TIMEOUT 500 //HAL_MAX_DELAY-1 //-1 for not infinite timeout in i2c

#define MPU6050_SIZE_SAMPLE_BUFFER 100

#define MPU6050_VOLTAGE 3.3
#define MPU6050_ADC_DEPTH 0xFFFF

#define MPU_6050_TEMP_FIFO_EN_MASK (1<<7)
#define MPU_6050_XG_FIFO_EN_MASK (1<<6)
#define MPU_6050_YG_FIFO_EN_MASK (1<<5)
#define MPU_6050_ZG_FIFO_EN_MASK (1<<4)
#define MPU_6050_ACCEL_FIFO_EN_MASK (1<<3)

#define DLPF_CFG_MASK (0x7)

//user defines
//external data not implemented
#define MPU_6050_TEMP_EN  0
#define MPU_6050_XG_EN	  1
#define MPU_6050_YG_EN    1
#define MPU_6050_ZG_EN    1
#define MPU_6050_ACCEL_EN 1

#define MPU_6050_NUM_DIMS (MPU_6050_TEMP_EN + MPU_6050_XG_EN + MPU_6050_YG_EN + MPU_6050_ZG_EN + (MPU_6050_ACCEL_EN * 3 ))

#define MPU6050_PRINT_CSV 1
#define MPU6050_PRINT_LABELLED 0
#define MPU6050_PRINT_EACH_LINE 0

#define MPU_6050_FIFO_CONFIG_USER_DEFINED \
	((MPU_6050_TEMP_EN) ? MPU_6050_TEMP_FIFO_EN_MASK : 0) | \
	((MPU_6050_XG_EN) ? MPU_6050_XG_FIFO_EN_MASK : 0) | \
	((MPU_6050_YG_EN) ? MPU_6050_YG_FIFO_EN_MASK : 0) | \
	((MPU_6050_ZG_EN) ? MPU_6050_ZG_FIFO_EN_MASK : 0) | \
	((MPU_6050_ACCEL_EN) ? MPU_6050_ACCEL_FIFO_EN_MASK : 0)

static const u16 accel_dplf_to_bandwidth_lut[] = {
	260,
	184,
	94,
	44,
	21,
	10,
	5
};

static const u16 gyro_dplf_to_bandwidth_lut[] = {
	256,
	188,
	98,
	42,
	20,
	10,
	5
};


typedef struct reg
{
	u8 addr;
	char* name;
}reg;

typedef struct Mpu_6050_data_s
{
#if MPU_6050_ACCEL_EN
	s16 x_accel_data;
	s16 y_accel_data;
	s16 z_accel_data;
#endif

#if MPU_6050_XG_EN
	s16 x_gyro_data;
#endif

#if MPU_6050_YG_EN
	s16 y_gyro_data;
#endif

#if MPU_6050_ZG_EN
	s16 z_gyro_data;
#endif

#if MPU_6050_TEMP_EN
	s16 temp_data;
#endif
}Mpu_6050_data_s;


typedef struct MPU6050_accelerometer_handle
{
	I2C_HandleTypeDef* i2c_handle;
	u8 i2c_address;
	u8 fifo_config;

}Mpu_6050_handle_s;


//fixme rename a majority of these lol
typedef enum 
{
    X_ACCEL_IDX = 0, 
    Y_ACCEL_IDX = 1,
    Z_ACCEL_IDX = 2,
    X_GYRO_IDX  = 3,
    Y_GYRO_IDX  = 4,
    Z_GYRO_IDX  = 5
}MPU6050_dimension;



#if MPU6050_PRINT_CSV

#define MPU6050_PRINT_FUNCTION uart_printf
#define MPU6050_PRINT_FLOAT_EN 0

#if MPU6050_PRINT_FLOAT_EN
#define MPU6050_PRINT_FMT "%f,"
#define MPU6050_PRINT_DATA_TYPE float
#else
#define MPU6050_PRINT_FMT "%d,"
#define MPU6050_PRINT_DATA_TYPE s16
#endif

#define MPU6050_PRINT_LAST_LINE uart_println("")

#elif MPU6050_PRINT_LABELLED

//todo

#else // MPU6050_PRINT_EACH_LINE

#define MPU6050_PRINT_FUNCTION uart_println
#define MPU6050_PRINT_FMT "%d"
#define MPU6050_PRINT_LAST_LINE

#endif

HAL_StatusTypeDef MPU6050_read_reg(Mpu_6050_handle_s* handle, u8 address, u8* pData);
HAL_StatusTypeDef MPU6050_write_reg(Mpu_6050_handle_s* handle, u8 address, u8 data);

HAL_StatusTypeDef MPU6050_init(Mpu_6050_handle_s* handle, I2C_HandleTypeDef* i2c_handle, u8 i2c_address, u8 sample_rate_divider, u8 int_config, u8 user_config, u8 config_reg);

HAL_StatusTypeDef MPU6050_get_raw_data(Mpu_6050_handle_s* handle, u8* buf);

HAL_StatusTypeDef MPU6050_clean_data(Mpu_6050_data_s* data);

void MPU6050_condition_data(Mpu_6050_data_s* pData);
void MPU6050_print_data(Mpu_6050_data_s* pData);

void MPU6050_read_fifo_data(u32 index, Mpu_6050_data_s* data);
HAL_StatusTypeDef MPU6050_get_fifo_data(Mpu_6050_handle_s* handle, u8* num_samples);
HAL_StatusTypeDef MPU6050_get_accel_sample(Mpu_6050_handle_s* handle, s16* buf);

HAL_StatusTypeDef MPU6050_reset_fifo_(Mpu_6050_handle_s* handle);
void MPU6050_utility_data_buffer_to_struct(s16* data, Mpu_6050_data_s* pData);


#endif /* SRC_MPU6050EROMETER_UTILS_H_ */
