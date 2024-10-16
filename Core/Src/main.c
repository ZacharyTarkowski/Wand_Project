/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "General_Utils.h"
#include "Ring_Buffer.h"
#include "MPU_6050_Utils.h"

#include "Dynamic_Time_Warping.h"
#include "LED_Utils.h"


Mpu_6050_handle_s MPU6050_handle;
Mpu_6050_handle_s* pMPU6050 = &MPU6050_handle;
u8 data_ready_flag = 0;
u8 capture_flag = 0;
u32 capture_flag_valid_time;
u8 timer_flag = 0;

ring_buffer_s ring_buffer_1;
ring_buffer_s ring_buffer_2;
ring_buffer_s ring_buffer_3;
ring_buffer_s circle_spell_ring_buffer;
ring_buffer_s line_spell_ring_buffer;

ring_buffer_s orientation_line_spell_ring_buffer;
ring_buffer_s orientation_circle_spell_ring_buffer;


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum  {
	IDLE,
	CAPTURE_1,
	CAPTURE_2,
	PRINT_AND_COMPARE,
	COMPARE_STATIC,
	HARD_RESET,
	MAX_WAND_STATE
}WAND_STATE;

const char* wand_state_name_lut[MAX_WAND_STATE] = {
	"IDLE",
	"CAPTURE_1",
	"CAPTURE_2",
	"PRINT_AND_COMPARE",
	"COMPARE_STATIC",
	"HARD_RESET",
	"MAX_WAND_STATE"
};




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	u32 capture_flag_current_time;

	switch(GPIO_Pin)
	{
	case GPIO_PIN_9:
		//uart_println("Data Ready Interrupt Triggered");
		data_ready_flag = 1;
		break;
	case GPIO_PIN_10:
		//uart_println("Button Press Interrupt Triggered");
		capture_flag_current_time = HAL_GetTick();
		if(capture_flag_current_time > capture_flag_valid_time)
		{
			capture_flag = !capture_flag;
			capture_flag_valid_time = capture_flag_current_time + 250;
		}
		break;
	default:
		//uart_println("Unhandled interrupt triggered %d",GPIO_Pin);
		break;
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim3.Instance)
	{
		timer_flag = 1;
	}
}


s16 raw_circle_spell_data[] = {
360,16412,2364,-487,98,-298,
-262,16128,2440,-479,-128,-321,
-308,16162,2498,-362,-637,-397,
472,15708,2434,-158,-1135,-156,
1170,16260,2092,254,-1370,-241,
1836,15626,1944,488,-945,-97,
1362,15754,1590,901,-298,-437,
1218,17076,834,1192,285,-1046,
1100,17398,550,802,938,-1069,
384,16900,424,178,1428,-805,
-100,17032,430,-209,1759,-744,
-782,17458,560,-682,1782,-847,
-1490,17036,926,-1566,1453,-840,
-2130,16634,1666,-2205,675,-433,
-1228,15818,2122,-2170,-530,-49,
22,15388,2614,-1427,-1799,168,
1046,15812,2434,-896,-1420,52,
646,16562,2526,-506,-494,-344,
56,15996,2702,-502,49,-256,
-100,16418,2592,-291,95,-493
};

s16 raw_orientation_circle_spell_data[] = {
-214,40,68,-234,693,25,
-482,-118,190,-336,531,69,
-692,-298,236,-251,271,-68,
-348,-430,200,-32,-272,457,
-346,-618,236,170,-480,421,
472,-128,-236,648,-741,455,
892,-222,-486,826,-479,598,
664,-478,-638,872,-98,435,
802,-52,-1140,1129,302,368,
970,338,-1504,1068,690,-226,
896,234,-1666,688,1142,-220,
300,162,-1720,429,1602,125,
198,194,-1830,256,1658,33,
-224,502,-1804,-111,1555,-553,
-568,634,-1610,-799,1516,-394,
-1218,280,-1110,-1488,1077,-140,
-1176,-226,-784,-1843,320,166,
-1348,-1290,44,-1670,-653,475,
1002,-1658,136,-632,-1707,1020,
2000,116,-428,254,-1088,478,
986,60,-440,-73,373,373
};

s16 raw_line_spell_data[] = {
-1134,16430,1234,-354,-230,-311,
-730,15990,782,-879,92,-84,
-404,14662,1516,-735,210,1231,
-100,15120,1272,295,93,374,
-18,16544,688,1321,-240,1282,
670,15524,-292,1331,-274,1224,
892,18910,-650,1722,-191,131,
1194,18442,-976,625,-290,-187,
844,15666,-870,-229,133,-9,
1194,17140,-928,445,206,57
};

s16 raw_orientation_line_spell_data[] = {
100,82,306,-601,-770,-504,
-50,-116,500,-650,-499,-539,
40,-1180,674,-680,-750,-31,
28,-532,732,-309,-566,200,
432,138,488,189,-618,268,
592,-72,574,105,-909,173,
676,922,402,302,-843,84,
958,984,262,171,-847,10,
1058,646,308,100,-816,-206,
970,332,254,-76,-659,-622,
934,196,192,-78,-562,-730
};




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_StatusTypeDef status = HAL_ERROR;
	Mpu_6050_data_s data;
	Mpu_6050_data_s* pData;

	//should make a function to calc this automagically
	// 8000 / (x+1) = desired_sample_rate

	
	u8 sample_rate_divider = 0x80;//0x14;//0x80;
	status = MPU6050_init(pMPU6050, &hi2c1, 0x68, sample_rate_divider, 0x30, 0x40, 0x06 );
	
	status |= ring_buffer_init(&ring_buffer_1, RING_BUFFER_MAX_SIZE);
	status |= ring_buffer_init(&ring_buffer_2, RING_BUFFER_MAX_SIZE);
	status |= ring_buffer_init(&ring_buffer_3, 10);
	status |= ring_buffer_init(&circle_spell_ring_buffer, sizeof(raw_circle_spell_data) / (sizeof(s16)*(MPU_6050_NUM_DIMS)) );
	status |= ring_buffer_init(&line_spell_ring_buffer, sizeof(raw_line_spell_data) / (sizeof(s16)*(MPU_6050_NUM_DIMS)) );
	status |= ring_buffer_init(&orientation_line_spell_ring_buffer, sizeof(raw_orientation_line_spell_data) / (sizeof(s16)*(MPU_6050_NUM_DIMS)) );
	status |= ring_buffer_init(&orientation_circle_spell_ring_buffer, sizeof(raw_orientation_circle_spell_data) / (sizeof(s16)*(MPU_6050_NUM_DIMS)) );
	if(status != HAL_OK)
	{
		// hi2c1.Instance->CR1 |= (1<<15);
		// RCC->APB1RSTR |= (1<<22); 
		// RCC->APB1RSTR &= ~(1<<22); 

		//MODIFY_REG(hi2c->Instance->CR1, (I2C_CR1_ENGC | I2C_CR1_NOSTRETCH), (hi2c->Init.GeneralCallMode | hi2c->Init.NoStretchMode));
		//status = MPU6050_init(pMPU6050, &hi2c1, 0x68, sample_rate_divider );
		return -1;
	}

	ring_buffer_MPU6050_parse_data_buffer(&circle_spell_ring_buffer,raw_circle_spell_data);
	ring_buffer_MPU6050_parse_data_buffer(&line_spell_ring_buffer,raw_line_spell_data);
	ring_buffer_MPU6050_parse_data_buffer(&orientation_line_spell_ring_buffer,raw_orientation_line_spell_data);
	ring_buffer_MPU6050_parse_data_buffer(&orientation_circle_spell_ring_buffer,raw_orientation_circle_spell_data);

	//ring_buffer_print_all_elements(&circle_spell_ring_buffer);
	

	if(dtw_init(SINGLE_MODE) != HAL_OK)
	{
		return -1;
	}
	

	u8 toggle = 0;
	u8 first_run = 1;
	u8 num_samples = 0;
	

	u32 loopnum = 0;
	capture_flag = 0;
	u8 toggle_buf = 0;
	timer_flag = 0;

	WAND_STATE state = IDLE;
	DTW_Result result;
	u8 MPU6050_hard_reset_flag;

	set_led_color(LED_COLOR_RED);

	//HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		switch(state)
		{
			//this might get annoying
			//nvm it literally never triggeres lol
			uart_println("State is : %s", wand_state_name_lut[state] );


			case IDLE:
			if(capture_flag)
			{
				first_run = 1;
				state = CAPTURE_1;
			}
			else if(timer_flag)
			{
				if(first_run)
				{
					//uart_println("Capturing %d",state);
					first_run = 0;
					ring_buffer_clear(&ring_buffer_3);
					status = MPU6050_reset_fifo_(pMPU6050);
					data_ready_flag = 0;
					//delay needed, otherwise fifo read fails because it got reset? might need to just clear the data ready flag cuz data isnt actually ready
					//HAL_Delay(500);
					if(status != HAL_OK)
					{
						uart_println("Failed to reset FIFO!");
						MPU6050_hard_reset_flag = 1;
					}
					loopnum = 0;
				}

				if(data_ready_flag && !MPU6050_hard_reset_flag)
				{
					data_ready_flag = 0;
					status = ring_buffer_MPU6050_read_and_store(pMPU6050, &ring_buffer_3 );
					if(status != HAL_OK)
					{
						uart_println("Read and store failed");
						MPU6050_hard_reset_flag = 1;
						
					}

					timer_flag = 0;
					
					first_run = 1;
					uart_println("Timer Triggered");

					

					ring_buffer_print_to_write_index(&ring_buffer_3);



				}
			}

			break;

			case CAPTURE_1:
			case CAPTURE_2:

			if(capture_flag)
			{
				if(first_run)
				{
					uart_println("Capturing %d",state);
					first_run = 0;
					status = MPU6050_reset_fifo_(pMPU6050);
					data_ready_flag = 0;
					//delay needed, otherwise fifo read fails because it got reset? might need to just clear the data ready flag cuz data isnt actually ready
					//HAL_Delay(500);
					if(status != HAL_OK)
					{
						uart_println("Failed to reset FIFO!");
						MPU6050_hard_reset_flag = 1;
					}
					loopnum = 0;
				}

				if(data_ready_flag && !MPU6050_hard_reset_flag)
				{
					toggle = 1;
					data_ready_flag = 0;
					//capture_flag = 0;
					//capture data
					//add to ring buffer
					status = ring_buffer_MPU6050_read_and_store(pMPU6050, ( state==CAPTURE_1 ? &ring_buffer_1 : &ring_buffer_2 ) );
					if(status != HAL_OK)
					{
						uart_println("Read and store failed %d",loopnum);
						MPU6050_hard_reset_flag = 1;
						
					}

				if(MPU6050_hard_reset_flag)
				{
					state = HARD_RESET;
				}


					loopnum++;
				}
			}
			
			if(!capture_flag)
			{
				if( !first_run )
				{
					uart_println("Done Capturing %d",state);

					state = COMPARE_STATIC; //this for comparing static spell
					//state = state == CAPTURE_1 ? CAPTURE_2 : PRINT_AND_COMPARE;//this for comparing two captures
					first_run = 1;
				}
			}
			break;

			case PRINT_AND_COMPARE:


				uart_println("Ring Buffer 1");
				ring_buffer_print_to_write_index(&ring_buffer_1);
				uart_println("Ring Buffer 2");
				ring_buffer_print_to_write_index(&ring_buffer_2);

				//added this in!!

				//DEBUG CODE FOR ORINTATION. DOESNT WORK BECAUSE NEED TO GO IDLE BETWEEN CAPTURES IN ORDER TO SENSE ORIENTATION
				// ring_buffer_MPU6050_apply_vector(&ring_buffer_1, &ring_buffer_3.buffer[0]);
				// ring_buffer_MPU6050_apply_vector(&ring_buffer_2, &ring_buffer_3.buffer[0]);

				// uart_println("Ring Buffer 1");
				// ring_buffer_print_to_write_index(&ring_buffer_1);
				// uart_println("Ring Buffer 2");
				// ring_buffer_print_to_write_index(&ring_buffer_2);
				// uart_println("Ring Buffer 3");
				// ring_buffer_print_all_elements(&ring_buffer_3);

				

				result = DTW_Distance(ring_buffer_1.buffer, ring_buffer_2.buffer,ring_buffer_1.write_index,ring_buffer_2.write_index);
				print_dtw_result(&result);

				u32 average_dtw = 0;
				average_dtw = (result.x_accel_result + result.y_accel_result + result.z_accel_result + result.x_gyro_result + result.y_gyro_result + result.z_gyro_result ) / ( MPU_6050_NUM_DIMS );
				uart_println("Avg DTW %d",average_dtw);

				//normalization?
				// result.x_accel_result = result.x_accel_result / average_dtw;
				// result.y_accel_result = result.y_accel_result / average_dtw;
				// result.z_accel_result = result.z_accel_result / average_dtw;
				// result.x_gyro_result  = result.x_gyro_result  / average_dtw;
				// result.y_gyro_result  = result.y_gyro_result  / average_dtw;
				// result.z_gyro_result  = result.z_gyro_result  / average_dtw;
				
				// uart_println("Normalized DTW Maybe");
				// print_dtw_result(&result);

				if( result.x_accel_result < 12500 && result.y_accel_result < 12500 && result.z_accel_result < 12500 )
				{
					set_led_color(LED_COLOR_GREEN);
				}
				else
				{
					set_led_color(LED_COLOR_RED);
				}
				


				ring_buffer_clear(&ring_buffer_1);
				ring_buffer_clear(&ring_buffer_2);
				state = IDLE;


			break;

			case COMPARE_STATIC:
				uart_println("Ring Buffer 1");
				ring_buffer_print_to_write_index(&ring_buffer_1);
				//uart_println("Static Spell : Circle");
				//ring_buffer_print_all_elements(&circle_spell_ring_buffer);

				uart_println("DEBUG ORIENTATION CODE");
				ring_buffer_MPU6050_apply_vector(&ring_buffer_1, &ring_buffer_3.buffer[0]);
				
				uart_println("Ring Buffer 1");
				ring_buffer_print_to_write_index(&ring_buffer_1);
				uart_println("Ring Buffer 3");
				ring_buffer_print_to_write_index(&ring_buffer_3);

				
				DTW_Result result_circle = DTW_Distance(ring_buffer_1.buffer, circle_spell_ring_buffer.buffer,ring_buffer_1.write_index,circle_spell_ring_buffer.size);
				uart_println("Circle");
				//print_dtw_result(&result_circle);

				DTW_Result result_line = DTW_Distance(ring_buffer_1.buffer, line_spell_ring_buffer.buffer,ring_buffer_1.write_index, line_spell_ring_buffer.size);
				uart_println("Line");
				//print_dtw_result(&result_line);

				DTW_Result result_orientation_circle = DTW_Distance(ring_buffer_1.buffer, orientation_circle_spell_ring_buffer.buffer,ring_buffer_1.write_index,orientation_circle_spell_ring_buffer.size);
				uart_println("Orientation Circle");
				print_dtw_result(&result_orientation_circle);

				DTW_Result result_orientation_line = DTW_Distance(ring_buffer_1.buffer, orientation_line_spell_ring_buffer.buffer, ring_buffer_1.write_index, orientation_line_spell_ring_buffer.size);
				uart_println("Orientation Line");
				print_dtw_result(&result_orientation_line);


				//once orientation is locked down, realisitcally i only care bout 2 dimensions for simple things like circles
				//in the actual spell framework would be cool for static spells to tell it the dimensions it cares about, and only compare against those. can also work in fudge factors for different effects
				if( result_circle.x_accel_result < 12500 && result_circle.y_accel_result < 12500 && result_circle.z_accel_result < 12500 )
				{
					set_led_color(LED_COLOR_GREEN);
				}
				else if ( result_orientation_circle.x_accel_result < 12500 && result_orientation_circle.y_accel_result < 12500 && result_orientation_circle.z_accel_result < 12500 )
				{
					set_led_color(LED_COLOR_PURPLE);
				}
				else if ( result_line.x_accel_result < 12500 && result_line.y_accel_result < 12500 && result_line.z_accel_result < 12500)
				{
					set_led_color(LED_COLOR_BLUE);
				}
				else if ( result_orientation_line.x_accel_result < 12500 && result_orientation_line.y_accel_result < 12500 && result_orientation_line.z_accel_result < 12500 )
				{
					set_led_color(LED_COLOR_BROWN);
				}
				else
				{
					set_led_color(LED_COLOR_RED);
				}
				


				ring_buffer_clear(&ring_buffer_1);
				state = IDLE;
			break;

			case HARD_RESET:
				//hard reset
				MPU6050_hard_reset_flag = 0;

				uart_println("MPU6050 Hard Reset Needed");

				//debug
				while(1)
				{

				}
			break;

			default:
				uart_println("UNKNOWN STATE");
			break;

		}

		if(MPU6050_hard_reset_flag)
		{
			state = HARD_RESET;
		}

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0xFFFF;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 25;//should be 50hz //1hz is 1281;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
                          |GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
