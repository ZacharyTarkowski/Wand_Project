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

Mpu_6050_handle_s MPU6050_handle;
Mpu_6050_handle_s* pMPU6050 = &MPU6050_handle;

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

  DTW_Result result;
  WAND_STATE state = IDLE;
  WAND_STATE next_state = CAPTURE_1;
	
	u8 sample_rate_divider = 10;
	u8 first_run = 1;
  u8 MPU6050_hard_reset_flag = 0;


	status = MPU6050_init(pMPU6050, &hi2c1, 0x68, sample_rate_divider, 0x30, 0x40, 0x06 );
  status |= ring_buffer_init(&ring_buffer_capture_1, ring_1_initial_data, sizeof(ring_1_initial_data), MPU_6050_NUM_DIMS,  SPELL_SIZE);
	status |= ring_buffer_init(&ring_buffer_capture_2, ring_2_initial_data, sizeof(ring_2_initial_data), MPU_6050_NUM_DIMS, SPELL_SIZE);
  status |= ring_buffer_init(&ring_buffer_capture_3, 0, 0, MPU_6050_NUM_DIMS, SPELL_SIZE);

  status |= ring_buffer_init(&ring_buffer_spell_1, ring_1_initial_data, sizeof(ring_1_initial_data), MPU_6050_NUM_DIMS,  SPELL_SIZE );
	status |= ring_buffer_init(&ring_buffer_spell_2, ring_2_initial_data, sizeof(ring_2_initial_data), MPU_6050_NUM_DIMS, SPELL_SIZE );

  status |= ring_buffer_init(&ring_buffer_idle, 0, 0, MPU_6050_NUM_DIMS,  RING_BUFFER_MAX_SIZE);
  
	status |= dtw_init();

	if(status != HAL_OK)
	{
		return -1;
	}

	capture_flag = 0;
	timer_flag = 0;
	data_ready_flag = 0;
	timer_flag = 0;
  set_led_color(LED_COLOR_RED);
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  volatile float integral_pitch  = 0;
  volatile float integral_roll   = 0;
  volatile float integral_yaw    = 0;

  float pitch_accel = 0;
  float roll_accel  = 0;

  float pitch_accel_1 = 0;
  float roll_accel_1  = 0;

  float pitch_accel_2 = 0;
  float roll_accel_2  = 0;

  volatile float pitch_accel_deg = 0;
  volatile float roll_accel_deg  = 0;

  u32 num_samples = 0;
  u32 sample_window_start = 0;

  //ring_buffer_pitch_roll_rotation(&ring_buffer_capture_1,0,0);
  //ring_buffer_pitch_roll_rotation(&ring_buffer_capture_2,0,0);

  uart_println("DTW Post rotation");
  result = DTW_Distance(&ring_buffer_capture_1, &ring_buffer_capture_2);
  result = DTW_Distance(&ring_buffer_capture_2, &ring_buffer_capture_1);

  buffer_element temp[6];
  for(int i = 0; i< 384; i++)
  {
    buffer_element temp[6] = {i*1000};
    ring_buffer_write_element(&ring_buffer_capture_3,temp);
  }
  //ring_buffer_print_all_elements(&ring_buffer_capture_3);
  ring_buffer_print_all_elements_from_read_index(&ring_buffer_capture_3);
  ring_buffer_capture_3.rollover_count = 1;
  ring_buffer_pitch_roll_rotation(&ring_buffer_capture_3,0,0);
  ring_buffer_capture_3.rollover_count = 0;
  ring_buffer_print_all_elements_from_read_index(&ring_buffer_capture_3);
  ring_buffer_capture_3.rollover_count = 0;
  ring_buffer_pitch_roll_rotation(&ring_buffer_capture_3,0,180);
  ring_buffer_capture_3.rollover_count = 0;
  ring_buffer_pitch_roll_rotation(&ring_buffer_capture_3,0,180);
  //ring_buffer_print_all_elements_from_read_index(&ring_buffer_capture_3);

  ring_buffer_clear(&ring_buffer_capture_1);
  ring_buffer_clear(&ring_buffer_capture_2);
  ring_buffer_clear(&ring_buffer_capture_3);

  //state = ROLLING_COMPARE;
  state = IDLE;

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		switch(state)
		{
			case IDLE:
			if(capture_flag)
			{
				first_run = 1;
				state = next_state;
			}
      else if(timer_flag && 1)
			{
				timer_flag = 0;
				if(first_run)
				{
					first_run = 0;
					
          //dont want to clear anymore, for movement detection
					//ring_buffer_clear(&ring_buffer_idle);
					status = ring_buffer_MPU6050_get_accel_sample(pMPU6050, &ring_buffer_idle);
					data_ready_flag = 0;
					
					if(status != HAL_OK)
					{
						uart_println("Failed to reset FIFO!");
						MPU6050_hard_reset_flag = 1;
					}

				}

				if(data_ready_flag && !MPU6050_hard_reset_flag)
				{
					data_ready_flag = 0;

					status = ring_buffer_MPU6050_get_accel_sample(pMPU6050, &ring_buffer_idle);
					if(status != HAL_OK)
					{
						uart_println("Read and store failed");
						MPU6050_hard_reset_flag = 1;
					}

          //constantly doing angle calcs, dont need to do it that way...
          get_accel_angles(&ring_buffer_idle,&pitch_accel,&roll_accel);
          //uart_println("Angle estimates : Pitch %f, Roll %f", pitch_accel, roll_accel);

          //compfilter_tick(&ring_buffer_idle,&pitch_accel,&roll_accel);
          //uart_println("Angle estimates comp : Pitch %f, Roll %f", pitch_accel, roll_accel);

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
					//status = MPU6050_reset_fifo_(pMPU6050);
					//data_ready_flag = 0;

					// if(status != HAL_OK)
					// {
					// 	uart_println("Failed to reset FIFO!");
					// 	MPU6050_hard_reset_flag = 1;
					// }

          if(state == CAPTURE_1)
          {
            pitch_accel_1 = pitch_accel;
            roll_accel_1 = roll_accel;
            next_state = CAPTURE_2;
          }
          else if (state == CAPTURE_2)
          {
            pitch_accel_2 = pitch_accel;
            roll_accel_2 = roll_accel;
            next_state = PRINT_AND_COMPARE;
          }
				}

				if(data_ready_flag && !MPU6050_hard_reset_flag)
				{
					data_ready_flag = 0;

					status = ring_buffer_MPU6050_get_accel_sample(pMPU6050, ( state==CAPTURE_1 ? &ring_buffer_capture_1 : &ring_buffer_capture_2 ) );
					if(status != HAL_OK)
					{
						MPU6050_hard_reset_flag = 1;
					}

				}
			}
			
			if(!capture_flag)
			{
				if( !first_run )
				{
					uart_println("Done Capturing %d",state);

					//state = COMPARE_STATIC; 
					//state = state == CAPTURE_2 ? PRINT_AND_COMPARE : IDLE;
          state = IDLE;
          
					first_run = 1;
				}
			}
			break;

      case ROLLING_COMPARE:

        if(data_ready_flag && !MPU6050_hard_reset_flag)
        {
          data_ready_flag = 0;

          status = ring_buffer_MPU6050_get_accel_sample(pMPU6050, &ring_buffer_capture_3 );
          if(status != HAL_OK)
          {
            MPU6050_hard_reset_flag = 1;
          }

         
          
          num_samples++;
        }

        //remember the first (spellsize / window) are essentially worthless
        if(num_samples >= DTW_WINDOW && ring_buffer_capture_3.rollover_count > 0)
        {
          //get acceleration for the beginning of the calc, not like this
          //get_accel_angles(&ring_buffer_capture_3,&pitch_accel,&roll_accel);
          uart_println("Before");
          //ring_buffer_print_all_elements_from_read_index(&ring_buffer_capture_3);
          result = DTW_Distance(&ring_buffer_capture_3, &ring_buffer_spell_2);

          get_accel_angles(&ring_buffer_capture_3,&pitch_accel,&roll_accel);
          uart_println("Angle estimates : Pitch %f, Roll %f", pitch_accel, roll_accel);
          ring_buffer_pitch_roll_rotation(&ring_buffer_capture_3, pitch_accel, roll_accel);

          
          
          uart_println("Comparison to %s",spell_name_2);
          //ring_buffer_print_all_elements_from_read_index(&ring_buffer_capture_3);
          
          result = DTW_Distance(&ring_buffer_capture_3, &ring_buffer_spell_2);
          //print_dtw_result(&result);

          sample_window_start += DTW_WINDOW;
          if(sample_window_start >= SPELL_SIZE) {sample_window_start -= SPELL_SIZE; }

          num_samples = 0;
        }

      break;

			case PRINT_AND_COMPARE:

				uart_println("Ring Buffer 1");
				//ring_buffer_MPU6050_apply_mean_centering(&ring_buffer_capture_1);
				//ring_buffer_print_to_write_index(&ring_buffer_capture_1);

				uart_println("Ring Buffer 2");
				//ring_buffer_MPU6050_apply_mean_centering(&ring_buffer_capture_2);
				//ring_buffer_print_to_write_index(&ring_buffer_capture_2);

        uart_println("Angle estimates 1 : Pitch %f, Roll %f", pitch_accel_1, roll_accel_1);
        uart_println("Angle estimates 2 : Pitch %f, Roll %f", pitch_accel_2, roll_accel_2 );

        uart_println("DTW pre rotation");
        result = DTW_Distance(&ring_buffer_capture_1, &ring_buffer_capture_2);

        //temp
        uart_println("Buf1 Pre rotation");
        ring_buffer_print_to_write_index(&ring_buffer_capture_1);
				result = DTW_Distance(&ring_buffer_capture_1, &ring_buffer_spell_2);
        uart_println("Buf2 Pre rotation");
        ring_buffer_print_to_write_index(&ring_buffer_capture_2);
				result = DTW_Distance(&ring_buffer_capture_2, &ring_buffer_spell_2);

        ring_buffer_pitch_roll_rotation(&ring_buffer_capture_1,pitch_accel_1,roll_accel_1);
        ring_buffer_pitch_roll_rotation(&ring_buffer_capture_2,pitch_accel_2,roll_accel_2);

        uart_println("DTW Post rotation");
				result = DTW_Distance(&ring_buffer_capture_1, &ring_buffer_capture_2);


        //temp
        uart_println("Buf1 post rotation");
        ring_buffer_print_to_write_index(&ring_buffer_capture_1);
				result = DTW_Distance(&ring_buffer_capture_1, &ring_buffer_spell_2);
        uart_println("Buf2 post rotation");
        ring_buffer_print_to_write_index(&ring_buffer_capture_2);
				result = DTW_Distance(&ring_buffer_capture_2, &ring_buffer_spell_2);

				ring_buffer_clear(&ring_buffer_capture_1);
				ring_buffer_clear(&ring_buffer_capture_2);


				state = IDLE;
        next_state = CAPTURE_1;
        capture_flag = 0;


			break;

			case COMPARE_STATIC:
				uart_println("Ring Buffer 1");

        uart_println("Circle DTW");
        uart_println("Angle estimates : Pitch %f, Roll %f", pitch_accel_1, roll_accel_1 );

        ring_buffer_print_to_write_index(&ring_buffer_capture_1);

        result = DTW_Distance(&ring_buffer_capture_1, &ring_buffer_capture_2);
				print_dtw_result(&result);

        if(result.x_accel_result < DTW_X_THRESHOLD && result.z_accel_result < DTW_Z_THRESHOLD)
        {
          set_led_color(LED_COLOR_GREEN);
        }
        else
        {
          set_led_color(LED_COLOR_RED);
        }

				ring_buffer_clear(&ring_buffer_capture_1);
				state = IDLE;
			break;

			case HARD_RESET:
				//hard reset

				uart_println("MPU6050 Hard Reset Needed");
				//debug. need a circuit to powercycle the accelerometer
				while(1){};
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
  htim3.Init.Period = 1281 / FREQ;
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
