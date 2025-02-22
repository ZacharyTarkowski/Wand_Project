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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

#define DEBUG_STATE_DEF 0

Mpu_6050_handle_s MPU6050_handle;
Mpu_6050_handle_s* pMPU6050 = &MPU6050_handle;

HAL_StatusTypeDef wand_init()
{
  //30hz (1024/33)
  u8 sample_rate_divider = 33;

  u32 default_ring_config = 0x0;

  HAL_StatusTypeDef status = HAL_ERROR;
	status = MPU6050_init(pMPU6050, &hi2c1, 0x68, sample_rate_divider, 0x30, 0x40, 0x06 );
  
  status |= ring_buffer_init(&ring_buffer_capture_1, default_ring_config, "Capture 1", 0, 0, MPU_6050_NUM_DIMS, SPELL_SIZE + SPELL_COMPARISON_TICK_SIZE + 10);
	status |= ring_buffer_init(&ring_buffer_capture_2, default_ring_config, "Capture 2", 0, 0, MPU_6050_NUM_DIMS, SPELL_SIZE + SPELL_COMPARISON_TICK_SIZE + 10);
  status |= ring_buffer_init(&ring_buffer_capture_3, default_ring_config, "Capture 3", 0, 0, MPU_6050_NUM_DIMS, SPELL_SIZE);

  status |= ring_buffer_init(&ring_buffer_spell_1, default_ring_config, "Spell 1", ring_1_initial_data, sizeof(ring_1_initial_data), MPU_6050_NUM_DIMS,  SPELL_SIZE );
	status |= ring_buffer_init(&ring_buffer_spell_2, default_ring_config, "Spell 2", ring_2_initial_data, sizeof(ring_2_initial_data), MPU_6050_NUM_DIMS, SPELL_SIZE );
  status |= ring_buffer_init(&ring_buffer_spell_3, default_ring_config, "Spell 3", ring_3_initial_data, sizeof(ring_3_initial_data), MPU_6050_NUM_DIMS, SPELL_SIZE + 20);

  status |= ring_buffer_init(&ring_buffer_idle, default_ring_config, "Idle", 0, 0, MPU_6050_NUM_DIMS,  RING_BUFFER_MAX_SIZE);
  
	status |= dtw_init();

  
	capture_flag = 0;
	timer_flag = 0;
	data_ready_flag = 0;
	timer_flag = 0;
  set_led_color(LED_COLOR_RED);
  HAL_TIM_Base_Start_IT(&htim3);

  return status;
}

#define ADVANCE_STATE(next_state) last_state = state; state = next_state; 

//this also does IDLE->CAPTURE_2
#define BUFFER_SWAP(last_state) current_buffer = (last_state == CAPTURE_2) ? &ring_buffer_capture_1 : &ring_buffer_capture_2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  WAND_STATE last_state = INIT;
  ring_buffer_s* current_buffer;
	
	status = wand_init();

	if(status != HAL_OK)
	{
		return -1;
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  float pitch_accel = 0; float roll_accel  = 0;
  float pitch_accel_1 = 0; float roll_accel_1  = 0;
  float pitch_accel_2 = 0; float roll_accel_2  = 0;
  volatile float pitch_accel_deg = 0; volatile float roll_accel_deg  = 0;

  u32 num_samples = 0;
  u32 last_capture_num_samples = 0;
  u32 sample_window_start = 0;
  u8 first_run = 1;
  u8 MPU6050_hard_reset_flag = 0;

  state = INIT;
  #if DEBUG_STATE_DEF
    state = DEBUG_STATE;
  #endif
  //state = GET_SPELL;

  MPU6050_reset_fifo(pMPU6050);

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		switch(state)
		{
      case INIT:
        current_buffer = &ring_buffer_capture_1;

        if(data_ready_flag && status != HAL_ERROR)
				{
					data_ready_flag = 0;
					status = ring_buffer_MPU6050_get_accel_sample(pMPU6050,  current_buffer);
				}

        if(current_buffer->write_index > SPELL_SIZE)
        {
          MPU6050_reset_fifo(pMPU6050);
          //ADVANCE_STATE(ROLLING_COMPARE);
          ADVANCE_STATE(CAPTURE_1);
        }

      break;

			case IDLE:

      if(timer_flag)
			{
				timer_flag = 0;

        if(finished_all_dtw_comparisons != 1)
        {
          uart_println("DTW did not finish in time!");
          state = HARD_RESET;
        }

        ADVANCE_STATE(CAPTURE_1);
			}

			break;

			case CAPTURE_1:
			case CAPTURE_2:

        ring_buffer_set_index(current_buffer, last_capture_num_samples);

        if(data_ready_flag && status != HAL_ERROR)
        {
          data_ready_flag = 0;
          status = ring_buffer_MPU6050_read_and_store(pMPU6050,  current_buffer, &last_capture_num_samples);
        }
        ADVANCE_STATE(ROLLING_COMPARE);
      break;

      case ROLLING_COMPARE:

          uart_println("%s last capture %d",current_buffer->name, last_capture_num_samples);
          get_accel_angles(current_buffer, &pitch_accel,&roll_accel);
          //uart_println("Angle estimates : Pitch %f, Roll %f", pitch_accel, roll_accel);

          ring_buffer_copy(&ring_buffer_capture_2, current_buffer);
          ring_buffer_pitch_roll_rotation(&ring_buffer_capture_2, pitch_accel, roll_accel);

          //uart_println("Comparison to %s",spell_name_1);
          result = DTW_Distance(&ring_buffer_capture_2, &ring_buffer_spell_1);
          uart_println("Spell 1 %d %d", result.x_accel_result, result.z_accel_result);
          if(result.x_accel_result < 10000 && result.z_accel_result < 10000)
          {
            uart_println("live match");
            uart_println("Angle estimates : Pitch %f, Roll %f", pitch_accel, roll_accel);
            ring_buffer_print_to_write_index(&ring_buffer_capture_2);
          }

          result = DTW_Distance(&ring_buffer_capture_2, &ring_buffer_spell_2);
          uart_println("Spell 2 %d %d", result.x_accel_result, result.z_accel_result);

          //TEMP TEMP TEMP FOR TIMING
          //MPU6050_reset_fifo(pMPU6050);
          HAL_Delay(150);


          ADVANCE_STATE(CAPTURE_1);

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

      case DEBUG_STATE:


        ring_buffer_print_to_write_index(&ring_buffer_spell_3);
        result = DTW_Distance(&ring_buffer_spell_1, &ring_buffer_spell_3);
				print_dtw_result(&result);

        result = DTW_Distance(&ring_buffer_spell_1, &ring_buffer_spell_2);
				print_dtw_result(&result);

        u32 startTime = HAL_GetTick();
        u32 endTime;
        for(int i = 0; i<200; i++)
        {
          //memcpy(ring_buffer_spell_1.buffer[0],ring_buffer_spell_1.buffer[3],ring_buffer_spell_1.write_index*sizeof(buffer_element));
        }
        endTime = HAL_GetTick();
        uart_println("Memcpy (200) time : %d Size : %d", endTime - startTime,ring_buffer_spell_1.write_index);
        
        startTime = HAL_GetTick();
        u32 n = 15;
        for(int i = 0; i<n; i++)
        {
          ring_buffer_MPU6050_get_accel_sample(pMPU6050, &ring_buffer_idle);
        }
        endTime = HAL_GetTick();
        uart_println("%d Samples : %d ms",n, endTime - startTime);

        ring_buffer_print_to_write_index(&ring_buffer_capture_3);
        ring_buffer_print_to_write_index(&ring_buffer_spell_1);
        startTime = HAL_GetTick();
        ring_buffer_copy(&ring_buffer_capture_3,&ring_buffer_spell_1);
        endTime = HAL_GetTick();
        uart_println("Ringbuffcpy time : %d Size : %d x %d", endTime - startTime,ring_buffer_spell_1.dim_size,ring_buffer_spell_1.num_dims);

        startTime = HAL_GetTick();
        ring_buffer_set_index(&ring_buffer_capture_3,15);
        endTime = HAL_GetTick();
        ring_buffer_print_to_write_index(&ring_buffer_capture_3);
        uart_println("Setindex time : %d Size : %d x %d", endTime - startTime);
        

        ring_buffer_print_to_write_index(&ring_buffer_capture_3);
        startTime = HAL_GetTick();
        status = MPU6050_get_fifo_data(pMPU6050, &num_samples);
        ring_buffer_MPU6050_read_and_store(pMPU6050, &ring_buffer_capture_3, &last_capture_num_samples);
        endTime = HAL_GetTick();
        uart_println("%d Samples : %d ms",num_samples, endTime - startTime);

        //this should wait for an entire button press cycle
        while(!capture_flag){
            // startTime = HAL_GetTick();
            // status = MPU6050_get_fifo_data(pMPU6050, &num_samples);
            // ring_buffer_MPU6050_read_and_store(pMPU6050, &ring_buffer_capture_3);
            // endTime = HAL_GetTick();
            // uart_println("%d Samples : %d ms",num_samples, endTime - startTime);
            // HAL_Delay(1000);
          };
        while(capture_flag){};

      break;
        
      case GET_SPELL:
        if(capture_flag)
        {
          current_buffer = &ring_buffer_capture_1;
          ring_buffer_clear(current_buffer);
          while(current_buffer->write_index < SPELL_SIZE)
          {
            if(data_ready_flag && status != HAL_ERROR)
            {
              data_ready_flag = 0;
              status = ring_buffer_MPU6050_get_accel_sample(pMPU6050,  current_buffer);
            }
          }

          get_accel_angles(current_buffer, &pitch_accel,&roll_accel);
          uart_println("Angle estimates : Pitch %f, Roll %f", pitch_accel, roll_accel);

          ring_buffer_pitch_roll_rotation(current_buffer, pitch_accel, roll_accel);

          ring_buffer_print_to_write_index(current_buffer);
        }
        while(capture_flag){};
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

    if(status == HAL_ERROR)
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
