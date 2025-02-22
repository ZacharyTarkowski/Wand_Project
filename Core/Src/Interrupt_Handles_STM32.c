#include "Interrupt_Handles_STM32.h"

volatile u8 data_ready_flag = 0;
volatile u8 capture_flag = 0;
volatile u32 capture_flag_valid_time;
volatile u8 timer_flag = 0;
volatile u8 finished_all_dtw_comparisons = 0;

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


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == htim3.Instance)
	{
		timer_flag = 1;
	}
}