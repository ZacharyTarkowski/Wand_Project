#ifndef H_Interrupt_Handles_STM32
#define H_Interrupt_Handles_STM32

#include "General_Utils.h"

extern volatile u8 data_ready_flag;
extern volatile u8 capture_flag;
extern volatile u32 capture_flag_valid_time;
extern volatile u8 timer_flag;
extern volatile u8 finished_all_dtw_comparisons;

extern TIM_HandleTypeDef htim3;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif