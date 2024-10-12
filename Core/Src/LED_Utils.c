#include "LED_Utils.h"

void set_led_color(LED_COLOR color)
{
	HAL_GPIO_WritePin(LED_GPIO_BANK, LED_PIN_RED | LED_PIN_GREEN | LED_PIN_BLUE, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GPIO_BANK, color, GPIO_PIN_SET);
}