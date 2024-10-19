#ifndef SRC_LED_UTILS_H_
#define SRC_LED_UTILS_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"

#define LED_GPIO_BANK 	GPIOA
#define LED_PIN_RED 	GPIO_PIN_0
#define LED_PIN_BLUE 	GPIO_PIN_4
#define LED_PIN_GREEN 	GPIO_PIN_1

typedef enum { 

	LED_COLOR_RED = LED_PIN_RED,
	LED_COLOR_GREEN = LED_PIN_GREEN,
	LED_COLOR_BLUE = LED_PIN_BLUE,

    //fixme probably
	LED_COLOR_BROWN = LED_PIN_RED | LED_PIN_GREEN,
	LED_COLOR_PURPLE = LED_PIN_RED | LED_PIN_BLUE,
	LED_COLOR_TEAL = LED_PIN_GREEN | LED_PIN_BLUE,
    //fixme
	LED_COLOR_BROWN2 = LED_PIN_RED | LED_PIN_GREEN | LED_PIN_BLUE
	
} LED_COLOR;

void set_led_color(LED_COLOR color);

#endif