/*
 * UART_Utils.c
 *
 *  Created on: Aug 13, 2024
 *      Author: zacht
 */

#include "General_Utils.h"

/* Master Todo : General Utils
 *
 * Probably want to add some form of choose your uart object
 * Definitely want to abstract the HAL_UART_TRANSMIT calls so other vendor HAL's could be popped in
 * Do some recieve stuff
 * Do cool shit like clearing lines for status percentages etc
 * Not exactly a thread safe file
 *
 */

UART_HandleTypeDef huart2;
static char msg_buf[512];

const char* newlineBuffer = "\n\r";

//todo fix newlines
void printstring(char* msg)
{
	HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, newlineBuffer, strlen(newlineBuffer), HAL_MAX_DELAY);
}

void uart_printf(const char* fmt, ...)
{
	//variable arguments because I'm cool
	va_list args;
	va_start(args, fmt);


	//variable snprintf
	vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
	HAL_UART_Transmit(&huart2, msg_buf, strlen(msg_buf), HAL_MAX_DELAY);

	va_end(args);
}

void uart_println(const char* fmt, ...)
{
	//variable arguments because I'm cool
	va_list args;
	va_start(args, fmt);


	//variable snprintf
	vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
	HAL_UART_Transmit(&huart2, msg_buf, strlen(msg_buf), HAL_MAX_DELAY);

	//send a newline
	HAL_UART_Transmit(&huart2, newlineBuffer, strlen(newlineBuffer), HAL_MAX_DELAY);

	va_end(args);
}

