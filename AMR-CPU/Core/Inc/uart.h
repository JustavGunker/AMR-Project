/*
 * uart.h
 *
 *  Created on: Jun 10, 2024
 *      Author: gusta
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"


void writeUART(char* string);
void clrscr();
void gotoxy(uint8_t x, uint8_t y);

#endif /* INC_UART_H_ */
