/*
 * uart.c
 *
 *  Created on: Jun 10, 2024
 *      Author: gusta
 */

#include "uart.h"


void writeUART(char* string){
	uart_buf_len = sprintf(uart_buf,string);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}

void clrscr(){
	writeUART("%c[2J",ESC);
  }
void gotoxy(uint8_t x, uint8_t y){
  writeUART("%c[%d;%dH",ESC,y+1,x+1);
}
