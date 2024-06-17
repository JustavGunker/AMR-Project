/*
 * adc.c
 *
 *  Created on: Jun 10, 2024
 *      Author: gusta
 */
#include "adc.h"

void ADC_Init(adc_t* adc1, adc_t* adc2, adc_t* adc3){
	adc1->Port = GPIOA;
	adc1->Pin = CSADC1_Pin;

	adc2->Port = GPIOB;
	adc2->Pin = CSADC2_Pin;

	adc3->Port = GPIOB;
	adc3->Pin = CSADC3_Pin;
}


uint16_t LTC2452_Read(SPI_HandleTypeDef* spi, adc_t adc) {
      uint8_t rxData[2] = {0,0}; // 2 bytes to be recieved


      HAL_GPIO_WritePin(adc.Port, adc.Pin, GPIO_PIN_RESET); // Pull CS pin low
      HAL_Delay(10); // Small delay for setup
      HAL_SPI_Receive(spi,rxData,2,HAL_MAX_DELAY);
      HAL_GPIO_WritePin(adc.Port, adc.Pin, GPIO_PIN_SET); // Pull CS pin high
      HAL_Delay(30); // Max conversion time is 23
      // Extract ADC value from received data
      return ((uint16_t)rxData[0] << 8) | rxData[1]; // Result is first and second byte
}

float convVol(uint16_t adcvalue, float v_ref){
  //return (signed)((adcvalue - 32768)*v_ref) >> 15;
  return ((adcvalue - 32768)*v_ref)/32768;
}
float convVol2(uint16_t adcvalue, float v_ref){
  return (adcvalue*v_ref)/32768;
}
