/*
 * adc.h
 *
 *  Created on: Jun 10, 2024
 *      Author: gusta
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

typedef struct adc_t
{
	GPIO_TypeDef * Port;
	uint16_t Pin;
}adc_t;

void ADC_Init(adc_t* adc1, adc_t* adc2, adc_t* adc3);
uint16_t LTC2452_Read(SPI_HandleTypeDef* spi, adc_t adc);
float convVol(uint16_t adcvalue, float v_ref);
float convVol2(uint16_t adcvalue, float v_ref);
float vol2uTesla(float voltage);
float get_magx(SPI_HandleTypeDef* spi,adc_t adc);
float get_magz(SPI_HandleTypeDef* spi,adc_t adc);
float get_magy(SPI_HandleTypeDef* spi,adc_t adc);

#endif /* INC_ADC_H_ */
