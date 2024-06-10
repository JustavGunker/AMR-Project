/*
 * tilt.h
 *
 *  Created on: Jun 10, 2024
 *      Author: gusta
 */

#ifndef INC_TILT_H_
#define INC_TILT_H_



#include "main.h"


#define LSM9DS1_CTRL_REG6_XL 0x20
#define LSM9DS1_CTRL_REG7_XL 0x21
#define LSM9DS1_CTRL_REG9 0x23
#define LSM9DS1_OUT_X_L_XL 0x28
#define LSM9DS1_OUT_Y_L_XL 0x2A
#define LSM9DS1_OUT_Z_L_XL 0x2C



void LSM9DS1_Write(SPI_HandleTypeDef* spi,uint8_t reg, uint8_t data);
uint8_t LSM9DS1_Read(SPI_HandleTypeDef* spi,uint8_t reg);
void LSM9DS1_Init(SPI_HandleTypeDef* spi);
void LSM9DS1_Read_Acceleration(SPI_HandleTypeDef* spi, int16_t* accArray);
float convAcc(int16_t accvalue, float acc_ref);
double getTilt(SPI_HandleTypeDef* spi);




#endif /* INC_TILT_H_ */
