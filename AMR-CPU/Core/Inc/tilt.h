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


uint8_t LSM9DS1_Write(uint8_t reg, uint8_t data);
uint8_t LSM9DS1_Read(uint8_t reg, uint8_t *data);
uint8_t LSM9DS1_Init(void);
uint8_t LSM9DS1_Read_Acceleration_X(int16_t *accValue);
uint8_t LSM9DS1_Read_Acceleration_Y(int16_t *accValue);
uint8_t LSM9DS1_Read_Acceleration_Z(int16_t *accValue);
float convAcc(int16_t accvalue, float acc_ref);
double findTilt(float x, float y, float z);




#endif /* INC_TILT_H_ */
