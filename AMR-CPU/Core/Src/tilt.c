/*
 * tilt.c
 *
 *  Created on: Jun 10, 2024
 *      Author: gusta
 */

#include "tilt.h"


uint8_t LSM9DS1_Write(uint8_t reg, uint8_t data) {
        uint8_t txData[2] = {reg , data};
        uint8_t error = 0;

        // Trigger conversion and receive data
        HAL_GPIO_WritePin(GPIOA, CSAG_Pin, GPIO_PIN_RESET); // Pull CS pin low
        HAL_Delay(10);
        error = HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, CSAG_Pin, GPIO_PIN_SET); // Pull CS pin high
        HAL_Delay(10);

        return error;
    }

  uint8_t LSM9DS1_Read(uint8_t reg, uint8_t *data) {
          uint8_t txData[2] = {reg | 0x80, 0}; // Set MSB to 1 for read
          uint8_t rxData[2] = {0, 0};

          uint8_t error = 0;

          // Trigger conversion and receive data
          HAL_GPIO_WritePin(GPIOA, CSAG_Pin, GPIO_PIN_RESET); // Pull CS pin low
          HAL_Delay(10);
          error = HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, HAL_MAX_DELAY);
          HAL_GPIO_WritePin(GPIOA, CSAG_Pin, GPIO_PIN_SET); // Pull CS pin high
          HAL_Delay(10);
          (*data) = rxData[1];

          return error;
      }
  uint8_t LSM9DS1_Init(void){
	  uint8_t error = 0;
	  error = LSM9DS1_Write(LSM9DS1_CTRL_REG6_XL, 0x60); // Set 119Hz and +- 2g
	  error = LSM9DS1_Write(LSM9DS1_CTRL_REG7_XL, 0x80); // Enable high resolution and LP, cutoff 50hz
	  error = LSM9DS1_Write(LSM9DS1_CTRL_REG9, 0x44);  // Disable I2C and enable gyroscope sleep
	  return error;
  }
  uint8_t LSM9DS1_Read_Acceleration_X(int16_t *accValue) {
	  uint8_t xl;
	  uint8_t xh;
      uint8_t error = LSM9DS1_Read(LSM9DS1_OUT_X_L_XL, &xl);
      error = LSM9DS1_Read(LSM9DS1_OUT_X_L_XL + 1, &xh);
      (*accValue) = (int16_t)(xh << 8 | xl);
      return error;
  }
  uint8_t LSM9DS1_Read_Acceleration_Y(int16_t *accValue) {
  	  uint8_t yl;
  	  uint8_t yh;
	  uint8_t error = LSM9DS1_Read(LSM9DS1_OUT_Y_L_XL, &yl);
      error = LSM9DS1_Read(LSM9DS1_OUT_Y_L_XL + 1, &yh);
      (*accValue) = (int16_t)(yh << 8 | yl);
      return error;
    }
  uint8_t LSM9DS1_Read_Acceleration_Z(int16_t *accValue) {
  	  uint8_t zl;
  	  uint8_t zh;
      uint8_t error = LSM9DS1_Read(LSM9DS1_OUT_Z_L_XL, &zl);
      error = LSM9DS1_Read(LSM9DS1_OUT_Z_L_XL + 1, &zh);
      (*accValue) = (int16_t)(zh << 8 | zl);
      return error;
    }
  float convAcc(int16_t accvalue, float acc_ref){
  	  float v_in;
  	  v_in = ((accvalue+125)*acc_ref)/16384;
  	  return v_in;
    }
  double findTilt(float x, float y, float z){
	  double temp = (double)((y)/sqrt((x*x)+(y*y)+(z*z)));
	  return (acos(temp)*180)/M_PI;
  }
