/*
 * tilt.c
 *
 *  Created on: Jun 10, 2024
 *      Author: gusta
 */

#include "tilt.h"


void LSM9DS1_Write(SPI_HandleTypeDef* spi,uint8_t reg, uint8_t data) {
        uint8_t txData[2] = {reg , data};

        // Trigger conversion and receive data
        HAL_GPIO_WritePin(GPIOA, CSAG_Pin, GPIO_PIN_RESET); // Pull CS pin low
        HAL_Delay(10);
        HAL_SPI_Transmit(spi, txData, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, CSAG_Pin, GPIO_PIN_SET); // Pull CS pin high
        HAL_Delay(10);
    }

  uint8_t LSM9DS1_Read(SPI_HandleTypeDef* spi, uint8_t reg) {
          uint8_t txData[2] = {reg | 0x80, 0}; // Set MSB to 1 for read
          uint8_t rxData[2] = {0, 0};

          // Trigger conversion and receive data
          HAL_GPIO_WritePin(GPIOA, CSAG_Pin, GPIO_PIN_RESET); // Pull CS pin low
          HAL_Delay(10);
          HAL_SPI_TransmitReceive(spi, txData, rxData, 2, HAL_MAX_DELAY);
          HAL_GPIO_WritePin(GPIOA, CSAG_Pin, GPIO_PIN_SET); // Pull CS pin high
          HAL_Delay(10);
          return rxData[1];
      }
  void LSM9DS1_Init(SPI_HandleTypeDef* spi){
	  LSM9DS1_Write(spi,LSM9DS1_CTRL_REG6_XL, 0x60); // Set 119Hz and +- 2g
	  LSM9DS1_Write(spi,LSM9DS1_CTRL_REG7_XL, 0x80); // Enable high resolution and LP, cutoff 50hz
	  LSM9DS1_Write(spi,LSM9DS1_CTRL_REG9, 0x44);  // Disable I2C and enable gyroscope sleep
  }
  void LSM9DS1_Read_Acceleration(SPI_HandleTypeDef* spi, int16_t* accArray) {
  	  uint8_t xl,xh,yl,yh,zl,zh;
      xl = LSM9DS1_Read(spi,LSM9DS1_OUT_X_L_XL);
      xh = LSM9DS1_Read(spi,LSM9DS1_OUT_X_L_XL + 1);
      yl = LSM9DS1_Read(spi,LSM9DS1_OUT_Y_L_XL);
      yh = LSM9DS1_Read(spi,LSM9DS1_OUT_Y_L_XL + 1);
	  zl = LSM9DS1_Read(spi,LSM9DS1_OUT_Z_L_XL);
	  zh = LSM9DS1_Read(spi,LSM9DS1_OUT_Z_L_XL + 1);
      accArray[0] = (int16_t)(xh << 8 | xl);
      accArray[1] = (int16_t)(yh << 8 | yl);
      accArray[2] = (int16_t)(zh << 8 | zl);
  }

  float convAcc(int16_t accvalue, float acc_ref){
  	  float input = ((accvalue+125)*acc_ref)/16384;
  	  return input;
    }

  double getTilt(SPI_HandleTypeDef* spi){
	  float acc_ref = 1.0;
	  int16_t accArray[3];
	  LSM9DS1_Read_Acceleration(spi, accArray);
	  float x = convAcc(accArray[0],acc_ref);
	  float y = convAcc(accArray[1],acc_ref);
	  float z = convAcc(accArray[2],acc_ref);
	  double temp = (double)((y)/sqrt((x*x)+(y*y)+(z*z)));
	  return (acos(temp)*180)/M_PI;
  }
