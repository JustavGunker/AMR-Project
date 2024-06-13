/*
 * lora.c
 *
 *  Created on: Jun 10, 2024
 *      Author: gusta
 */

#include "lora.h"


void LoRa_reset() {
	HAL_GPIO_WritePin(GPIOA, RST_LoRa_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, RST_LoRa_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
}

/*
 * Writes byte to LoRa module
 * Syntax: LoRa_write_reg(hspi1, &address, &data)
 */
void LoRa_write_reg(SPI_HandleTypeDef* spi, uint8_t address, uint8_t data) {
	uint8_t tx[2] = {address | 0x80,data};

	HAL_GPIO_WritePin(GPIOA, CS_LoRa_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(spi, tx, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOA, CS_LoRa_Pin, GPIO_PIN_SET);
}


/*
 * Read byte from LoRa module and returns value
 * Syntax: x = LoRa_read_reg(hspi1, address)
 */
uint8_t LoRa_read_reg(SPI_HandleTypeDef* spi, uint8_t address) {
	uint8_t txData[2] = {address, 0};
	uint8_t rxData[2] = {0, 0};
	uint8_t data;

	HAL_GPIO_WritePin(GPIOA, CS_LoRa_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);

	HAL_SPI_TransmitReceive(spi, txData, rxData, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOA, CS_LoRa_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	data = rxData[1];

	return data;
}
/*
 * Fills FiFo register (0x00) with data, and sets payload length to number of bytes
 */
void LoRa_fill_fifo(SPI_HandleTypeDef* spi, uint8_t* data, uint8_t bytes) {
	uint8_t payload_len_addr = RegPayloadLength;
	uint8_t tx_addr = RegFiFo | 0x80;


	LoRa_write_reg(spi, payload_len_addr, bytes);

	HAL_GPIO_WritePin(GPIOA, CS_LoRa_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);

	HAL_SPI_Transmit(spi, &tx_addr, 1, HAL_MAX_DELAY);

	for (int i = 0; i < bytes; i++) {
		HAL_SPI_Transmit(spi, data + i, 1, HAL_MAX_DELAY);
	}

	HAL_GPIO_WritePin(GPIOA, CS_LoRa_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

/*
 * Sets device modes. See header for different modes
 */
void LoRa_set_mode(SPI_HandleTypeDef* spi, int8_t mode) {
	uint8_t addr;
	uint8_t read;
	uint8_t data;

	addr = RegOpMode;
	read = LoRa_read_reg(spi, addr);

	data = (read & 0xF8) | mode;

	LoRa_write_reg(spi, addr, data);

	if(mode == tx_mode){
		HAL_GPIO_WritePin(GPIOA, TX_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,RX_Pin,GPIO_PIN_RESET);
	} else if(mode == rx_cont_mode){
		HAL_GPIO_WritePin(GPIOA, TX_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,RX_Pin,GPIO_PIN_SET);
	}
}

/*
 * Initialization of LoRa module. Sleep mode -> LoRa mode -> Stdby mode
 */
void LoRa_init(SPI_HandleTypeDef* spi) {
	uint8_t addr = RegOpMode;
	uint8_t data;
	uint8_t read;
	uint8_t ptrBase;

	LoRa_write_reg(spi,RegFiFoTxBaseAddr,0x00);
	ptrBase = LoRa_read_reg(spi,RegFiFoTxBaseAddr);
	LoRa_write_reg(spi,RegFiFoAddPtr,ptrBase); // Change FiFo pointer to FiFo base adress



	LoRa_set_mode(spi, sleep_mode);
	HAL_Delay(10);


	//enter LoRa mode
	data = (LoRa_read_reg(spi, addr)) | 0x80; //set bit 7 to 1;
	LoRa_write_reg(spi, addr, data);
	HAL_Delay(10);

	LoRa_set_mode(spi, stdby_mode);
	HAL_Delay(10);

	//set frequency
	LoRa_write_reg(spi, RegFrMsb, 0xD9);
	LoRa_write_reg(spi, RegFrMid, 0x00);
	LoRa_write_reg(spi, RegFrLsb, 0x00);

	//crc
	read = LoRa_read_reg(spi, RegModemConfig1) | 0x02;
	LoRa_write_reg(spi, RegModemConfig1, read);
}

void LoRa_read_payload(SPI_HandleTypeDef* spi, char *data) {
	uint8_t irqReg, nBytes, FIFOaddr, i, RxDone, validHeader;

	irqReg = LoRa_read_reg(spi, 0x12);
	RxDone, validHeader = 0;
	irqReg = LoRa_read_reg(spi, 0x12);
	RxDone = irqReg & 0x40;

	if (RxDone) {
		LoRa_write_reg(spi, 0x12, irqReg & 0x40);
		irqReg = LoRa_read_reg(spi, 0x12);
		validHeader = irqReg & 0x10;

		if (validHeader) {
			if (irqReg & 0x20) {
				LoRa_write_reg(spi, 0x12, irqReg & 0x20);
				return "CRC error";
			}

			LoRa_write_reg(spi, 0x12, irqReg & 0x10);

			irqReg = LoRa_read_reg(spi, 0x12);

			nBytes = LoRa_read_reg(spi, 0x13);

			FIFOaddr = LoRa_read_reg(spi, 0x10);

			char *data_reg = (char *)malloc(nBytes*sizeof(char));

			LoRa_write_reg(spi, 0x0D, FIFOaddr);

			for (i = 0; i < nBytes; i++) {
				data_reg[i] = LoRa_read_reg(spi, 0x00);
			}

			strncpy(data,data_reg, nBytes);
			data[nBytes] = '\0';

			free(data_reg);

		}
	}

}

