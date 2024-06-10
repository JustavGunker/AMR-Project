/*
 * lora.h
 *
 *  Created on: Jun 10, 2024
 *      Author: gusta
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "main.h"

#define RegFiFo					0x00
#define RegOpMode				0x01
#define RegFrMsb				0x06
#define RegFrMid				0x07
#define RegFrLsb				0x08
#define RegPaConfig				0x09
#define RegOcp					0x0B
#define RegLna					0x0C
#define RegFiFoAddPtr			0x0D
#define RegFiFoTxBaseAddr		0x0E
#define RegFiFoRxBaseAddr		0x0F
#define RegFiFoRxCurrentAddr	0x10
#define RegIrqFlags				0x12
#define RegRxNbBytes			0x13
#define RegPktRssiValue			0x1A
#define	RegModemConfig1			0x1D
#define RegModemConfig2			0x1E
#define RegSymbTimeoutL			0x1F
#define RegPreambleMsb			0x20
#define RegPreambleLsb			0x21
#define RegPayloadLength		0x22
#define RegModemConfig3			0x26
#define RegSyncWord				0x39
#define RegDioMapping1			0x40
#define RegDioMapping2			0x41
#define RegVersion				0x42

#define sleep_mode			0
#define	stdby_mode			1
#define tx_mode				3
#define rx_cont_mode		5
#define rx_single_mode		6

void LoRa_reset();
void LoRa_write_reg(SPI_HandleTypeDef spi, uint8_t* address, uint8_t* data);
uint8_t LoRa_read_reg(SPI_HandleTypeDef spi, uint8_t address);
void LoRa_fill_fifo(SPI_HandleTypeDef spi, uint8_t* data, uint8_t bytes);
void LoRa_set_mode(SPI_HandleTypeDef spi, uint8_t mode);
void LoRa_init(SPI_HandleTypeDef spi);
char LoRa_read_payload(SPI_HandleTypeDef spi, char* data);

#endif /* INC_LORA_H_ */
