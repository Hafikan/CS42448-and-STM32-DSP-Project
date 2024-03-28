/*
 * CS42448.h
 *
 *  Created on: Mar 27, 2024
 *      Author: Yunus TORUN
 */

#include <inttypes.h>
#include "stm32g4xx_hal.h"
#ifndef CS42448_H_
#define CS42448_H_
typedef struct{
	I2C_HandleTypeDef i2c_port;
	uint8_t CHIP_ADDRESS;
	uint16_t Timeout;
	uint8_t Trials;
	GPIO_TypeDef* Codec_Reset_Pin_Port;
	uint16_t Codec_Reset_Pin;

}Codec_Typedef;
HAL_StatusTypeDef Codec_IsReady(Codec_Typedef * codec);
HAL_StatusTypeDef PowerDownEnable(Codec_Typedef * codec);
HAL_StatusTypeDef PowerDownDisable(Codec_Typedef * codec);
HAL_StatusTypeDef MuteAdc(Codec_Typedef * codec);
HAL_StatusTypeDef HandleRegisters(Codec_Typedef * codec,uint8_t register_address);
HAL_StatusTypeDef SetI2SInterface(Codec_Typedef * codec);
HAL_StatusTypeDef SetADCMode(Codec_Typedef * codec);
HAL_StatusTypeDef InVolumeControl(Codec_Typedef * codec, uint8_t in_ch_register_addr);

#endif /* CS42448_H_ */
