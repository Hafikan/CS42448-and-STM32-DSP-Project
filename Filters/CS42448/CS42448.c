/*
 * CS42448.c
 *
 *  Created on: Mar 27, 2024
 *      Author: Yunus TORUN
 */

#include "CS42448.h"

#include <inttypes.h>
#include "stm32g4xx_hal.h"
HAL_StatusTypeDef   Codec_IsReady(Codec_Typedef * codec){

	HAL_StatusTypeDef device_is_ready;
	device_is_ready = HAL_I2C_IsDeviceReady(&codec->i2c_port, codec->CHIP_ADDRESS, codec->Trials, codec->Timeout);
	return device_is_ready;
}
HAL_StatusTypeDef   PowerDownEnable(Codec_Typedef * codec){
	HAL_GPIO_WritePin(codec->Codec_Reset_Pin_Port, codec->Codec_Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(codec->Timeout);
	uint8_t enable_data_buffer[2] = {0xFF};
	HAL_StatusTypeDef is_write = HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, enable_data_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;
}
HAL_StatusTypeDef   PowerDownDisable(Codec_Typedef * codec){

	uint8_t disable_data_buffer[2] = {0x00};
	HAL_StatusTypeDef is_write = HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, disable_data_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);

	return is_write;
}
HAL_StatusTypeDef  MuteAdc(Codec_Typedef * codec){
	uint8_t mute_buffer[2]={24};
	HAL_StatusTypeDef is_write =HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x06, I2C_MEMADD_SIZE_8BIT, mute_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;
}
HAL_StatusTypeDef  HandleRegisters(Codec_Typedef * codec,uint8_t register_address){
	uint8_t ic_r_buffer[2]={};
	HAL_StatusTypeDef is_read=HAL_I2C_Mem_Read(&codec->i2c_port,codec->CHIP_ADDRESS, register_address, I2C_MEMADD_SIZE_8BIT, ic_r_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_read;
}

HAL_StatusTypeDef  SetI2SInterface(Codec_Typedef * codec){
	uint8_t ic_d_buffer[2]={201};
	uint8_t ic_d_buffer1[2]={73};
	HAL_StatusTypeDef is_write =HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x04, I2C_MEMADD_SIZE_8BIT, ic_d_buffer, 1,codec->Timeout);
	HAL_Delay(codec->Timeout);
	is_write =HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x04, I2C_MEMADD_SIZE_8BIT, ic_d_buffer1, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;

}

HAL_StatusTypeDef  SetADCMode(Codec_Typedef * codec){

	uint8_t ac_d_buffer[2]={28};
	HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x05, I2C_MEMADD_SIZE_8BIT, ac_d_buffer, 1, codec->Timeout);
	HAL_Delay(500);



}
HAL_StatusTypeDef  InVolumeControl(Codec_Typedef * codec, uint8_t in_ch_register_addr){
	HAL_StatusTypeDef is_write = HAL_BUSY;
	uint8_t vol[2] = {0b01110000};
	is_write = HAL_I2C_Mem_Write(&codec->i2c_port, codec->CHIP_ADDRESS, in_ch_register_addr, I2C_MEMADD_SIZE_8BIT, vol, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;
}


