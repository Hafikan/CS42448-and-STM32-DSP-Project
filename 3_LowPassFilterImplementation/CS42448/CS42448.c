/*
 * CS42448_I2CController.cpp
 *
 *  Created on: Mar 19, 2024
 *      Author: cerberus
 */

#include "CS42448.h"
#include "stm32f3xx_hal.h"

HAL_StatusTypeDef Codec_IsReady(Codec_Typedef * codec){

	HAL_StatusTypeDef device_is_ready;
	GPIO_PinState CDIN_Pin, CS_Pin;

	CDIN_Pin = HAL_GPIO_ReadPin(codec->Codec_CDIN_Pin_Port, codec->Codec_CDIN_Pin);
	CS_Pin = HAL_GPIO_ReadPin(codec->Codec_CS_Pin_Port, codec->Codec_CS_Pin);
	device_is_ready = HAL_I2C_IsDeviceReady(&codec->i2c_port, codec->CHIP_ADDRESS, codec->Trials, codec->Timeout);
	return device_is_ready;
}
HAL_StatusTypeDef PowerDownEnable(Codec_Typedef * codec){
	HAL_GPIO_WritePin(codec->Codec_Reset_Pin_Port, codec->Codec_Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(codec->Timeout);
	unsigned short int enable_data_buffer[2] = {0xFF};
	HAL_StatusTypeDef is_write = HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, enable_data_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;
}
HAL_StatusTypeDef PowerDownDisable(Codec_Typedef * codec){

	unsigned short int disable_data_buffer[2] = {0x00};
	HAL_StatusTypeDef is_write = HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, disable_data_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);

	return is_write;
}
HAL_StatusTypeDef MuteAdc(Codec_Typedef * codec){
	uint8_t mute_buffer[2]={24};
	HAL_StatusTypeDef is_write =HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x06, I2C_MEMADD_SIZE_8BIT, mute_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;
}
HAL_StatusTypeDef HandleRegisters(Codec_Typedef * codec,uint8_t register_address){
	uint8_t ic_r_buffer[2]={};
	HAL_StatusTypeDef is_read=HAL_I2C_Mem_Read(&codec->i2c_port,codec->CHIP_ADDRESS, register_address, I2C_MEMADD_SIZE_8BIT, ic_r_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_read;
}

HAL_StatusTypeDef SetI2SInterface(Codec_Typedef * codec){
	uint8_t ic_d_buffer[2]={201};
	uint8_t ic_d_buffer1[2]={73};
	HAL_StatusTypeDef is_write =HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x04, I2C_MEMADD_SIZE_8BIT, ic_d_buffer, 1,codec->Timeout);
	HAL_Delay(codec->Timeout);
	is_write =HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x04, I2C_MEMADD_SIZE_8BIT, ic_d_buffer1, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;

}

HAL_StatusTypeDef SetADCMode(Codec_Typedef * codec){

	uint8_t ac_d_buffer[2]={28};
	HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x05, I2C_MEMADD_SIZE_8BIT, ac_d_buffer, 1, codec->Timeout);
	HAL_Delay(500);



}
HAL_StatusTypeDef InVolumeControl(Codec_Typedef * codec, uint8_t in_ch_register_addr){
	HAL_StatusTypeDef is_write = HAL_BUSY;
	uint8_t vol[2] = {0b01110000};
	is_write = HAL_I2C_Mem_Write(&codec->i2c_port, codec->CHIP_ADDRESS, in_ch_register_addr, I2C_MEMADD_SIZE_8BIT, vol, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;
}
void Filter_Init(EMA_LPF * filter, float alpha){
	Filter_SetAlpha(filter, alpha);
	filter->filtered_signal = 0.0f;


}
void Filter_SetAlpha(EMA_LPF * filter, float alpha){
	filter->alpha = alpha;

}
float Filter_Update(EMA_LPF * filter, float input){
	filter->filtered_signal = filter->alpha * input + (1-filter->alpha)*filter->filtered_signal;
	return filter->filtered_signal;
}
void LedControl(Codec_Typedef * codec){

	HAL_GPIO_TogglePin(codec->Codec_Reset_Pin_Port, codec->Codec_Reset_Pin);
	HAL_Delay(codec->Timeout);

}


