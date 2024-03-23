/*
 * CS42448_I2CController.cpp
 *
 *  Created on: Mar 19, 2024
 *      Author: cerberus
 */

#include "CS42448.hpp"
#include "stm32f3xx_hal.h"
#include "math.h"
#include "time.h"
#include "main.h"
HAL_StatusTypeDef Codec_Controller::Codec_IsReady(Codec_Typedef * codec){

	HAL_StatusTypeDef device_is_ready;


	device_is_ready = HAL_I2C_IsDeviceReady(&codec->i2c_port, codec->CHIP_ADDRESS, codec->Trials, codec->Timeout);
	return device_is_ready;
}
HAL_StatusTypeDef Codec_Controller::PowerDownEnable(Codec_Typedef * codec){
	HAL_GPIO_WritePin(codec->Codec_Reset_Pin_Port, codec->Codec_Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(codec->Timeout);
	uint8_t enable_data_buffer[2] = {0xFF};
	HAL_StatusTypeDef is_write = HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, enable_data_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;
}
HAL_StatusTypeDef Codec_Controller::PowerDownDisable(Codec_Typedef * codec){

	uint8_t disable_data_buffer[2] = {0x00};
	HAL_StatusTypeDef is_write = HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, disable_data_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);

	return is_write;
}
HAL_StatusTypeDef Codec_Controller::MuteAdc(Codec_Typedef * codec){
	uint8_t mute_buffer[2]={24};
	HAL_StatusTypeDef is_write =HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x06, I2C_MEMADD_SIZE_8BIT, mute_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;
}
HAL_StatusTypeDef Codec_Controller::HandleRegisters(Codec_Typedef * codec,uint8_t register_address){
	uint8_t ic_r_buffer[2]={};
	HAL_StatusTypeDef is_read=HAL_I2C_Mem_Read(&codec->i2c_port,codec->CHIP_ADDRESS, register_address, I2C_MEMADD_SIZE_8BIT, ic_r_buffer, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_read;
}

HAL_StatusTypeDef Codec_Controller::SetI2SInterface(Codec_Typedef * codec){
	uint8_t ic_d_buffer[2]={201};
	uint8_t ic_d_buffer1[2]={73};
	HAL_StatusTypeDef is_write =HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x04, I2C_MEMADD_SIZE_8BIT, ic_d_buffer, 1,codec->Timeout);
	HAL_Delay(codec->Timeout);
	is_write =HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x04, I2C_MEMADD_SIZE_8BIT, ic_d_buffer1, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;

}

HAL_StatusTypeDef Codec_Controller::SetADCMode(Codec_Typedef * codec){

	uint8_t ac_d_buffer[2]={28};
	HAL_I2C_Mem_Write(&codec->i2c_port,codec->CHIP_ADDRESS, 0x05, I2C_MEMADD_SIZE_8BIT, ac_d_buffer, 1, codec->Timeout);
	HAL_Delay(500);



}
HAL_StatusTypeDef Codec_Controller::InVolumeControl(Codec_Typedef * codec, uint8_t in_ch_register_addr){
	HAL_StatusTypeDef is_write = HAL_BUSY;
	uint8_t vol[2] = {0b01110000};
	is_write = HAL_I2C_Mem_Write(&codec->i2c_port, codec->CHIP_ADDRESS, in_ch_register_addr, I2C_MEMADD_SIZE_8BIT, vol, 1, codec->Timeout);
	HAL_Delay(codec->Timeout);
	return is_write;
}
void Codec_Controller::Filter_Init(EMA_LPF * filter, float alpha){
	Filter_SetAlpha(filter, alpha);
	filter->filtered_signal = 0.0f;


}
void Codec_Controller::Filter_SetAlpha(EMA_LPF * filter, float alpha){
	filter->alpha = alpha;

}
float Codec_Controller::Filter_Update(EMA_LPF * filter, float input){
	filter->filtered_signal = filter->alpha * input + (1-filter->alpha)*filter->filtered_signal;
	return filter->filtered_signal;
}
void Codec_Controller::LedControl(Codec_Typedef * codec){

	HAL_GPIO_TogglePin(codec->Codec_Reset_Pin_Port, codec->Codec_Reset_Pin);
	HAL_Delay(codec->Timeout);

}
using namespace LowPassFilter;
LPF<int order>::LPF(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.

      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }

      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }

      // Return the filtered value
      return y[0];
    }
    __STATIC_INLINE void DWT_Init(void)
    {
    	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
    	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
    }

    __STATIC_INLINE void delay_us(uint32_t us)
    {
    	uint32_t us_count_tic =  us * (SystemCoreClock / 1000000U);
    	DWT->CYCCNT = 0U;
    	while(DWT->CYCCNT < us_count_tic);
    }

    __STATIC_INLINE uint32_t micros(void){
    	return  DWT->CYCCNT / (SystemCoreClock / 1000000U);
    }
};
}


