/*
 * CS42448.hpp
 *
 *  Created on: Mar 23, 2024
 *      Author: Yunus TORUN
 */


#include <inttypes.h>
#include "stm32f3xx_hal.h"
#include "main.h"
#ifndef CS42448_HPP_
#define CS42448_HPP_

namespace Codec_Controller{


typedef struct{
	I2C_HandleTypeDef i2c_port;
	uint8_t CHIP_ADDRESS;
	uint16_t Timeout;
	uint8_t Trials;
	GPIO_TypeDef* Codec_Reset_Pin_Port;
	uint16_t Codec_Reset_Pin;



}Codec_Typedef;
typedef struct {
	float alpha;
	float filtered_signal;

}EMA_LPF;

HAL_StatusTypeDef Codec_IsReady(Codec_Typedef * codec);
HAL_StatusTypeDef PowerDownEnable(Codec_Typedef * codec);
HAL_StatusTypeDef PowerDownDisable(Codec_Typedef * codec);
HAL_StatusTypeDef MuteAdc(Codec_Typedef * codec);
HAL_StatusTypeDef HandleRegisters(Codec_Typedef * codec,uint8_t register_address);
HAL_StatusTypeDef SetI2SInterface(Codec_Typedef * codec);
HAL_StatusTypeDef SetADCMode(Codec_Typedef * codec);
HAL_StatusTypeDef InVolumeControl(Codec_Typedef * codec, uint8_t in_ch_register_addr);
void Filter_Init(EMA_LPF * filter, float alpha);
void Filter_SetAlpha(EMA_LPF * filter, float alpha);
float Filter_Update(EMA_LPF * filter, float input);
void LedControl(Codec_Typedef * codec);
}
namespace LowPassFilter{
	template <int order> class LPF{
	private:
		float a[order];
		float b[order+1];
		float omega0;
		float dt;
		bool adapt;
		float tn1 = 0;
		float x[order+1]; // Raw values
		float y[order+1]; // Filtered values
		__STATIC_INLINE void DWT_Init(void);
		__STATIC_INLINE void delay_us(uint32_t us);
		__STATIC_INLINE uint32_t micros(void);

	public:
		LPF(float f0, float fs, bool adaptive);
		void setCoef();
		float filt(float xn);
	};


}



#endif /* CS42448_HPP_ */
