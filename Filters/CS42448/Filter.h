/*
 * Filter.h
 *
 *  Created on: Mar 28, 2024
 *      Author: Yunus TORUN
 */
#include "arm_math.h"
#ifndef FILTER_H_
#define FILTER_H_
typedef struct{
	arm_fir_instance_f32 instance;
	float32_t * coefficients;
	uint16_t buffer_size;
	int16_t * raw_input_buffer_ptr;
	float32_t * filtered_output_buffer_ptr;
	uint16_t num_taps;
}Filter_Typedef;

void Filter_Init(Filter_Typedef * filter);
#endif /* FILTER_H_ */
