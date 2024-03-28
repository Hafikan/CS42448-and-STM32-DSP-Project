/*
 * Filter.c
 *
 *  Created on: Mar 28, 2024
 *      Author: Yunus TORUN
 */
#include <inttypes.h>
#include "Filter.h"
#include "arm_math.h"
#define BLOCK_SIZE 32
void Filter_Init(Filter_Typedef * filter){
	float32_t fir_state[BLOCK_SIZE+filter->num_taps-1];
	uint16_t num_blocks = filter->buffer_size / BLOCK_SIZE;
	float32_t filter_input_buffer[filter->buffer_size];
	float32_t filter_coeffs[filter->num_taps];
	for(int i = 0;i<filter->buffer_size;i++){
		filter_input_buffer[i] = (float32_t) filter->raw_input_buffer_ptr[i];
	}
	for(int i = 0;i<filter->num_taps;i++){
		filter_coeffs[i] =  filter->coefficients[i];
	}

	float32_t *filter_in_ptr = &filter_input_buffer[0];
	uint16_t block_size= BLOCK_SIZE;
	arm_fir_init_f32(&filter->instance, filter->num_taps, (float32_t *)&filter_coeffs[0], &fir_state[0], block_size);
	for(int i =0;i<num_blocks;i++){
		arm_fir_f32(&filter->instance,filter_in_ptr+(i*block_size), filter->filtered_output_buffer_ptr+(i*block_size),block_size);
	}

}
