/*
 * WS2812.c
 *
 *  Created on: Apr 26, 2024
 *      Author: lior
 */

#include "ws2812.h"
#include "stm32f1xx_hal.h"
#include "math.h"

#define LOGICAL_ONE_HIGH_PULSE_WIDTH_NS 	800
#define LOGICAL_ZERO_HIGH_PULSE_WIDTH_NS 	400
#define MSG_BIT_TIME_ELAPSED_NS				1250
#define MSG_MAX_TIMING_ERROR_NS				150

HAL_StatusTypeDef ws_2812_init(WS2812_HandleTypeDef *hpxl, TIM_HandleTypeDef *htim, int led_num,
		uint32_t tim_channel, uint16_t dma_buf[], int dma_buf_size, double timer_freq_hz){

	double _one_ccr_value = 0;
	double _zero_ccr_value = 0;
	double _one_timing_error = 0;
	double _zero_timing_error = 0;
	double _msg_timing_error = 0;
	double counter_tick_period_ns = (1 / timer_freq_hz) * 1e9;
	double timer_ARR = (double)htim->Instance->ARR;
	/**
	 * calculate CCR values for logical 1 and logical 0 on ws2812 communication protocol
	 */
	_zero_ccr_value = round(LOGICAL_ZERO_HIGH_PULSE_WIDTH_NS / counter_tick_period_ns);
	_one_ccr_value =  round(LOGICAL_ONE_HIGH_PULSE_WIDTH_NS / counter_tick_period_ns);

	/**
	 * make sure timing tolerances stand up to WS2812 requirements
	 */
	_zero_timing_error = fabs(LOGICAL_ZERO_HIGH_PULSE_WIDTH_NS - (_zero_ccr_value * counter_tick_period_ns));
	_one_timing_error =  fabs(LOGICAL_ONE_HIGH_PULSE_WIDTH_NS - (_one_ccr_value * counter_tick_period_ns));
	_msg_timing_error =  fabs(MSG_BIT_TIME_ELAPSED_NS - timer_ARR * counter_tick_period_ns);

	if(_zero_timing_error > MSG_MAX_TIMING_ERROR_NS ||
			_one_timing_error > MSG_MAX_TIMING_ERROR_NS ||
			_msg_timing_error > MSG_MAX_TIMING_ERROR_NS){
		// timing error is too big. go back and redefine timer to stand up to tolerance requirements
		return false;
	}

	/**
	 * set metadata for led strip
	 */
	hpxl->led_num = led_num;
	hpxl->htim = htim;
	hpxl->tim_channel = tim_channel;
	hpxl->dma_buf = dma_buf;
	hpxl->dma_buf_size = dma_buf_size;
	hpxl->logical_zero_ccr = _zero_ccr_value;
	hpxl->logical_one_ccr = _one_ccr_value;

	// if we got here it means all went well
	return true;
}

/**
 * @brief This function updates the DMA buffer with the required led colors. the functions goes over the entire
 * 		  LED array and changes the colors of the required LEDS
 *
 * @param led_buf - this is and int array that contains the leds we want to set the color to
 * @param led_buf_size - this is the size of the led_buf
 * @param dma_buf[] - this is a pointer to the dma_buffer
 * @param dma_buf_size - this is the size of the DMA buffer
 */
void ws2812_set_color_rgb(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, uint16_t dma_buf[],
		int dma_buf_size, WS2812_RGBTypeDef color){
	uint32_t packet = 0;
	uint32_t mask = 0;
	uint32_t state = 0;
	int dma_index = 0;
	int led_index = 0;

	// create WS2812 message packet
	packet |= color.g << 16;
	packet |= color.r << 8;
	packet |= color.b;

	// convert message packet to DMA CCR values
	for(int i=0; i < led_buf_size; i++){
		for(int j=0; j < 24; j++)
		{
			// determine bit state
			mask = 1 << (23 - j);
			state = (packet & mask) >> (23 - j);
			led_index = led_buf[i];
			dma_index = (led_index * 24) + j +1;


			// set PWM CCR value to DMA buffer
			if(state == 0){
				hpxl->dma_buf[dma_index] = hpxl->logical_zero_ccr;
				dma_buf[dma_index] = hpxl->logical_zero_ccr;
			}
			else{
				dma_buf[dma_index] = hpxl->logical_one_ccr;
			}
		}
	}
	return;
}

HAL_StatusTypeDef ws2812_write(WS2812_HandleTypeDef *hpxl, uint16_t dma_buf[], int dma_buf_size){
	HAL_StatusTypeDef ret = HAL_ERROR;

	// send data via PWM_DMA functionality
	dma_buf[0] = 0;
	dma_buf[dma_buf_size-1] = 0;	// set 0 PWM duty cycle to reset transmission
	ret = HAL_TIM_PWM_Start_DMA(hpxl->htim, hpxl->tim_channel, (uint32_t *)dma_buf, dma_buf_size);
	HAL_Delay(1);
	HAL_TIM_PWM_Stop_DMA(hpxl->htim, hpxl->tim_channel);

	return ret;
}

HAL_StatusTypeDef ws2812_reset(WS2812_HandleTypeDef *hpxl, uint16_t dma_buf[], int dma_buf_size){
	HAL_StatusTypeDef ret = HAL_ERROR;

	// send logical zeros to DMA. this will set an RGB value of 0 to all leds
	for(int i=0; i<dma_buf_size-1; i++){
		dma_buf[i] = hpxl->logical_zero_ccr;
	}

	// send dma buffer data to LEDS
	ret = ws2812_write(hpxl, dma_buf, dma_buf_size);

	return ret;

}

void ws2812_set_color_hsv(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, WS2812_HSVTypeDef hsv){
	WS2812_RGBTypeDef rgb = {0,0,0};
	double R = 0;
	double G = 0;
	double B = 0;

	double C = hsv.v * hsv.s;
	double X = C * (1- fabs(((hsv.h / 60) % 2)-1));
	double M = hsv.v - C;

	if(hsv.h > 360.0 || hsv.h < 0.0 || hsv.s < 0.0 || hsv.s > 1.0 || hsv.v < 0.0 || hsv.v > 1.0){
		return;
	}

	if(hsv.h >= 0 && hsv.h < 60){
		R = C;
		G = X;
		B = 0;
	}
	else if(hsv.h >= 60 && hsv.h < 120){
		R = X;
		G = C;
		B = 0;
	}
	else if(hsv.h >= 120 && hsv.h < 180){
		R = 0;
		G = C;
		B = X;
	}
	else if(hsv.h >= 180 && hsv.h < 240){
		R = 0;
		G = X;
		B = C;
	}
	else if(hsv.h >= 240 && hsv.h < 300){
		R = X;
		G = 0;
		B = C;
	}
	else if(hsv.h >= 300 && hsv.h < 360){
		R = C;
		G = 0;
		B = X;
	}

	rgb.r = (uint8_t)(255 * (R+M));
	rgb.g = (uint8_t)(255 * (G+M));
	rgb.b = (uint8_t)(255 * (B+M));

	ws2812_set_color_rgb(hpxl, led_buf, led_buf_size, color);

	return;
}


