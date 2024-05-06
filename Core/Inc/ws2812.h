/*
 * ws2812.h
 *
 *  Created on: Apr 26, 2024
 *      Author: lior
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include "stm32f1xx_hal.h"
#include "stdbool.h"

#define WS2812_MSG_LENGTH 24	/* 24 bits per led*/

typedef struct{
	uint8_t r;
	uint8_t g;
	uint8_t b;
}WS2812_RGBTypeDef;

typedef struct{
	double h;  // 0-360
	double s;  // 0-1
	double v;  // 0-1
}WS2812_HSVTypeDef;

typedef struct{
	int led_num;
	TIM_HandleTypeDef *htim;
	uint32_t tim_channel;
	uint16_t *dma_buf;
	int dma_buf_size;
	uint32_t logical_one_ccr;
	uint32_t logical_zero_ccr;

}WS2812_HandleTypeDef;

////////////////////////////////////////////
///////// core functionality
////////////////////////////////////////////
HAL_StatusTypeDef ws2812_init(WS2812_HandleTypeDef *hpxl, TIM_HandleTypeDef *htim, uint32_t tim_channel, double timer_freq_hz,
		uint16_t dma_buf[], int dma_buf_size, int led_num);
HAL_StatusTypeDef ws2812_write(WS2812_HandleTypeDef *hpxl);
HAL_StatusTypeDef ws2812_reset(WS2812_HandleTypeDef *hpxl);
void ws2812_set_color_rgb(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, WS2812_RGBTypeDef color);
void ws2812_set_color_hsv(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, WS2812_HSVTypeDef hsv);

////////////////////////////////////////////
///////// effects
////////////////////////////////////////////
void ws2812_soft_start_hsv(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, WS2812_HSVTypeDef hsv,
		uint32_t set_time_ms);
void ws2812_rainbow(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, double rotation_time, int rotations);



#endif /* INC_WS2812_H_ */
