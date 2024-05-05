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

	// allocated timer
	TIM_HandleTypeDef *htim;
	uint32_t tim_channel;

	// dma buffer
	uint32_t *dma_buf
	int dma_buf_size

	// CCR values
	uint32_t logical_one_ccr;
	uint32_t logical_zero_ccr;

}WS2812_HandleTypeDef;

HAL_StatusTypeDef ws_2812_init(WS2812_HandleTypeDef *hpxl, TIM_HandleTypeDef *htim, int led_num,
		uint32_t tim_channel, uint16_t dma_buf[], int dma_buf_size, double timer_freq_hz);
void ws2812_set_color_rgb(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, uint16_t dma_buf[],
		int dma_buf_size, WS2812_RGBTypeDef color);
HAL_StatusTypeDef ws2812_write(WS2812_HandleTypeDef *hpxl, uint16_t dma_buf[], int dma_buf_size);
HAL_StatusTypeDef ws2812_reset(WS2812_HandleTypeDef *hpxl, uint16_t dma_buf[], int dma_buf_size);

// not implemented functions
void ws2812_set_color_hsv(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size, uint16_t dma_buf[],
		int dma_buf_size, WS2812_RGBTypeDef color);
HAL_StatusTypeDef ws2812_pulse(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size,
		WS2812_RGBTypeDef color, uint8_t low, uint8_t high, int freq_hz);
HAL_StatusTypeDef ws2812_circular_rainbow(WS2812_HandleTypeDef *hpxl, int led_buf[], int led_buf_size,
		WS2812_RGBTypeDef color, uint8_t low, uint8_t high, int freq_hz);




#endif /* INC_WS2812_H_ */
