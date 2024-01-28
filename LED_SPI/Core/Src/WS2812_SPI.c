/*
 * WS2812_SPI.c
 *
 *  Created on: Jan 27, 2024
 *      Author: jadem
 */


#include "main.h"
#include "WS2812_SPI.h"

#define MAX_LED 72
uint8_t LED_Data[MAX_LED][4];

extern SPI_HandleTypeDef hspi1;

void set_LED(int led_num, int Green, int Red, int Blue){
	LED_Data[led_num][0]=led_num;
	LED_Data[led_num][1]=Green;
	LED_Data[led_num][2]=Red;
	LED_Data[led_num][3]=Blue;
}

void ws2812_spi (int Green, int Red, int Blue){
	uint32_t color = Green<<16 | Red<<8 | Blue;
	uint8_t sendData[24];

	int indx = 0;

	for(int i=23;i>=0;i--){
		if(((color>>i)&0x01)==1) sendData[indx++]=0b110; // to store 1
		else sendData[indx++]=0b100; // store 0
	}

	HAL_SPI_Transmit(&hspi1, sendData, 24, 1000);
}

void WS2812_Send(void){
	for(int i=0;i<MAX_LED;i++){
		ws2812_spi (LED_Data[i][1], LED_Data[i][2], LED_Data[i][3]);
	}
	HAL_Delay(1);
}