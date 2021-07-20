/*
 * radio.c
 *
 *  Created on: 31 Oct 2016
 *      Author: Steve Chang
 */
#include <stm32f10x.h>
#include <stdio.h>

#include "radio.h"
#include "usart.h"

#include "freertos/include/FreeRTOS.h"
#include "freertos/include/task.h"
#include "freertos/include/semphr.h"

static SemaphoreHandle_t radio3Semaphore;

void Radio3_Init()
{
	radio3Semaphore = xSemaphoreCreateBinary();

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; /* High : idle, Low : busy */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //內部拉高
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//把EXTI1連到PB1

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

	//設定EXTI1

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//設定NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); /* 4 bits preemption */

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI1_IRQHandler(void) 
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1));
			xSemaphoreGiveFromISR(radio3Semaphore, NULL);
	}
	EXTI_ClearITPendingBit(EXTI_Line1); //清除等待位元
}

bool Radio3_IsBusy()
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) ? false : true;
}

void Radio3_SendRaw(uint8_t *raw, size_t len)
{
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0) {
//UsbUart_Printf("Drop tx packet ...\r\n");
		return;
	}
	Usart3_Write((uint8_t *)raw, len);
}
/*
void Radio3_SendPacket(UartPacket *p)
{
    if(p)
    	Radio3_SendRaw(p->raw, p->size);
}
*/
void Radio3_Handler(RadioRx rxCb)
{
	static uint8_t cache[MAX_RX_LEN];
	static size_t cache_size = 0;

	if(xSemaphoreTake(radio3Semaphore, 1 / portTICK_RATE_MS) != pdPASS)
		return;

	int len = Usart3_Poll();
	if(len <= 0)
		return;

	if(cache_size + len > MAX_RX_LEN)
		len = MAX_RX_LEN - cache_size;

	Usart3_Read(cache, len);

	rxCb(cache, len);
}
