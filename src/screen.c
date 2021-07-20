/*
 * led.c
 *
 *  Created on: 8 Oct 2019
 *      Author: Steve Chang
 */
#include <stm32f10x.h>
#include <stdio.h>
#include <string.h>

#include "freertos/include/FreeRTOS.h"
#include "freertos/include/task.h"
#include "freertos/include/semphr.h"

#include "screen.h"

static uint8_t rom8x16_bits[] = {
/* Character 0 (0x30):
   ht=16, width=8
   +--------+
   |        |
   | *****  |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   | *****  |
   |        |
   |        |
   |        |
   +--------+ */
0x00,
0x7c,
0xc6,
0xc6,
0xc6,
0xc6,
0xc6,
0xc6,
0xc6,
0xc6,
0xc6,
0xc6,
0x7c,
0x00,
0x00,
0x00,

/* Character 1 (0x31):
   ht=16, width=8
   +--------+
   |        |
   |   **   |
   | ****   |
   |   **   |
   |   **   |
   |   **   |
   |   **   |
   |   **   |
   |   **   |
   |   **   |
   |   **   |
   |   **   |
   | ****** |
   |        |
   |        |
   |        |
   +--------+ */
0x00,
0x18,
0x78,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x7e,
0x00,
0x00,
0x00,

/* Character 2 (0x32):
   ht=16, width=8
   +--------+
   |        |
   | *****  |
   |**   ** |
   |**   ** |
   |**   ** |
   |     ** |
   |    **  |
   |   **   |
   |  **    |
   | **     |
   |**      |
   |**   ** |
   |******* |
   |        |
   |        |
   |        |
   +--------+ */
0x00,
0x7c,
0xc6,
0xc6,
0xc6,
0x06,
0x0c,
0x18,
0x30,
0x60,
0xc0,
0xc6,
0xfe,
0x00,
0x00,
0x00,

/* Character 3 (0x33):
   ht=16, width=8
   +--------+
   |        |
   | *****  |
   |**   ** |
   |**   ** |
   |     ** |
   |     ** |
   |  ****  |
   |     ** |
   |     ** |
   |     ** |
   |     ** |
   |**   ** |
   | *****  |
   |        |
   |        |
   |        |
   +--------+ */
0x00,
0x7c,
0xc6,
0xc6,
0x06,
0x06,
0x3c,
0x06,
0x06,
0x06,
0x06,
0xc6,
0x7c,
0x00,
0x00,
0x00,

/* Character 4 (0x34):
   ht=16, width=8
   +--------+
   |        |
   |    **  |
   |   ***  |
   |  ****  |
   | ** **  |
   |**  **  |
   |**  **  |
   |******* |
   |******* |
   |    **  |
   |    **  |
   |    **  |
   |   **** |
   |        |
   |        |
   |        |
   +--------+ */
0x00,
0x0c,
0x1c,
0x3c,
0x6c,
0xcc,
0xcc,
0xfe,
0xfe,
0x0c,
0x0c,
0x0c,
0x1e,
0x00,
0x00,
0x00,

/* Character 5 (0x35):
   ht=16, width=8
   +--------+
   |        |
   |******* |
   |**      |
   |**      |
   |**      |
   |**      |
   |******  |
   |     ** |
   |     ** |
   |     ** |
   |     ** |
   |**   ** |
   | *****  |
   |        |
   |        |
   |        |
   +--------+ */
0x00,
0xfe,
0xc0,
0xc0,
0xc0,
0xc0,
0xfc,
0x06,
0x06,
0x06,
0x06,
0xc6,
0x7c,
0x00,
0x00,
0x00,

/* Character 6 (0x36):
   ht=16, width=8
   +--------+
   |        |
   |        |
   | *****  |
   |**   ** |
   |**      |
   |**      |
   |**      |
   |******  |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   | *****  |
   |        |
   |        |
   |        |
   +--------+ */
0x00,
0x7c,
0xc6,
0xc0,
0xc0,
0xc0,
0xfc,
0xc6,
0xc6,
0xc6,
0xc6,
0xc6,
0x7c,
0x00,
0x00,
0x00,

/* Character 7 (0x37):
   ht=16, width=8
   +--------+
   |        |
   |******* |
   |**   ** |
   |     ** |
   |     ** |
   |    **  |
   |   **   |
   |  **    |
   |  **    |
   |  **    |
   |  **    |
   |  **    |
   |  **    |
   |        |
   |        |
   |        |
   +--------+ */
0x00,
0xfe,
0xc6,
0x06,
0x06,
0x0c,
0x18,
0x30,
0x30,
0x30,
0x30,
0x30,
0x30,
0x00,
0x00,
0x00,

/* Character 8 (0x38):
   ht=16, width=8
   +--------+
   |        |
   | *****  |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   | *****  |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   | *****  |
   |        |
   |        |
   |        |
   +--------+ */
0x00,
0x7c,
0xc6,
0xc6,
0xc6,
0xc6,
0x7c,
0xc6,
0xc6,
0xc6,
0xc6,
0xc6,
0x7c,
0x00,
0x00,
0x00,

/* Character 9 (0x39):
   ht=16, width=8
   +--------+
   |        |
   | *****  |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   |**   ** |
   | ****** |
   |     ** |
   |     ** |
   |     ** |
   |**   ** |
   | *****  |
   |        |
   |        |
   |        |
   +--------+ */
0x00,
0x7c,
0xc6,
0xc6,
0xc6,
0xc6,
0xc6,
0x7e,
0x06,
0x06,
0x06,
0xc6,
0x7c,
0x00,
0x00,
0x00,
};

/*
 * Longpan 
 */ 

static uint16_t long_bits[] = {
0x4010,
0x4408,
0x7e7f,
0x4022,
0x7c14,
0x047f,
0x7c00,
0x403e,
0x7c22,
0x403e,
0x7c22,
0x403e,
0x7e22,
0x422e,
0x3e24,
0x0000
};

static uint16_t pan_bits[] = {
0x7808,
0x483e,
0x482a,
0x8e2a,
0xf87e,
0x4822,
0x302a,
0x4826,
0x8442,
0xfe7f,
0x0004,
0xf80f,
0x0818,
0xf86f,
0x0808,
0x0000
};

/*
 * 
 */
  
static uint8_t screen_buffer_0[32];
static uint8_t screen_buffer_1[32];

void delay_tick(uint32_t c)
{
  volatile unsigned int i = c;
  while(i--);
}

/*
 * LED screen 0
 */ 

static void led_0_msb_send(uint8_t data)
{
  uint8_t bit;
  for(bit=0;bit<8;bit++) {
    if(data & (1 << (7-bit))) /* MSB first */
      GPIO_SetBits(GPIOA, GPIO_Pin_4);
    else
      GPIO_ResetBits(GPIOA, GPIO_Pin_4);

    GPIO_SetBits(GPIOA, GPIO_Pin_3); /* Clock high */    
 	delay_tick(20);
    GPIO_ResetBits(GPIOA, GPIO_Pin_3); /* Clock low */
    GPIO_ResetBits(GPIOA, GPIO_Pin_4); /* Data low */
    delay_tick(20);
  }
}

static void led_0_lsb_send(uint8_t data)
{
  uint8_t bit;
  for(bit=0;bit<8;bit++) {
    if(data & (1 << bit)) /* LSB first */
      GPIO_SetBits(GPIOA, GPIO_Pin_4);
    else
      GPIO_ResetBits(GPIOA, GPIO_Pin_4);

    GPIO_SetBits(GPIOA, GPIO_Pin_3); /* Clock high */    
    delay_tick(20);
    GPIO_ResetBits(GPIOA, GPIO_Pin_3); /* Clock low */
    GPIO_ResetBits(GPIOA, GPIO_Pin_4); /* Data low */
    delay_tick(20);
  }
}

static void _screen_0_refresh(void)
{
  uint8_t addr;
  for(addr=0;addr<0x20;addr+=4) {
    led_0_msb_send(addr);
    led_0_lsb_send(screen_buffer_0[addr]);
    led_0_lsb_send(screen_buffer_0[addr+1]);
    led_0_lsb_send(screen_buffer_0[addr+2]);
    led_0_lsb_send(screen_buffer_0[addr+3]);
  }
  led_0_msb_send(0x98);
}

void screen_0_refresh(uint8_t num0, uint8_t num1)
{
  uint8_t i;
  for(i=0;i<16;i++) {
    uint8_t j=i<<1;
    if(num0 > 9)
      screen_buffer_0[j] = 0;
    else
      screen_buffer_0[j] = rom8x16_bits[num0*16+i]; /* */
      
    if(num1 > 9)
      screen_buffer_0[j+1] = 0;
    else
      screen_buffer_0[j+1] = rom8x16_bits[num1*16+i];
  }
  
  /* Add !point! */
  
   screen_buffer_0[27] |= 0x1;
   screen_buffer_0[29] |= 0x1;
  
  _screen_0_refresh();
}

void screen_0_mmap(uint16_t *data)
{
  memcpy(screen_buffer_0, data, 32);
  _screen_0_refresh();
}

/*
 * LED screen 1
 */ 

static void led_1_msb_send(uint8_t data)
{
  uint8_t bit;
  for(bit=0;bit<8;bit++) {
    if(data & (1 << (7-bit))) /* MSB first */
      GPIO_SetBits(GPIOB, GPIO_Pin_6);
    else
      GPIO_ResetBits(GPIOB, GPIO_Pin_6);

    GPIO_SetBits(GPIOB, GPIO_Pin_5); /* Clock high */    
    delay_tick(20);
    GPIO_ResetBits(GPIOB, GPIO_Pin_5); /* Clock low */
    GPIO_ResetBits(GPIOB, GPIO_Pin_6); /* Data low */
    delay_tick(20);
  }
}

static void led_1_lsb_send(uint8_t data)
{
  uint8_t bit;
  for(bit=0;bit<8;bit++) {
    if(data & (1 << bit)) /* LSB first */
      GPIO_SetBits(GPIOB, GPIO_Pin_6);
    else
      GPIO_ResetBits(GPIOB, GPIO_Pin_6);

    GPIO_SetBits(GPIOB, GPIO_Pin_5); /* Clock high */    
    delay_tick(20);
    GPIO_ResetBits(GPIOB, GPIO_Pin_5); /* Clock low */
    GPIO_ResetBits(GPIOB, GPIO_Pin_6); /* Data low */
    delay_tick(20);
  }
}

static void _screen_1_refresh(void)
{
  uint8_t addr;
  for(addr=0;addr<0x20;addr+=4) {
    led_1_msb_send(addr);
    led_1_lsb_send(screen_buffer_1[addr]);
    led_1_lsb_send(screen_buffer_1[addr+1]);
    led_1_lsb_send(screen_buffer_1[addr+2]);
    led_1_lsb_send(screen_buffer_1[addr+3]);
  }
  led_1_msb_send(0x98);
}

void screen_1_refresh(uint8_t num0, uint8_t num1)
{
  uint8_t i;
  for(i=0;i<16;i++) {
    uint8_t j=i<<1;
    if(num0 > 9)
      screen_buffer_1[j] = 0;
    else
      screen_buffer_1[j] = rom8x16_bits[num0*16+i]; /* */
      
    if(num1 > 9)
      screen_buffer_1[j+1] = 0;
    else
      screen_buffer_1[j+1] = rom8x16_bits[num1*16+i];
  }

  /* Add !point! */
  
   screen_buffer_1[26] |= 0x80;
   screen_buffer_1[28] |= 0x80;
  
  _screen_1_refresh();
}

void screen_1_mmap(uint16_t *data)
{
  memcpy(screen_buffer_1, data, 32);
  _screen_1_refresh();
}

/*
*
*/

void screen_display_longpan()
{
    screen_0_mmap(long_bits);
    screen_1_mmap(pan_bits);
}
