/*
 * led.h
 *
 *  Created on: 8 Oct 2019
 *      Author: Steve Chang
 */
#ifndef _LED_H
#define _LED_H

#include "bool.h"

void screen_0_refresh(uint8_t num0, uint8_t num1);
void screen_0_mmap(uint16_t *data);

void screen_1_refresh(uint8_t num0, uint8_t num1);
void screen_1_mmap(uint16_t *data);

void screen_display_longpan();

#endif
