/*
 * beeper.h
 *
 *  Created on: 25 Dec 2019
 *      Author: Steve Chang
 */
#ifndef _BEEPER_H_
#define _BEEPER_H_

#include "bool.h"

void Beeper_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Beeper_EnableRepeat(bool yesNo);
void Beeper_Stop();

void Beeper_Boot();
void Beeper_Arming();
void Beeper_Armed();
void Beeper_Sos();
void Beeper_Ready();
void Beeper_Search();
void Beeper_Setup();

void Beeper_Short();
void Beeper_TwoShort();

void Beeper_Handler();

#endif
