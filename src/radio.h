/*
 * radio.h
 *
 *  Created on: 31 Oct 2016
 *      Author: Steve Chang
 */
#ifndef _RADIO_H
#define _RADIO_H

#include "bool.h"

typedef void (*RadioRx)(uint8_t *data, size_t size);

void Radio3_Init();
void Radio3_SendRaw(uint8_t *raw, size_t len);
//void Radio3_SendPacket(UartPacket *p);
void Radio3_Handler(RadioRx rxCb);

bool Radio3_IsBusy();

#endif
