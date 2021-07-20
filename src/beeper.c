/*
 * beeper.h
 *
 *  Created on: 25 Dec 2019
 *      Author: Steve Chang
 */
#include "stm32f10x.h"
#include "beeper.h"

#include "freertos/include/FreeRTOS.h"
#include "freertos/include/task.h"
#include "freertos/include/queue.h"

#include "bool.h"
#include <string.h>

typedef struct {
	uint8_t tick;
	const uint8_t *sheet;
	size_t position;
	bool active;
	bool repeat;
} Beeper;

#define BEEPER_COMMAND_REPEAT 0xFE
#define BEEPER_COMMAND_STOP   0xFF

/* Beeper Sound Sequences: (Square wave generation)
 * Sequence must end with 0xFF or 0xFE. 0xFE repeats the sequence from
 * start when 0xFF stops the sound when it's completed.
 *
 * "Sound" Sequences are made so that 1st, 3rd, 5th.. are the delays how
 * long the beeper is on and 2nd, 4th, 6th.. are the delays how long beeper
 * is off. Delays are in milliseconds/10 (i.e., 5 => 50ms).
 */
// short fast beep
static const uint8_t _shortBeep[] = {
    10, 10, BEEPER_COMMAND_STOP
};
// short fast beep
static const uint8_t _bootBeep[] = {
    10, 10, 10, 10, BEEPER_COMMAND_STOP
};
// arming beep
static const uint8_t _armingBeep[] = {
    30, 5, 5, 5, BEEPER_COMMAND_STOP
};
// armed beep (first pause, then short beep)
static const uint8_t _armedBeep[] = {
    0, 245, 10, 5, BEEPER_COMMAND_STOP
};
// disarming beeps
static const uint8_t _disarmBeep[] = {
    15, 5, 15, 5, BEEPER_COMMAND_STOP
};
// beeps while stick held in disarm position (after pause)
static const uint8_t _disarmRepeatBeep[] = {
    0, 100, 10, BEEPER_COMMAND_STOP
};
// Long beep and pause after that
static const uint8_t _lowBatteryBeep[] = {
    25, 50, BEEPER_COMMAND_STOP
};
// critical battery beep
static const uint8_t _critBatteryBeep[] = {
    50, 2, BEEPER_COMMAND_STOP
};

// transmitter-signal-lost tone
static const uint8_t _txLostBeep[] = {
    50, 50, BEEPER_COMMAND_STOP
};
// SOS morse code:
static const uint8_t _sosBeep[] = {
    10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70, BEEPER_COMMAND_STOP
};
// Arming when GPS is fixed
static const uint8_t _armedGpsFix[] = {
    5, 5, 15, 5, 5, 5, 15, 30, BEEPER_COMMAND_STOP
};
// Ready beeps. When gps has fix and copter is ready to fly.
static const uint8_t _readyBeep[] = {
    4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5, BEEPER_COMMAND_STOP
};
// 2 fast short beeps
static const uint8_t _2shortBeeps[] = {
    5, 5, 5, 5, BEEPER_COMMAND_STOP
};
// 2 longer beeps
static const uint8_t _2longerBeeps[] = {
    20, 15, 35, 5, BEEPER_COMMAND_STOP
};
// 3 beeps
static const uint8_t _gyroCalibrated[] = {
    20, 10, 20, 10, 20, 10, BEEPER_COMMAND_STOP
};
// Search code
static const uint8_t _searchBeep[] = {
    10, 10, 10, 10, 10, 40, 40, 10, 80, 160, BEEPER_COMMAND_STOP
};
// setup beep
static const uint8_t _setupBeep[] = {
    40, 5, 5, 5, 5, 5, BEEPER_COMMAND_STOP
};

static Beeper s_beeper = {0};

static GPIO_TypeDef* BEEPER_PORT = 0;
static uint16_t BEEPER_PIN = 0;

static void Beeper_Play(const uint8_t *sheet)
{
	const uint8_t *currentSheet = s_beeper.sheet;

	s_beeper.sheet = sheet;

	if(currentSheet != s_beeper.sheet || 
		(currentSheet == s_beeper.sheet && s_beeper.active == false)) {
		s_beeper.tick = 0;
		s_beeper.position = 0;
		s_beeper.active = true;
		//s_beeper.repeat = repeat;
		if(s_beeper.sheet[0] != 0)
			GPIO_SetBits(BEEPER_PORT, BEEPER_PIN); /* Beeper on */
		else
			GPIO_ResetBits(BEEPER_PORT, BEEPER_PIN); /* Beeper off */
	}
}

void Beeper_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	BEEPER_PORT = GPIOx;
	BEEPER_PIN = GPIO_Pin;
	GPIO_ResetBits(BEEPER_PORT, BEEPER_PIN); /* Beeper off */
}

void Beeper_Stop()
{
	s_beeper.active = false;
	s_beeper.repeat = false;

	GPIO_ResetBits(BEEPER_PORT, BEEPER_PIN); /* Beeper off */
}

void Beeper_EnableRepeat(bool yesNo)
{
	s_beeper.repeat = yesNo;
}

void Beeper_Boot()
{
	Beeper_Play(_bootBeep);
}

void Beeper_Arming()
{
	Beeper_Play(_armingBeep);
}

void Beeper_Armed()
{
	Beeper_Play(_armedBeep);
}

void Beeper_Sos()
{
	Beeper_Play(_sosBeep);
}

void Beeper_Ready()
{
	Beeper_Play(_readyBeep);
}

void Beeper_Search()
{
	Beeper_Play(_searchBeep);
}

void Beeper_Setup()
{
	Beeper_Play(_setupBeep);
}

void Beeper_Short()
{
	Beeper_Play(_shortBeep);
}

void Beeper_TwoShort()
{
	Beeper_Play(_2shortBeeps);
}

void Beeper_Handler()
{
	if(s_beeper.sheet == 0)
		return;

	if(s_beeper.active == false)
		return;

	uint8_t period = s_beeper.sheet[s_beeper.position];

	if(++s_beeper.tick > period) {
		s_beeper.position++;
		s_beeper.tick = 0;
		if(s_beeper.sheet[s_beeper.position] == BEEPER_COMMAND_STOP) {
			GPIO_ResetBits(BEEPER_PORT, BEEPER_PIN); /* Beeper off */
			if(s_beeper.repeat) {
				s_beeper.tick = 0;
				s_beeper.position = 0;
			} else
				s_beeper.active = false;
			return;
		}
	}

	if(s_beeper.position % 2 == 0)
		GPIO_SetBits(BEEPER_PORT, BEEPER_PIN); /* Beeper on */
	else 
		GPIO_ResetBits(BEEPER_PORT, BEEPER_PIN); /* Beeper off */
}
