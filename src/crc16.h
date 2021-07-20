/*
  Name  : CRC-16 CCITT
  Poly  : 0x1021    x^16 + x^12 + x^5 + 1
  Init  : 0xFFFF
  Revert: false
  XorOut: 0x0000
  Check : 0x29B1 ("123456789")
  MaxLen: 4095 bytes (32767 bits) - îáíàðóæåíèå îäèíàðíûõ, äâîéíûõ, òðîéíûõ è âñåõ íå÷åòíûõ îøèáîê
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRC16_H
#define __CRC16_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

uint16_t crc_modbus( const unsigned char *input_str, int num_bytes );

#ifdef __cplusplus
}
#endif

#endif //__CRC16_H