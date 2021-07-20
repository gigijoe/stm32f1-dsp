/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

static uint8_t          crc_tab16_init          = 0;
static uint16_t         crc_tab16[256];

#define   CRC_POLY_16   0xA001

/*
 * static void init_crc16_tab( void );
 *
 * For optimal performance uses the CRC16 routine a lookup table with values
 * that can be used directly in the XOR arithmetic in the algorithm. This
 * lookup table is calculated by the init_crc16_tab() routine, the first time
 * the CRC function is called.
 */

static void init_crc16_tab( void ) {

  uint16_t i;
  uint16_t j;
  uint16_t crc;
  uint16_t c;

  for (i=0; i<256; i++) {

    crc = 0;
    c   = i;

    for (j=0; j<8; j++) {

      if ( (crc ^ c) & 0x0001 ) crc = ( crc >> 1 ) ^ CRC_POLY_16;
      else                      crc =   crc >> 1;

      c = c >> 1;
    }

    crc_tab16[i] = crc;
  }

  crc_tab16_init = 1;

}  /* init_crc16_tab */

#define   CRC_START_MODBUS  0xFFFF

uint16_t crc_modbus( const unsigned char *input_str, int num_bytes ) {

  uint16_t crc;
  const unsigned char *ptr;
  int a;

  if ( ! crc_tab16_init ) init_crc16_tab();

  crc = CRC_START_MODBUS;
  ptr = input_str;

  if ( ptr != 0 ) for (a=0; a<num_bytes; a++) {

    crc = (crc >> 8) ^ crc_tab16[ (crc ^ (uint16_t) *ptr++) & 0x00FF ];
  }

  return crc;

}  /* crc_modbus */