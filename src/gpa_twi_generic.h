#ifndef GPA_TWI_GENERIC_H_
#define GPA_TWI_GENERIC_H_ 

#include "stdint.h"

typedef enum
{
	TWI_STATUS_DONE = 0,
	TWI_STATUS_BUS_ERROR,
	TWI_STATUS_ARB_LOSS,
	TWI_STATUS_NACK_REC,
	TWI_STATUS_ERROR
} twi_generic_status;


void twi_init(uint16_t speed);


void twi_send_read(uint8_t _addr, uint8_t* tx, uint8_t _txlen, uint8_t* rx, uint8_t _rxlen);


void twi_send(uint8_t _addr, uint8_t* tx, uint8_t _txlen);

void twi_read(uint8_t _addr, uint8_t* rx, uint8_t _rxlen);

twi_generic_status twi_status();

uint8_t twi_ready();

#endif



