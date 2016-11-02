#ifndef GPA_TWI_XMEGA_H_
#define GPA_TWI_XMEGA_H_ 
#include <avr/io.h>
#include <avr/interrupt.h>

#define XMEGA_TWI_BAUD(F_TWI) (F_CPU/(2*F_TWI) - 5)

#define XMEGA_TWI TWIE

#define XMEGA_TWI_INTLVL TWI_MASTER_INTLVL_MED_gc



#endif