#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "gpa_twi_generic.h"
#include "gpa_twi_xmega.h"


volatile uint8_t* txbuf;
volatile uint8_t txlen;

volatile uint8_t* rxbuf;
volatile uint8_t rxlen;

volatile uint8_t addr;

volatile uint8_t ready = 1;
 
volatile twi_generic_status status;


void twi_init(uint16_t speed){
	TWIE.MASTER.BAUD = speed;
 
	TWIE.MASTER.CTRLA = XMEGA_TWI_INTLVL 
		| TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_ENABLE_bm;
	
	TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
}

 
void twi_send_read(uint8_t _addr, uint8_t* tx, uint8_t _txlen, uint8_t* rx, uint8_t _rxlen){
	while(!ready);

	txbuf = tx; txlen = _txlen;
	rxbuf = rx; rxlen = _rxlen;
	addr = _addr << 1;
	ready = 0;

	if(txlen > 0){
		XMEGA_TWI.MASTER.ADDR = addr & 0xFE;
	}else if(rxlen > 0){
		XMEGA_TWI.MASTER.ADDR = addr | 0x01;
	}else{
		/*Nothing to transmit/recieve apparently?*/
		ready = 1;
	}
}

void twi_send(uint8_t _addr, uint8_t* tx, uint8_t _txlen){
	twi_send_read(_addr, tx, _txlen, NULL, 0);
}

void twi_read(uint8_t _addr, uint8_t* rx, uint8_t _rxlen){
	twi_send_read(_addr, NULL, 0, rx, _rxlen); 
}

twi_generic_status twi_status(){
	return status;
}

uint8_t twi_ready(){
	return ready;
}

ISR(TWIE_TWIM_vect)
{
	if(XMEGA_TWI.MASTER.STATUS & TWI_MASTER_ARBLOST_bm){
		/*Arbitrarion lost interrupt*/
		XMEGA_TWI.MASTER.STATUS |= TWI_MASTER_ARBLOST_bm;
		status = TWI_STATUS_ARB_LOSS;
		ready = 1;
	}else if(XMEGA_TWI.MASTER.STATUS & TWI_MASTER_BUSERR_bm){
		/*Bus error interrupt*/
		XMEGA_TWI.MASTER.STATUS |= TWI_MASTER_BUSERR_bm;
		status = TWI_STATUS_BUS_ERROR;
		ready = 1;
	}else if (XMEGA_TWI.MASTER.STATUS & TWI_MASTER_WIF_bm) {
		/*Write complete interrupt*/
		if(XMEGA_TWI.MASTER.STATUS & TWI_MASTER_RXACK_bm)
		{
			/*NACk recieved, abort*/
			XMEGA_TWI.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
			status = TWI_STATUS_NACK_REC;
			ready = 1;
		}else if(txlen > 0){ 
			/*Still bytes to write*/
			XMEGA_TWI.MASTER.DATA = *txbuf;
			txbuf++;
			txlen--;
		}else if(rxlen > 0){
			XMEGA_TWI.MASTER.ADDR = addr | 0x01;
		}else{
			/*All bytes have been written and nothing to transmit, STOP*/
			XMEGA_TWI.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
			status = TWI_STATUS_DONE;
			ready = 1;
		}
	}else if (XMEGA_TWI.MASTER.STATUS & TWI_MASTER_RIF_bm) {
		/*Read complete interrupt*/
		if(rxlen > 0)
		{
			/*Still bytes to read, issue ACK*/
			*rxbuf = XMEGA_TWI.MASTER.DATA;
			XMEGA_TWI.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
			rxbuf++;
			rxlen--;
		}else{
			/*All bytes have been read, issue NACK and STOP*/
			XMEGA_TWI.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
			status = TWI_STATUS_DONE;
			ready = 1;
		}
	}else{
		/*WITF did just happen here?*/
		status = TWI_STATUS_ERROR;
		ready = 1;
	}

}

