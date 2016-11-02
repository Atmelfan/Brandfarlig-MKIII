#include "gpa_rc5.h"
#include <avr/io.h>
#include <util/delay.h>
uint16_t cmd;
uint8_t index;
rc5_state cstate;

void rc5_reset(){
	cmd = 0x0000;
	index = 0;
	cstate = RC5_STATE_START;
	
}

void rc5_init(){
	rc5_reset();	
}

uint8_t rc5_update(uint16_t d, uint8_t pulse){
	
	rc5_on_recieve2(cmd);

	if (index >= 14)
	{
		rc5_on_recieve(cmd);
		rc5_reset();
		return 0;
	}

	if(cstate == RC5_STATE_START){//START
		cmd |= 0x01;
		index++;
		cstate = RC5_STATE_LO;
		return 1;
	}

	// if (d < RC5_SHORT_MINPULSE || d > RC5_LONG_MAXPULSE)
	// {
	// 	rc5_reset();
	// 	//rc5_on_recieve(0x8000 + 0);
	// 	return 0;
	// }

	uint8_t shortt = (d < RC5_SHORT_MAXPULSE) ? 1 : 0;

	/*LO state*/
	if (cstate == RC5_STATE_LO)
	{
		if (pulse)
		{
			if (shortt){				//SHORT PULSE -> LO2
				cstate = RC5_STATE_LO2;
			}else{						//LONG PULSE  -> HI
				cstate = RC5_STATE_HI;
			}
		}else{
			//rc5_on_recieve(0x8000 + 1);//Space recieved during LO
			rc5_reset();
			return 0;
		}
		
	}

	else if (cstate == RC5_STATE_LO2)
	{
		if (shortt && !pulse){
			cstate = RC5_STATE_LO;
		}else{
			//rc5_on_recieve(0x8000 + 2);
			rc5_reset();
			return 0;
		}
	}

	else if (cstate == RC5_STATE_HI)
	{
		if (!pulse)
		{
			if (shortt){
				cstate = RC5_STATE_HI2;
			}else{
				cstate = RC5_STATE_LO;
			}
		}else{
			//rc5_on_recieve(0x8000 + 3);
			rc5_reset();
			return 0;
		}
		
	}

	else if (cstate == RC5_STATE_HI2)
	{
		if (shortt && pulse)
		{
			cstate = RC5_STATE_HI;
		}else{
			//rc5_on_recieve(0x8000 + 4);
			rc5_reset();
			return 0;
		}
		
	}else{
		//rc5_on_recieve(0x8000 + 5);
		rc5_reset();
		return 0;
	}

	if (cstate == RC5_STATE_HI)
	{
		cmd <<= 1;
		index++;
		//PORTC.OUTSET = (1 << 7);
		//_delay_ms(1);
		//PORTC.OUTCLR = (1 << 7);

	}else if (cstate == RC5_STATE_LO){
		cmd <<= 1;
		cmd |= 0x01;
		index++;
		//PORTC.OUTSET = (1 << 7);
		//_delay_ms(1);
		//PORTC.OUTCLR = (1 << 7);
		
	}

	if (index >= 14 && pulse)
	{
		rc5_on_recieve(cmd);
		rc5_reset();
		return 0;
	}

	return 1;

}





