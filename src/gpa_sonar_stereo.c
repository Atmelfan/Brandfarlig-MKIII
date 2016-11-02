#include "gpa_sonar_stereo.h"
#include <util/delay.h>

uint8_t sonar_timed_out = 1;

uint16_t t1, t2;

uint8_t rx1, rx2;

void sonar_init(){
	SONAR_PORT.DIRSET = 0xFF;//All ouputs
	PORTC.DIRSET = (1 << 5);

}

void sonar_gen_pulse(uint8_t ch){
	cli();
	ch = 2*ch;
	SONAR_PORT.OUTSET = (0x01 << ch); 
	for (int i = 0; i < SONAR_PULSE_COUNT*2; ++i)
	{ 
		_delay_us(12.5);
		SONAR_PORT.OUTTGL = (0x03 << ch);
	}
	SONAR_PORT.OUTCLR = 0xFF;//All low
	sei();
}

void sonar_trigger(uint8_t ch){
	ACA.CTRLA = AC_AC0OUT_bm | AC_AC1OUT_bm;
	ACA.CTRLB = 31;
	ACA.AC0CTRL = AC_INTMODE_RISING_gc | AC_HYSMODE_LARGE_gc | AC_ENABLE_bm;
	ACA.AC1CTRL = AC_INTMODE_RISING_gc | AC_HYSMODE_LARGE_gc | AC_ENABLE_bm;
	ACA.AC0MUXCTRL = AC_MUXNEG_SCALER_gc | AC_MUXPOS_PIN0_gc;//E.g. select PIN0 for channel 0 or PIN2 for channel 1
	ACA.AC1MUXCTRL = AC_MUXNEG_SCALER_gc | AC_MUXPOS_PIN1_gc;//E.g. select PIN1 for channel 0 or PIN3 for channel 1
	//Start timer
	TCD0.CTRLA = TC_CLKSEL_OFF_gc;
	TCD0.CNT = 0;
	TCD0.PER = 10000;//15000cnt = 30ms
	TCD0.INTCTRLA |= TC_OVFINTLVL_HI_gc;
	TCD0.INTCTRLB |= TC_CCAINTLVL_HI_gc;
	TCD0.CCA = 1000;//Enable RX after 1.5ms
	TCD0.CTRLA = TC_CLKSEL_DIV64_gc;

	//Generate pulses...
	PORTC.OUTSET = (1 << 5);

	sonar_gen_pulse(ch);

}

/*Activates comparator*/
void sonar_enable_echo(){
	ACA.AC0CTRL |= AC_INTLVL_HI_gc;
	ACA.AC1CTRL |= AC_INTLVL_HI_gc;
	rx1=rx2=0;
	//ACA.STATUS &=~(AC_AC0IF_bm | AC_AC1IF_bm);


}

uint8_t sonar_valid(){
	return !sonar_timed_out;
}

/*Echo recieved, save time and disable further interrupts*/
ISR(ACA_AC0_vect){
	
	rx1++;
	if (rx1 >= 3)
	{
		ACA.AC0CTRL &= ~AC_INTLVL_HI_gc;
		PORTC.OUTTGL = (1 << 5);
		t1 = TCD0.CNT;
	}
	
} 

/*Echo recieved, save time and disable further interrupts*/
ISR(ACA_AC1_vect){
	//
	//
	rx2++;
	if (rx2 >= 3)
	{
		ACA.AC1CTRL &= ~AC_INTLVL_HI_gc;
		PORTC.OUTTGL = (1 << 5);
		t2 = TCD0.CNT;
	}
	
}

/*Activate comparators after a ringing has dissipated*/
ISR(TCD0_CCA_vect){
	sonar_enable_echo();
	//PORTC.OUTCLR = (1 << 5);
}

/*Timer has timed out*/
ISR(TCD0_OVF_vect){
 	sonar_timed_out = 1;
 	ACA.AC0CTRL &= ~AC_INTLVL_HI_gc;
 	ACA.AC1CTRL &= ~AC_INTLVL_HI_gc;
	PORTC.OUTCLR = (1 << 5);

}