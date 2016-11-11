#include "gpa_sonar_stereo.h"
#include <util/delay.h>
#include <stdlib.h>

uint8_t sonar_timed_out = 1;

uint16_t t1, t2, ft1, ft2;

uint16_t rt1, rt2, lt1, lt2;

uint8_t rx1, rx2;

uint8_t cch = 0;

volatile sonar_state cstate;

void sonar_init(){
	SONAR_PORT.DIRSET = 0xFF;//All ouputs
	PORTC.DIRSET = (1 << 5);

}

void sonar_gen_pulse(uint8_t out){
	cli();
	SONAR_PORT.OUTSET = (0x01 << out); 
	for (int i = 0; i < SONAR_PULSE_COUNT*2; ++i)
	{ 
		_delay_us(12.5);
		SONAR_PORT.OUTTGL = (0x03 << out);
	} 
	SONAR_PORT.OUTCLR = 0xFF;//All low
	sei();
}

void sonar_start_timer(sonar_state s){
	//Start timer
	TCD0.CTRLA = TC_CLKSEL_OFF_gc;
	TCD0.CNT = 0;
	TCD0.PER = 10000;//15000cnt = 30ms
	TCD0.INTCTRLA |= TC_OVFINTLVL_HI_gc;
	TCD0.INTCTRLB |= TC_CCAINTLVL_HI_gc;

	if(s == SONAR_MEAS_RIGHT){
		TCD0.CCA = 750;//Enable RX after 1.5ms
		TCD0.CCB = 500;//Enable RX after 1.5ms
	}else{
		TCD0.CCA = 750;//Enable RX after 1.5ms
		TCD0.CCB = 500;//Enable RX after 1.5ms
	}
	TCD0.CTRLA = TC_CLKSEL_DIV64_gc; 
	ft1=ft2=0;
	t1=t2=0;
}

void sonar_trigger(uint8_t ch){
	if (cstate != SONAR_READY){
		return;
	}
	cstate = SONAR_MEAS_RIGHT;

	//Configure comparators
	ACA.CTRLA = AC_AC0OUT_bm | AC_AC1OUT_bm;
	ACA.CTRLB = 31;
	ACA.AC0CTRL = AC_INTMODE_RISING_gc | AC_HYSMODE_LARGE_gc | AC_ENABLE_bm;
	ACA.AC1CTRL = AC_INTMODE_RISING_gc | AC_HYSMODE_LARGE_gc | AC_ENABLE_bm;
	ACA.AC0MUXCTRL = AC_MUXNEG_SCALER_gc | AC_MUXPOS_PIN0_gc;//E.g. select PIN0 for channel 0 or PIN2 for channel 1
	ACA.AC1MUXCTRL = AC_MUXNEG_SCALER_gc | AC_MUXPOS_PIN1_gc;//E.g. select PIN1 for channel 0 or PIN3 for channel 1
	
	//Start timer
	sonar_start_timer(cstate);

	//Generate pulses...
	PORTC.OUTCLR = (1 << 5);

	cch = ch;
	sonar_gen_pulse(cch*2);

}


sonar_channel_data sonar_data[SONAR_NUM_CHANNELS];

uint8_t sonar_current_channel=0, sonar_freerunning = 0;

void sonar_start(){
	uint8_t in = sonar_current_channel/2;
	ACA.CTRLA = AC_AC0OUT_bm | AC_AC1OUT_bm; 
	ACA.CTRLB = 31;
	ACA.AC0CTRL = AC_INTMODE_RISING_gc | AC_HYSMODE_LARGE_gc | AC_ENABLE_bm;
	ACA.AC1CTRL = AC_INTMODE_RISING_gc | AC_HYSMODE_LARGE_gc | AC_ENABLE_bm;
	ACA.AC0MUXCTRL = AC_MUXNEG_SCALER_gc | ((in*2 + 0) << 3);//E.g. select PIN0 for channel 0 or PIN2 for channel 1
	ACA.AC1MUXCTRL = AC_MUXNEG_SCALER_gc | ((in*2 + 1) << 3);//E.g. select PIN1 for channel 0 or PIN3 for channel 1	

	TCD0.CTRLA = TC_CLKSEL_OFF_gc;
	TCD0.CNT = 0;
	TCD0.PER = 20000;
	TCD0.INTCTRLA |= TC_OVFINTLVL_HI_gc; 
	TCD0.INTCTRLB |= TC_CCAINTLVL_HI_gc|TC_CCBINTLVL_HI_gc;
	if(sonar_current_channel == 1 || sonar_current_channel == 2){
		TCD0.CCA = 1000;//Enable RX after 1.5ms
		TCD0.CCB = 1000;//Enable RX after 1.0ms
	}else{
		TCD0.CCA = 1000;//Enable RX after 1.5ms
		TCD0.CCB = 1000;//Enable RX after 1.0ms
	}
	TCD0.CTRLA = TC_CLKSEL_DIV64_gc;
	PORTC.OUTSET = (1 << 5);
	rx1=rx2=ft1=ft2=0;
}

void sonar_tx(){
	uint8_t out = sonar_current_channel*2;
	cli();
	SONAR_PORT.OUTSET = (0x01 << out); 
	for (int i = 0; i < SONAR_PULSE_COUNT*2; ++i)
	{ 
		_delay_us(12.5);
		SONAR_PORT.OUTTGL = (0x03 << out);
	}
	SONAR_PORT.OUTCLR = 0xFF;//All low
	sei();
}

//Called after ~0.5-1.5ms 
void sonar_rx1_enable(){
	//PORTC.OUTTGL = (1 << 5);
	ACA.AC0CTRL |= AC_INTLVL_HI_gc;
}

//Called after ~0.5-1.5ms 
void sonar_rx2_enable(){
	ACA.AC1CTRL |= AC_INTLVL_HI_gc;
}

//Called between rx1_enable and sonar_end 
void sonar_rx1(){
	uint16_t t = TCD0.CNT;
	if (t-ft1 < 30){
		rx1++;
	}else{
		ft1=t;
		rx1=1;
	}
	
	if(rx1 >= 3){			/*Multiple consequative pulses, wave recieved!*/
		ACA.AC0CTRL &= ~AC_INTLVL_HI_gc;
		PORTC.OUTCLR = (1 << 5);
		ft1=t;
	}
}

//Called between rx2_enable and sonar_end 
void sonar_rx2(){
	uint16_t t = TCD0.CNT;
	if (t-ft2 < 30){
		rx2++;
	}else{
		ft2=t;
		rx2=1;
	}
	if(rx2 >= 3){			/*Multiple consequative pulses, wave recieved!*/
		ACA.AC1CTRL &= ~AC_INTLVL_HI_gc;
		PORTC.OUTCLR = (1 << 5);
		ft2=t;
	}
}

void sonar_rx_disable(){
	ACA.AC0CTRL &= ~AC_INTLVL_HI_gc;
 	ACA.AC1CTRL &= ~AC_INTLVL_HI_gc;
}

//Called'after ~32ms
void sonar_end(){
	PORTC.OUTCLR = (1 << 5);
	sonar_rx_disable();
	if(sonar_freerunning){
		sonar_data[sonar_current_channel].r_range = (rx1 >= 3) ? ft1 : 0;
		sonar_data[sonar_current_channel].l_range = (rx2 >= 3) ? ft2 : 0;

		sonar_current_channel++;
		if (sonar_current_channel >= SONAR_NUM_CHANNELS){
			sonar_current_channel=0;
		}
		sonar_start();
		sonar_tx();
	}
}

/**
 * Initiate measurement on channel $ch. If freerun == 1, it will automatically loop through all channels
 */
void sonar_trigger_free(uint8_t ch, uint8_t freerun){
	sonar_current_channel = ch;
	sonar_freerunning = freerun;
	sonar_start();
	sonar_tx();
}

/**
 * Initiate measurement on channel $ch
 */
uint8_t sonar_status(uint8_t ch){
	return 0;
}

/**
 * Returns 1 if range is available
 */
uint8_t sonar_has_range(uint8_t ch){
	return 0;

}

/**
 * Returns 1 if direction is available
 */
uint8_t sonar_has_direction(uint8_t ch){
	return 0;

}

/**
 * Calculates average range (if possible) or closest range  
 */
uint16_t sonar_get_range(uint8_t ch){
	int16_t dt = sonar_data[ch].r_range - sonar_data[ch].l_range;
	if(abs(dt) < SONAR_DT_MAX){
		return (sonar_data[ch].l_range + sonar_data[ch].r_range)/2;
	}else{
		return sonar_data[ch].l_range < sonar_data[ch].r_range ? sonar_data[ch].l_range : sonar_data[ch].r_range;
	}
	 

}

/**
 * Calculates average direction or 0 if not possible   
 */
int16_t sonar_get_direction(uint8_t ch){
	int16_t dt = sonar_data[ch].r_range - sonar_data[ch].l_range;
	return abs(dt) < SONAR_DT_MAX ? dt : 0;
}


/*Right echo recieved, save time and disable further comparator interrupts*/
ISR(ACA_AC0_vect){
	sonar_rx1();
	
} 

/*Left echo recieved, save time and disable further comparator interrupts*/
ISR(ACA_AC1_vect){
	sonar_rx2();
}

/*Activate right comparator after ringing has dissipated*/
ISR(TCD0_CCA_vect){
	sonar_rx1_enable();
}

/*Activate left comparator after ringing has dissipated*/
ISR(TCD0_CCB_vect){
	sonar_rx2_enable();
	//PORTC.OUTCLR = (1 << 5);
}

/*Timer has timed out*/
ISR(TCD0_OVF_vect){
	sonar_end();
}