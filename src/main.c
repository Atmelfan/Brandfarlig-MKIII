#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "motors.h"
#include "gpa_adc_freerun.h"
#include "gpa_twi_generic.h"
#include "gpa_twi_xmega.h"
#include "gpa_pca9553.h"
#include "gpa_pca9534.h"
#include "gpa_rc5.h"
#include "gpa_start.h"
#include "gpa_sonar_stereo.h"
#include "vl53l0x.h"
#include <math.h>

volatile uint8_t index = 0;
volatile uint8_t tg = 0;
volatile uint8_t edge;
volatile uint16_t count;

volatile uint8_t ccw = 1;

volatile uint16_t edges[4];

void usart_send(char* s){
	while(*s){
		while( !(USARTC1.STATUS & USART_DREIF_bm) ); //Wait until DATA buffer is empty
    	USARTC1.DATA = *s;  
    	s++;  
	}
}

void usart_int(uint16_t i, uint8_t base){
	char  buf[16];
	itoa(i, buf, base);
	usart_send(buf);
}

void usart_init(){
	USARTC1.BAUDCTRLB = 0xD0;
	USARTC1.BAUDCTRLA = 131;
	USARTC1.CTRLC = USART_CHSIZE_8BIT_gc;
	USARTC1.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
} 

void rc5_on_recieve(uint16_t code){
	uint8_t addr = (code >> 6) & 0x1F;
	uint8_t func = (code >> 0) & 0x3F;
	if (addr == 0 && func == 59)
	{
		ccw = 1;
	}else if (addr == 0 && func == 58)
	{
		ccw = 0;
	}
	start_oncommand(addr, func);

	//usart_int(addr, 16); 
	//usart_send("-");
	//usart_int(func, 16); 
	//usart_send("\n\r");

}

void rc5_on_recieve2(uint16_t code){
//usart_int(code, 16);
}

/*Called during init to retrieve mode from EEPROM*/
void start_get_saved(start_savedata* save){
	save->status = eeprom_read_byte((void*)(START_SAVE_POSITION+1));
	save->dohyo = eeprom_read_byte(START_SAVE_POSITION);
	// usart_send("Saved dohyo: ");
	// usart_int(save->dohyo >> 1, 10);
	// usart_send("\n\r");
}
/*Called during operation to save last mode in EEPROM*/
void start_set_saved(start_savedata* save){
	eeprom_write_byte((void*)(START_SAVE_POSITION+1), save->status);
	eeprom_write_byte(START_SAVE_POSITION, save->dohyo);
	// usart_send("New dohyo: ");
	// usart_int(save->dohyo >> 1, 10);
	// usart_send("\n\r");
}

void start_status_changed(start_state newstat){

}


/*********************************************/
//Init
/*********************************************/
void setup(){
	/*Setup and enable interrupts*/
	PMIC.CTRL = PMIC_LOLVLEN_bm|PMIC_MEDLVLEN_bm|PMIC_HILVLEN_bm;
	sei();
	/*Setup I2C*/
	twi_init(XMEGA_TWI_BAUD(100000));//TWI @ 100KHz

	/*Setup PCA9553 IO Expander*/
	pca9553_init(0x00, 0x00);

	/*Setup PCA9534 IO Expander*/
	pca9534_init(0x20);
	pca9534_dir(0x20, 0xff);
	/*Init VL53L0X*/
	vl53l0x_settings_t s;

	pca9534_out(0x20, 0x03 << 2);
	_delay_ms(2);
	vl53l0x_data_init(VL53L0X_DEFAULT_ADDR, VL53L0X_2V8_MODE, &s);
	vl53l0x_static_init(VL53L0X_DEFAULT_ADDR, &s);
	vl53l0x_set_addr(VL53L0X_DEFAULT_ADDR, VL53L0X_DEFAULT_ADDR+1);
	//pca9534_out(0x20+1, (0x02 >> 2));

	/*Setup serial for debugging*/
	PORTC.DIRSET = 1 << 7;
	usart_init();
	

	usart_send("\n\r");

	/*Setup RC5 reciever*/ 
	rc5_init();
	PORTE.PIN2CTRL |= PORT_ISC_BOTHEDGES_gc;//Falling edge
	PORTE.INT0MASK |= (1 << RC5_PIN);//Enable interrupt
	PORTE.INTCTRL |= PORT_INT0LVL_HI_gc|PORT_INT1LVL_HI_gc;
	//TCE0.PER = RC5_LONG_MAXPULSE + 1;
	TCE0.CNT = 0;
	//TCE0.CTRLA = TC_CLKSEL_DIV8_gc;//32MHz/8 = 250ns/LSB = ticks/
	TCE0.INTCTRLA = TC_OVFINTLVL_HI_gc;

	/*Setup start logic*/
	start_init();
	usart_send("Current status: ");
	switch(start_status()){
		case START_POWER_ON:
			usart_send("POWER_ON\n\r");
		break;
		case START_STARTED:
			usart_send("STARTED\n\r");
		break;
		case START_STOPPED_SAFE:
			usart_send("STOPPED_SAFE\n\r");
		break;
		default:
			usart_send("UNKNOWN\n\r");
		break;
		
	}
	usart_send("Current dohyo: ");
	usart_int(start_dohyo(), 10);
	usart_send("\n\r");

	/*Setup time keeping timer*/
	TCC1.PER = 5000;
	TCC1.CNT = 0;
	TCC1.CTRLA = TC_CLKSEL_DIV64_gc;
	TCC1.INTCTRLA = TC_OVFINTLVL_HI_gc;

	/*Setup edge sensors*/
	adc_init_freerun(&ADCA, ADC_REFSEL_INTVCC_gc, ADC_PRESCALER_DIV512_gc, 0);
	adc_config_channel(&ADCA.CH0,  ADC_CH_MUXPOS_PIN8_gc, ADC_CH_INTLVL_OFF_gc);
	adc_config_channel(&ADCA.CH1,  ADC_CH_MUXPOS_PIN9_gc, ADC_CH_INTLVL_OFF_gc);
	adc_config_channel(&ADCA.CH2, ADC_CH_MUXPOS_PIN10_gc, ADC_CH_INTLVL_OFF_gc);
	adc_config_channel(&ADCA.CH3, ADC_CH_MUXPOS_PIN11_gc, ADC_CH_INTLVL_LO_gc);
	adc_start_freerun(&ADCA, EVSYS_CHMUX_TCC1_OVF_gc);

	/*Init sonar*/
	sonar_init();

	


}

uint16_t k = 0;
float t = 0.0f;
int i = 0;

void update(){
	if(ccw)	{
		t = k*0.025;
	}else{
		t = -k*0.025;
	}
	
	if(start_status() == START_POWER_ON){
		
		float pwm0 = cos(t);
		float pwm1 = sin(t);
		pca9553_pwm0(255-255*fabs(pwm0));
		pca9553_pwm1(255-255*fabs(pwm1));
		pca9553_set(0, pwm1 < 0 ? PCA9553_LS_PWM1 : PCA9553_LS_ON);
		pca9553_set(1, pwm0 > 0 ? PCA9553_LS_PWM0 : PCA9553_LS_ON);
		pca9553_set(2, pwm0 < 0 ? PCA9553_LS_PWM0 : PCA9553_LS_ON);
		pca9553_set(3, pwm1 > 0 ? PCA9553_LS_PWM1 : PCA9553_LS_ON);
	}else if(start_status() == START_STARTED){
		pca9553_set(2, edges[0] < 3000 ? PCA9553_LS_OFF : PCA9553_LS_ON);
		pca9553_set(3, edges[1] < 3000 ? PCA9553_LS_OFF : PCA9553_LS_ON);
		pca9553_set(0, edges[2] < 3000 ? PCA9553_LS_OFF : PCA9553_LS_ON);
		pca9553_set(1, edges[3] < 3000 ? PCA9553_LS_OFF : PCA9553_LS_ON);


	}else if(start_status() == START_PROGRAMMING){
		pca9553_set(0, PCA9553_LS_ON);
		pca9553_set(1, PCA9553_LS_ON);
		pca9553_set(2, PCA9553_LS_ON);
		pca9553_set(3, PCA9553_LS_ON);
	}else if(start_status() == START_STOPPED_SAFE){
		pca9553_set(0, PCA9553_LS_OFF);
		pca9553_set(1, PCA9553_LS_OFF);
		pca9553_set(2, PCA9553_LS_OFF);
		pca9553_set(3, PCA9553_LS_OFF);
	}else{
		
		float pwm0 = cos(t);
		pca9553_pwm0(255-255*fabs(pwm0));
		pca9553_set(0, PCA9553_LS_PWM0);
		pca9553_set(1, PCA9553_LS_PWM0);
		pca9553_set(2, PCA9553_LS_PWM0);
		pca9553_set(3, PCA9553_LS_PWM0);
	}
	
	

	pca9553_update();
	// usart_send("Current status: ");
	// switch(start_status()){
	// 	case START_POWER_ON:
	// 		usart_send("POWER_ON\n\r");
	// 	break;
	// 	case START_STARTED:
	// 		usart_send("STARTED\n\r");
	// 	break;
	// 	case START_STOPPED_SAFE:
	// 		usart_send("STOPPED_SAFE\n\r");
	// 	break;
	// 	case START_STOPPED:
	// 		usart_send("STOPPED\n\r");
	// 	break;
	// 	case START_PROGRAMMING:
	// 		usart_send("PROGRAMMING\n\r");
	// 	break;
	// 	default:
	// 		usart_send("UNKNOWN\n\r");
	// 	break;
		
	// }
	_delay_ms(30);


	sonar_trigger(1);


}



void setClockTo32MHz() {
    CCP = CCP_IOREG_gc;              // disable register security for oscillator update
    OSC.CTRL = OSC_RC32MEN_bm;       // enable 32MHz oscillator
    while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator to be ready
    CCP = CCP_IOREG_gc;              // disable register security for clock update
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to 32MHz clock
}

int main(void){
	setClockTo32MHz();
	setup();
	while(1){
		update();
	}
	return 1;
}


ISR(PORTE_INT0_vect){
	TCE0.CTRLA = TC_CLKSEL_OFF_gc;
	
	//_delay_us(10);
	count = TCE0.CNT;

	edge = (PORTE.IN & (1 << RC5_PIN)) ? 1 : 0;//1 if rising edge, 0 otherwise

	//rc5_on_recieve(count);
	rc5_update(count, edge);
	TCE0.CNT = 0;
	TCE0.CTRLA = TC_CLKSEL_DIV64_gc;//32MHz/8 = 250ns/LSB = ticks/
}

ISR(TCE0_OVF_vect){
	rc5_reset(0, 0);
	TCE0.CTRLA = TC_CLKSEL_OFF_gc;
}

ISR(TCC1_OVF_vect){
	k++;
	start_update();
}

ISR(ADCA_CH3_vect){
	edges[0] = ADCA.CH0RES;
	edges[1] = ADCA.CH1RES;
	edges[2] = ADCA.CH2RES;
	edges[3] = ADCA.CH3RES;
}



