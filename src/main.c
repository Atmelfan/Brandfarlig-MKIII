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
//#include "gpa_pca9534.h"
#include "gpa_rc5.h"
#include "gpa_start.h"
//#include "gpa_sonar_stereo.h"
//#include "vl53l0x.h"
#include "motors.h"
#include <math.h>


#define EDGE_THRESHOLD 3000
#define DIST_THRESHOLD 2000		/*Lower -> further, Higher -> closer*/

#define BRF_ROTATE_CCW 	1
#define BRF_ROTATE_CW  	0

#define MOTORS_RIGHT 0
#define MOTORS_LEFT 1

#define EDGE_BL 0
#define EDGE_BR 1
#define EDGE_FL 2
#define EDGE_FR 3

#define DIST_FR 3
#define DIST_FL 7
#define DIST_BR 2
#define DIST_BL 6

#define DIST_FM 7
#define DIST_BM 6



/*IR commands*/
#define CMD_DIST_DEC		50
#define CMD_DIST_INC		51
#define DIST_D 100
//Indicator LED mode
#define CMD_LEDMODE_DIST 	52
#define CMD_LEDMODE_EDGE 	53
//Manouever delay
#define CMD_DELAY_DEC 		54
#define CMD_DELAY_INC 		55
#define DELAY_D 1
//Edge sensor hysteresis, will auto calibrate them too with the new hysteresis
#define CMD_EDGE_HYST_DEC 	60
#define CMD_EDGE_HYST_INC	61
#define EDGE_D_HYST 50
//Rotation direction
#define CMD_ROTATE_CCW 		58
#define CMD_ROTATE_CW 		59
//Base velocity
#define CMD_VEL_DEC 		60
#define CMD_VEL_INC			61
#define VEL_D 		1000
//Auto calibrate edge sensors
#define CMD_ACAL	 		62
//Reset status
#define CMD_RESET 			63


/*EEPROM variables*/
uint8_t ee_dohyo EEMEM = 0;
uint8_t ee_stat EEMEM = 0;

uint16_t ee_edge_threshold EEMEM = 3000;
uint16_t ee_edge_hysteresis EEMEM = 3000;
uint16_t edge_threshold;


#define BASE_VELOCITY_DEFAULT (UINT16_MAX/4)
uint16_t ee_base_velocity EEMEM = BASE_VELOCITY_DEFAULT;
volatile uint16_t base_velocity = BASE_VELOCITY_DEFAULT;


#define BASE_DELAY_DEFAULT 5
uint8_t ee_base_delay EEMEM = BASE_DELAY_DEFAULT;
volatile uint8_t base_delay = BASE_DELAY_DEFAULT;


uint8_t ee_rotation_direction EEMEM = BRF_ROTATE_CW;
volatile uint8_t rotation_direction = BRF_ROTATE_CW;

uint16_t ee_dist_threshold EEMEM = 1000;
uint16_t dist_threshold = 1000;

volatile uint8_t has_read_once = 0;

volatile uint8_t index = 0;
volatile uint8_t tg = 0;
volatile uint8_t edge;
volatile uint16_t count;

volatile uint8_t ccw = 1;

volatile uint8_t adc_ch = 0;
volatile uint16_t adc[12];

volatile uint8_t ledmode_dist = 0;

volatile uint8_t test_dist = 0;


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

void do_base_delay(uint8_t t){
	uint8_t s = t*(base_delay&0x00ff);
	for (int i = 0; i < s; ++i)
	{	
		_delay_ms(10);
	}
}

uint16_t edge_avg(){
	uint16_t sum = 0;
	for (int i = 0; i < 4; ++i)
	{
		sum += adc[8 + i];
	}
	return sum/4;
}

void edge_acal(uint16_t hyst){
	uint16_t avg = edge_avg();
	edge_threshold = (avg) & 0x0fff;
}

uint8_t edge_get(uint8_t ch){
	if(ch >= 4) return 0;

	return adc[8 + ch] < EDGE_THRESHOLD;
}

void rc5_on_recieve(uint16_t code){
	uint8_t addr = (code >> 6) & 0x1F;
	uint8_t func = (code >> 0) & 0x3F;
	if (addr == 0){
		if (func == CMD_ROTATE_CW || func == CMD_ROTATE_CCW){
			if(func == CMD_ROTATE_CW){
				rotation_direction = BRF_ROTATE_CW;
			}else{
				rotation_direction = BRF_ROTATE_CCW;
			}

			eeprom_write_byte(&ee_rotation_direction, rotation_direction);
		}else if (func == CMD_VEL_INC || func == CMD_VEL_DEC){
			if(func == CMD_VEL_INC && base_velocity <= (UINT16_MAX - VEL_D)){
				base_velocity += VEL_D;
			}else if (func == CMD_VEL_DEC && base_velocity > VEL_D){
				base_velocity -= VEL_D;
			}
			//base_velocity = 0;
			eeprom_write_word(&ee_base_velocity, base_velocity);
		}else if (func == CMD_DELAY_INC || func == CMD_DELAY_DEC){
			if (func == CMD_DELAY_INC && base_delay <= 10){
				base_delay += DELAY_D;
			}else if(func == CMD_DELAY_DEC && base_delay > 1){
				base_delay -= DELAY_D;
			}
			eeprom_write_byte(&ee_base_delay, base_delay);
		}else if (func == CMD_RESET){
			start_reset();
		}else if (func == CMD_LEDMODE_EDGE || func == CMD_LEDMODE_DIST){
			if (func == CMD_LEDMODE_DIST){
				ledmode_dist = 1;
			}else{
				ledmode_dist = 0;
			}
		}else if(func == CMD_DIST_INC || func == CMD_DIST_DEC){
			test_dist = 200;
			if (func == CMD_DIST_DEC && dist_threshold >= DIST_D){
				dist_threshold -= DIST_D;
			}else if (func == CMD_DIST_INC && dist_threshold <= 4000-DIST_D){
				dist_threshold += DIST_D;
			}
			eeprom_write_word(&ee_dist_threshold, dist_threshold);
		}

	}else{

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
	save->status = eeprom_read_byte(&ee_stat);
	save->dohyo = eeprom_read_byte(&ee_dohyo);
	// usart_send("Saved dohyo: ");
	// usart_int(save->dohyo >> 1, 10);
	// usart_send("\n\r");
}
/*Called during operation to save last mode in EEPROM*/
void start_set_saved(start_savedata* save){
	eeprom_write_byte(&ee_stat, save->status);
	eeprom_write_byte(&ee_dohyo, save->dohyo);
	// usart_send("New dohyo: ");
	// usart_int(save->dohyo >> 1, 10);
	// usart_send("\n\r");
}

void start_status_changed(start_state newstat){

}

uint16_t get_distance(uint8_t distsens){
	return adc[distsens];
}

uint8_t get_front(uint8_t frontsens){
	return !(PORTC.IN & (1 << frontsens));
}


void read_settings(uint8_t force_default){
	edge_threshold = eeprom_read_word(&ee_edge_threshold);
	if (edge_threshold == 0 || edge_threshold >= 4096 || force_default){
		edge_threshold = 3000;
		eeprom_write_word(&ee_edge_threshold, edge_threshold);
	}

	uint16_t hyst = eeprom_read_word(&ee_edge_hysteresis);
	if (hyst == 0 || hyst >= 4096 || force_default){
		hyst = 300;
		eeprom_write_word(&ee_edge_hysteresis, hyst);
	}
	//edge_acal(hyst);

	base_velocity = eeprom_read_word(&ee_base_velocity);
	if (base_velocity < 1000 || force_default){
		base_velocity = BASE_VELOCITY_DEFAULT;
		eeprom_write_word(&ee_base_velocity, base_velocity);
	}

	base_delay = eeprom_read_byte(&ee_base_delay);
	if (base_delay > 10 || force_default){
		base_delay = BASE_DELAY_DEFAULT;
		eeprom_write_byte(&ee_base_delay, base_delay);
	}


	rotation_direction = eeprom_read_byte(&ee_rotation_direction);
	if (force_default){
		rotation_direction = BRF_ROTATE_CW;
		eeprom_write_byte(&ee_rotation_direction, rotation_direction);
	}
	
	dist_threshold = eeprom_read_word(&ee_dist_threshold);
	if (dist_threshold == 0 || dist_threshold >= 4096 || force_default){
		dist_threshold = DIST_THRESHOLD;
		eeprom_write_word(&ee_dist_threshold, dist_threshold);
	}

}

/*********************************************/
//Init
/*********************************************/
void setup(){
	/*Setup and enable interrupts*/
	PMIC.CTRL = PMIC_LOLVLEN_bm|PMIC_MEDLVLEN_bm|PMIC_HILVLEN_bm;
	sei();


	/*Init motors*/
	motors_init();
	motors_pwm(MOTORS_RIGHT, 0, MOTORS_FORWARD);
	motors_pwm(MOTORS_LEFT, 0, MOTORS_FORWARD);

	/*Setup I2C*/
	twi_init(XMEGA_TWI_BAUD(100000));//TWI @ 100KHz

	/*Setup PCA9553 IO Expander*/
	pca9553_init(0x00, 0x00);

	/*Setup PCA9534 IO Expander*/
	//pca9534_init(0x20);
	//pca9534_dir(0x20, 0xff);
	//pca9534_out(0x20, 0x00);
	//_delay_ms(100);

	/*Init VL53L0X*/
	//vl53l0x_settings_t s;

	//pca9534_out(0x20, 0x03 << 4);
	//_delay_ms(2);
	//vl53l0x_data_init(VL53L0X_DEFAULT_ADDR, VL53L0X_2V8_MODE, &s);
	//vl53l0x_static_init(VL53L0X_DEFAULT_ADDR, &s);
	//vl53l0x_set_addr(VL53L0X_DEFAULT_ADDR, VL53L0X_DEFAULT_ADDR);
	//vl53l0x_start_continous(VL53L0X_DEFAULT_ADDR, &s, 0);
	//pca9534_out(0x20+1, (0x02 >> 2));

	/*Setup serial for debugging*/
	//PORTC.DIRSET = 1 << 7;
	//usart_init();
	//usart_send("\n\r");

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
	//sart_send("Current status: ");
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
	// 	default:
	// 		usart_send("UNKNOWN\n\r");
	// 	break;
		
	// }
	// usart_send("Current dohyo: ");
	// usart_int(start_dohyo(), 10);
	// usart_send("\n\r");

	/*Setup time keeping timer*/
	TCC1.PER = 5000;
	TCC1.CNT = 0;
	TCC1.CTRLA = TC_CLKSEL_DIV64_gc;
	TCC1.INTCTRLA = TC_OVFINTLVL_HI_gc;

	/*Setup edge sensors*/
	adc_init_freerun(&ADCA, ADC_REFSEL_INTVCC_gc, ADC_PRESCALER_DIV512_gc, 0);
	adc_config_channel(&ADCA.CH0, ADC_CH_MUXPOS_PIN0_gc, ADC_CH_INTLVL_OFF_gc);
	adc_config_channel(&ADCA.CH1, ADC_CH_MUXPOS_PIN1_gc, ADC_CH_INTLVL_OFF_gc);
	adc_config_channel(&ADCA.CH2, ADC_CH_MUXPOS_PIN2_gc, ADC_CH_INTLVL_OFF_gc);
	adc_config_channel(&ADCA.CH3, ADC_CH_MUXPOS_PIN3_gc, ADC_CH_INTLVL_LO_gc);
	adc_start_freerun(&ADCA, EVSYS_CHMUX_TCC1_OVF_gc);

	/*Init sonar*/
	//sonar_init();
	//sonar_trigger_free(1, 1);
	

	while(!has_read_once);
	
	read_settings(0);
	

}

void led_edges(){
	pca9553_set(2, edge_get(EDGE_BL) ? PCA9553_LS_OFF : PCA9553_LS_ON);
	pca9553_set(3, edge_get(EDGE_BR) ? PCA9553_LS_OFF : PCA9553_LS_ON);
	pca9553_set(0, edge_get(EDGE_FL) ? PCA9553_LS_OFF : PCA9553_LS_ON);
	pca9553_set(1, edge_get(EDGE_FR) ? PCA9553_LS_OFF : PCA9553_LS_ON);
}

void led_distance(){
	pca9553_set(2, get_distance(DIST_BL) > dist_threshold || get_front(DIST_BM) ? PCA9553_LS_OFF : PCA9553_LS_ON);
	pca9553_set(3, get_distance(DIST_BR) > dist_threshold || get_front(DIST_BM) ? PCA9553_LS_OFF : PCA9553_LS_ON);
	pca9553_set(0, get_distance(DIST_FL) > dist_threshold || get_front(DIST_FM) ? PCA9553_LS_OFF : PCA9553_LS_ON);
	pca9553_set(1, get_distance(DIST_FR) > dist_threshold || get_front(DIST_FM) ? PCA9553_LS_OFF : PCA9553_LS_ON);
}

uint16_t k = 0;
float t = 0.0f;
int i = 0;
uint8_t stat=0;

void update(){
	if(!rotation_direction)	{
		t = k*0.025;
	}else{
		t = -k*0.025;
	}
	
	if(start_status() == START_POWER_ON){
		float pwm0 = cos(t);
		float pwm1 = sin(t);
		pca9553_pwm0(255-255*fabs(pwm0));
		pca9553_pwm1(255-255*fabs(pwm1));

		if (test_dist){
			led_distance();
		}else{
			pca9553_set(0, pwm1 < 0 ? PCA9553_LS_PWM1 : PCA9553_LS_ON);
			pca9553_set(1, pwm0 > 0 ? PCA9553_LS_PWM0 : PCA9553_LS_ON);
			pca9553_set(2, pwm0 < 0 ? PCA9553_LS_PWM0 : PCA9553_LS_ON);
			pca9553_set(3, pwm1 > 0 ? PCA9553_LS_PWM1 : PCA9553_LS_ON);
		}
	}else if(start_status() == START_STARTED){
		if (ledmode_dist){
			led_distance();
		}else{
			led_edges();
		}
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

	if (start_status() == START_STARTED){
		if (edge_get(EDGE_FR) && edge_get(EDGE_FL)){//Both front edge sensors
			motors_pwm(MOTORS_RIGHT, base_velocity 	, MOTORS_REVERSE);//Full reverse
			motors_pwm(MOTORS_LEFT , base_velocity 	, MOTORS_REVERSE);
			do_base_delay(1);
		}else if (edge_get(EDGE_BR) && edge_get(EDGE_BL)){//Both back edge sensors
			motors_pwm(MOTORS_RIGHT, base_velocity 	, MOTORS_FORWARD);//Full forward
			motors_pwm(MOTORS_LEFT , base_velocity 	, MOTORS_FORWARD);
			do_base_delay(1);
		}else if (edge_get(EDGE_FL) || edge_get(EDGE_BR)){
			//Go reverse of forward
			if (edge_get(EDGE_FL)){
				motors_pwm(MOTORS_RIGHT, base_velocity 	, MOTORS_REVERSE);
				motors_pwm(MOTORS_LEFT , base_velocity 		, MOTORS_REVERSE);
			}else{
				motors_pwm(MOTORS_RIGHT, base_velocity 	, MOTORS_FORWARD);
				motors_pwm(MOTORS_LEFT , base_velocity 		, MOTORS_FORWARD);
			} 
			do_base_delay(2);
			//Turn left
			motors_pwm(MOTORS_RIGHT, base_velocity 	, MOTORS_REVERSE);
			motors_pwm(MOTORS_LEFT , base_velocity 	, MOTORS_FORWARD);
			do_base_delay(1);
		}else if (edge_get(EDGE_FR) || edge_get(EDGE_BL)){
			//Go reverse of forward
			if (edge_get(EDGE_FR)){
				motors_pwm(MOTORS_RIGHT, base_velocity 	, MOTORS_REVERSE);
				motors_pwm(MOTORS_LEFT , base_velocity 	, MOTORS_REVERSE);
			}else{
				motors_pwm(MOTORS_RIGHT, base_velocity 	, MOTORS_FORWARD);
				motors_pwm(MOTORS_LEFT , base_velocity 	, MOTORS_FORWARD);
			}
			do_base_delay(2);
			//Turn left
			motors_pwm(MOTORS_RIGHT, base_velocity 	, MOTORS_FORWARD);
			motors_pwm(MOTORS_LEFT , base_velocity 	, MOTORS_REVERSE);
			do_base_delay(1);
		}else{
			uint8_t bdist_fr = get_distance(DIST_FR) > dist_threshold;
			uint8_t bdist_fl = get_distance(DIST_FL) > dist_threshold;
			uint8_t bdist_br = get_distance(DIST_BR) > dist_threshold;
			uint8_t bdist_bl = get_distance(DIST_BL) > dist_threshold;

			if ((bdist_fr && bdist_fl) || get_front(DIST_FM)){
				motors_pwm(MOTORS_RIGHT, base_velocity 	, MOTORS_FORWARD);//Full forward
				motors_pwm(MOTORS_LEFT , base_velocity 	, MOTORS_FORWARD);
			}else if((bdist_br && bdist_bl) || get_front(DIST_BM)){
				motors_pwm(MOTORS_RIGHT, base_velocity 	, MOTORS_REVERSE);
				motors_pwm(MOTORS_LEFT , base_velocity 	, MOTORS_REVERSE);
			}else if(bdist_fr || bdist_fl){
				if (bdist_fl){
					motors_pwm(MOTORS_RIGHT, base_velocity		, MOTORS_FORWARD);
					motors_pwm(MOTORS_LEFT , base_velocity*.5 	, MOTORS_FORWARD);
					rotation_direction = BRF_ROTATE_CCW;
				}else{
					motors_pwm(MOTORS_RIGHT, base_velocity*.5 	, MOTORS_FORWARD);
					motors_pwm(MOTORS_LEFT , base_velocity		, MOTORS_FORWARD);
					rotation_direction = BRF_ROTATE_CW;
				}
				
			}else if(bdist_br || bdist_bl){
				if (bdist_bl){
					motors_pwm(MOTORS_RIGHT, base_velocity		, MOTORS_REVERSE);
					motors_pwm(MOTORS_LEFT , base_velocity*.5 	, MOTORS_REVERSE);
					rotation_direction = BRF_ROTATE_CW;
				}else{
					motors_pwm(MOTORS_RIGHT, base_velocity*.5 	, MOTORS_REVERSE);
					motors_pwm(MOTORS_LEFT , base_velocity		, MOTORS_REVERSE);
					rotation_direction = BRF_ROTATE_CCW;
				}
			}else if(rotation_direction == BRF_ROTATE_CW){
			 	motors_pwm(MOTORS_RIGHT, base_velocity, MOTORS_REVERSE);
			 	motors_pwm(MOTORS_LEFT, base_velocity, MOTORS_FORWARD);
			}else{
			 	motors_pwm(MOTORS_RIGHT, base_velocity, MOTORS_FORWARD);
			 	motors_pwm(MOTORS_LEFT, base_velocity, MOTORS_REVERSE);
			}
			//motors_pwm(MOTORS_RIGHT, base_velocity, MOTORS_FORWARD);
			//motors_pwm(MOTORS_LEFT, base_velocity, MOTORS_FORWARD);
		}
	}else{
		motors_pwm(MOTORS_RIGHT, 0, MOTORS_FORWARD);
		motors_pwm(MOTORS_LEFT, 0, MOTORS_FORWARD);
	}

	
	

	_delay_ms(2);
	//usart_int(vl53l0x_get_range(VL53L0X_DEFAULT_ADDR), 16);
	

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

	if (test_dist){
		test_dist--;
	}
}

ISR(ADCA_CH3_vect){
	adc[adc_ch+0] = ADCA.CH0RES;
	adc[adc_ch+1] = ADCA.CH1RES;
	adc[adc_ch+2] = ADCA.CH2RES;
	adc[adc_ch+3] = ADCA.CH3RES;

	adc_ch += 4;
	if (adc_ch >= 12){
		adc_ch = 0;
		has_read_once = 1;
	}

	adc_config_channel(&ADCA.CH0, (adc_ch + 0) << 3, ADC_CH_INTLVL_OFF_gc);
	adc_config_channel(&ADCA.CH1, (adc_ch + 1) << 3, ADC_CH_INTLVL_OFF_gc);
	adc_config_channel(&ADCA.CH2, (adc_ch + 2) << 3, ADC_CH_INTLVL_OFF_gc);
	adc_config_channel(&ADCA.CH3, (adc_ch + 3) << 3, ADC_CH_INTLVL_LO_gc);
}



