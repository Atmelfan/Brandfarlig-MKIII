#include "gpa_adc_freerun.h"

uint8_t ReadSignatureByte1(uint16_t Address)
{
    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    uint8_t Result;
    __asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;
    return Result;
}


void adc_init_freerun(ADC_t* adc ,ADC_REFSEL_t ref, ADC_PRESCALER_t prescl, uint8_t mode){
	adc->CTRLA = ADC_ENABLE_bm;//Enable ADC
	adc->CTRLB = mode;//Enable ADC
	adc->REFCTRL = ref;//Avcc/1.6 = 2V
	adc->PRESCALER = prescl;
	adc->CALL = ReadSignatureByte1(0x20); //ADC Calibration Byte 0
	adc->CALH = ReadSignatureByte1(0x21); //ADC Calibration Byte 1
	adc->EVCTRL = ADC_SWEEP_0123_gc | ADC_EVSEL_567_gc | ADC_EVACT_SWEEP_gc;//Sweep channel 0-4
}

void adc_config_channel(ADC_CH_t* ch, uint8_t mux, ADC_CH_INTLVL_t lvl){
	ch->CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;//Single ended input
	ch->MUXCTRL = mux;
	ch->INTCTRL = lvl;
}

void adc_start_freerun(ADC_t* adc, EVSYS_CHMUX_t trigger){
	adc->EVCTRL      = ADC_SWEEP_0123_gc | ADC_EVSEL_567_gc | ADC_EVACT_SWEEP_gc;
	adc->CTRLA       = ADC_ENABLE_bm;
	EVSYS.CH5MUX = trigger;
	EVSYS.CH5CTRL = 0;

}












