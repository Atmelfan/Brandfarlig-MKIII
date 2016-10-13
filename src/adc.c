#include "adc.h"

uint8_t ReadSignatureByte(uint16_t Address)
{
    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    uint8_t Result;
    __asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;
    return Result;
}

void adc_init(ADC_t* adc, ADC_REFSEL_t ref, ADC_PRESCALER_t prescl, uint8_t mode, uint16_t call, uint16_t calh){
	adc->CTRLA = ADC_ENABLE_bm;//Enable ADC
	adc->CTRLB = mode;//Enable ADC
	adc->REFCTRL = ref;//Avcc/1.6 = 2V
	adc->PRESCALER = prescl;
	adc->CALL = ReadSignatureByte(call); //ADC Calibration Byte 0
    adc->CALH = ReadSignatureByte(calh); //ADC Calibration Byte 1
}

void adc_config(ADC_CH_t* ch, ADC_CH_GAIN_t gain, ADC_CH_INPUTMODE_t mode, uint8_t pin){
	ch->CTRL = (gain | mode);
	ch->INTCTRL = 0x00;
	ch->MUXCTRL = (pin << 3);
}

void adc_config_diff(ADC_CH_t* ch, ADC_CH_GAIN_t gain, ADC_CH_INPUTMODE_t mode, ADC_CH_MUXPOS_t pos, ADC_CH_MUXNEG_t neg){
	ch->CTRL = (gain | mode);
	ch->INTCTRL = 0x00;
	ch->MUXCTRL = pos | neg;;
}

int16_t adc_read(ADC_CH_t* ch){
	ch->CTRL |= ADC_CH_START_bm;
	while(!ch->INTFLAGS) ; // Wait for complete
    ch->INTFLAGS |= ADC_CH_CHIF_bm;
    return ch->RES;
}

int16_t adc_read_pin(ADC_CH_t* ch, uint8_t pin){
	ch->MUXCTRL = (pin << 3);
    return adc_read(ch);
}

int16_t adc_offset(ADC_CH_t* ch){
	adc_config(ch, ADC_CH_GAIN_1X_gc, ADC_CH_INPUTMODE_INTERNAL_gc, 0);
    return adc_read(ch);
}
