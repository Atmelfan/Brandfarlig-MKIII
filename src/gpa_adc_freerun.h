#ifndef GPA_ADC_FREERUN_H_
#define GPA_ADC_FREERUN_H_ 
#include <stdint.h>
#include <avr/io.h>


void adc_init_freerun(ADC_t* adc ,ADC_REFSEL_t ref, ADC_PRESCALER_t prescl, uint8_t mode);

void adc_config_channel(ADC_CH_t* ch, uint8_t mux, ADC_CH_INTLVL_t lvl);

void adc_start_freerun(ADC_t* adc, EVSYS_CHMUX_t trigger);

#endif
