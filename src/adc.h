#ifndef GPA_ADC
#define GPA_ADC

#include <avr/io.h>
/*
 * Init adc,
 * ref = voltage reference, eg AREFA, VCC etc 
 * prescl = adc clock prescaler eg DIV1, DIV128 etc
 * mode = adc mode, FREERUNNING, CONV_MODE, RESOLUTION etc
 * call = Low calibration byte address
 * calh = High calibration byte address
 */
void adc_init(ADC_t* adc, ADC_REFSEL_t ref, ADC_PRESCALER_t prescl, uint8_t mode, uint16_t call, uint16_t calh);

/*
 * Config adc channel
 * gain = channel gain (1x, 8x etc)
 * mode = channel mode, internal, singleended etc
 * pin, pin on IO port (must be set to input)
 */
void adc_config(ADC_CH_t* ch, ADC_CH_GAIN_t gain, ADC_CH_INPUTMODE_t mode, uint8_t pin);

/*
 * Config differential adc channel
 */
void adc_config_diff(ADC_CH_t* ch, ADC_CH_GAIN_t gain, ADC_CH_INPUTMODE_t mode, ADC_CH_MUXPOS_t pos, ADC_CH_MUXNEG_t neg);

/*
 * Read adc channel value
 */
int16_t adc_read(ADC_CH_t* ch);

/*
 * Read adc channel value
 */
int16_t adc_read_pin(ADC_CH_t* ch, uint8_t pin);

/*
 * Read adc channel gnd offset (OBS! original settings will be lost)
 */
int16_t adc_offset(ADC_CH_t* ch);

#endif