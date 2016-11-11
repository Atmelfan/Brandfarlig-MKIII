#include "vl53l0x.h"
#include "stdlib.h"

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

void vl53l0x_tune(uint8_t addr);

uint8_t buffer[8];

void vl53l0x_write_register(uint8_t addr, uint8_t reg,  uint8_t value){
	buffer[0] = reg;
	buffer[1] = value;
	twi_send(addr, buffer, 2);
}

uint8_t vl53l0x_read_register(uint8_t addr, uint8_t reg){
	uint8_t send = reg;
	uint8_t ret;
	twi_send_read(addr, &send, 1, &ret, 1);
	while(!twi_ready());
	return ret;
}

void vl53l0x_write_multi(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t count){
	uint8_t buffer1[count + 1];
	buffer1[0] = reg;
	for (int i = 0; i < count; ++i) {
		buffer1[1 + i] = data[i];
	}

	twi_send(addr, buffer1, count + 1);
}

uint8_t vl53l0x_read_multi(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t count){
	
	twi_send_read(addr, &reg, 1, data, count);
	while(!twi_ready());
	return count;
}

void vl53l0x_write_word(uint8_t addr, uint8_t reg, uint16_t word){
	buffer[0] = reg;
	buffer[1] = (word >> 8) & 0xFF;
	buffer[2] = (word >> 0) & 0xFF;
	twi_send(addr, buffer, 3);
}

uint16_t vl53l0x_read_word(uint8_t addr, uint8_t reg){
	uint8_t ret[2];
	uint8_t send = reg;
	twi_send_read(addr, &send, 1, ret, 2);
	while(!twi_ready());

	return (ret[0] << 8) | (ret[1] << 0);	
}

void vl53l0x_write_dword(uint8_t addr, uint8_t reg, uint32_t dword){
	buffer[0] = reg;
	buffer[1] = (dword >> 24) & 0xFF;
	buffer[2] = (dword >> 16) & 0xFF;
	buffer[3] = (dword >> 8) & 0xFF;
	buffer[4] = (dword >> 0) & 0xFF;

	twi_send(addr, buffer, 5);
}

uint32_t vl53l0x_read_dword(uint8_t addr, uint8_t reg){
	uint8_t send = reg;
	twi_send_read(addr, &send, 1, buffer, 4);
	while(!twi_ready());

	return ((uint32_t)buffer[0] << 24)|((uint32_t)buffer[1] << 16)|((uint32_t)buffer[2] << 8)|((uint32_t)buffer[3] << 0);	
}



// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

 

/*Seems to be the 'sensitivity' setting*/
void vl53l0x_set_signalratelimit(uint8_t addr, float limit){
	if (limit < 0 || limit > 511.99f) return;
	
	vl53l0x_write_word(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit * (1 << 7));
}

uint8_t vl53l0x_set_addr(uint8_t addr, uint8_t new_addr){
	vl53l0x_write_register(addr,  VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, (new_addr << 1) & 0x7F);
	return new_addr;
}

void vl53l0x_data_init(uint8_t addr, vl53l0x_voltage_enum voltage, vl53l0x_settings_t* settings){

	uint8_t temp = vl53l0x_read_register(addr, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
	if (voltage == VL53L0X_2V8_MODE){
		vl53l0x_write_register(addr, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, temp | 0x01);//Set bit 0
	}else{
		vl53l0x_write_register(addr, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, temp & ~0x01);//Clear bit 0
	}

	//I2C standard mode apperently...
	vl53l0x_write_register(addr, 0x88, 0x00);

	//Whatever this is...
	vl53l0x_write_register(addr, 0x80, 0x01);
	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x00, 0x00);
	settings->stop_value = vl53l0x_read_register(addr, 0x91);
	vl53l0x_write_register(addr, 0x00, 0x01);
	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, 0x80, 0x00);


	vl53l0x_set_signalratelimit(addr, 0.25f);

	//No idea... Sensing a theme here?
	vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);
}

void vl53l0x_static_init(uint8_t addr, vl53l0x_settings_t* settings){

	uint8_t spad_count, spad_is_aperture;
	uint8_t spad_ref_map[6];

	vl53l0x_get_spadinfo(addr, &spad_count, &spad_is_aperture);

	vl53l0x_read_multi(addr, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_ref_map, 6);

	vl53l0x_set_refspad(addr, spad_count, spad_is_aperture, spad_ref_map);

	vl53l0x_tune(addr);

	//vl53l0x_set_gpio(addr, VL53L0X_GPIO_INTERRUPT, VL53L0X_GPIO_POLARITY_NEG);

	//uint32_t timing_budget = vl53l0x_get_timing_budget(addr);

	//vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

	//vl53l0x_set_timing_budget(addr, timing_budget);

	//vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
	//vl53l0x_single_refcal(addr, 0x40);

	//vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
	//vl53l0x_single_refcal(addr, 0x00);

	vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
}

void vl53l0x_start_continous(uint8_t addr, vl53l0x_settings_t* settings, uint32_t period_ms){
	vl53l0x_write_register(addr, 0x80, 0x01);
	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x00, 0x00);
	vl53l0x_write_register(addr, 0x91, settings->stop_value);
	vl53l0x_write_register(addr, 0x00, 0x01);
	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, 0x80, 0x00);

	vl53l0x_write_register(addr, VL53L0X_REG_SYSRANGE_START, 0x02);
}

uint16_t vl53l0x_get_range(uint8_t addr){

	while((vl53l0x_read_register(addr, VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0);

	uint16_t range = vl53l0x_read_word(addr, VL53L0X_REG_RESULT_RANGE_STATUS + 10);

	vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

	return range;
}

void vl53l0x_set_gpio(uint8_t addr, vl53l0x_gpio_enum mode, uint8_t polarity){

	vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, mode);

	uint8_t temp = vl53l0x_read_register(addr, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH);
	if (polarity == VL53L0X_GPIO_POLARITY_POS){
		vl53l0x_write_register(addr, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, temp | 0x10);
	}else{
		vl53l0x_write_register(addr, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, temp & ~0x10);
	}

	vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

void vl53l0x_single_refcal(uint8_t addr, uint8_t vhv){
	vl53l0x_write_register(addr, VL53L0X_REG_SYSRANGE_START, 0x01 | vhv);

	while((vl53l0x_read_register(addr, VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0);

	vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

	vl53l0x_write_register(addr, VL53L0X_REG_SYSRANGE_START, 0x00);

}

uint32_t vl53l0x_get_timing_budget(uint8_t addr){
	vl53l0x_sequence_step_enables_t enables;
	vl53l0x_sequence_step_timeouts_t timeouts;

	uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
	uint16_t const EndOverhead        = 960;
	uint16_t const MsrcOverhead       = 660;
	uint16_t const TccOverhead        = 590;
	uint16_t const DssOverhead        = 690;
	uint16_t const PreRangeOverhead   = 660;
	uint16_t const FinalRangeOverhead = 550;

	// "Start and end overhead times always present"
	uint32_t budget_us = StartOverhead + EndOverhead;

	vl53l0x_get_sequence_enables(addr, &enables);
	vl53l0x_get_sequence_timeouts(addr, &enables, &timeouts);

	if (enables.tcc)
	{
		budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss)
	{
		budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}
	else if (enables.msrc)
	{
		budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range)
	{
		budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range)
	{
		budget_us += (timeouts.final_range_us + FinalRangeOverhead);
	}

	return budget_us;
}

void vl53l0x_set_timing_budget(uint8_t addr, uint32_t budget_us){
	vl53l0x_sequence_step_enables_t enables;
	vl53l0x_sequence_step_timeouts_t timeouts;

	uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
	uint16_t const EndOverhead        = 960;
	uint16_t const MsrcOverhead       = 660;
	uint16_t const TccOverhead        = 590;
	uint16_t const DssOverhead        = 690;
	uint16_t const PreRangeOverhead   = 660;
	uint16_t const FinalRangeOverhead = 550;

	uint32_t const MinTimingBudget = 20000;

	if (budget_us < MinTimingBudget) { return; }

	uint32_t used_budget_us = StartOverhead + EndOverhead;

	vl53l0x_get_sequence_enables(addr, &enables);
	vl53l0x_get_sequence_timeouts(addr, &enables, &timeouts);

	if (enables.tcc)
	{
		used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss)
	{
		used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}
	else if (enables.msrc)
	{
		used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range)
	{
		used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range)
	{
		used_budget_us += FinalRangeOverhead;

		// "Note that the final range timeout is determined by the timing
		// budget and the sum of all other timeouts within the sequence.
		// If there is no room for the final range timeout, then an error
		// will be set. Otherwise the remaining time will be applied to
		// the final range."

		if (used_budget_us > budget_us){
			// "Requested timeout too big."
			return;
		}

		uint32_t final_range_timeout_us = budget_us - used_budget_us;

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."

		uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

		if (enables.pre_range) {
			final_range_timeout_mclks += timeouts.pre_range_mclks;
		}

		vl53l0x_write_word(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));

		// set_sequence_step_timeout() end
	}
}

void vl53l0x_set_refspad(uint8_t addr, uint8_t spad_count, uint8_t spad_is_aperture, uint8_t* spad_ref_map){
	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	vl53l0x_write_register(addr, VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	uint8_t first_spad_to_enable = spad_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++)
	{
		if (i < first_spad_to_enable || spads_enabled == spad_count)
		{
			// This bit is lower than the first one that should be enabled, or
			// (reference_spad_count) bits have already been enabled, so zero this bit
			spad_ref_map[i / 8] &= ~(1 << (i % 8));
		}
		else if ((spad_ref_map[i / 8] >> (i % 8)) & 0x1)
		{
			spads_enabled++;
		}
	}

	vl53l0x_write_multi(addr, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_ref_map, 6);
}

uint8_t vl53l0x_get_vcsel(uint8_t addr, vl53l0x_vcsel_type_enum type){
	if (type == VL53L0X_VCSEL_PERIOD_PRERANGE){
		return decodeVcselPeriod(vl53l0x_read_register(addr, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD));
	}else if (type == VL53L0X_VCSEL_PERIOD_FINALRANGE){
		return decodeVcselPeriod(vl53l0x_read_register(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	}else{
		return 0xFF;
	}
}

void vl53l0x_set_vcsel(uint8_t addr, vl53l0x_vcsel_type_enum type, uint8_t period_pclks, uint32_t budget_us){
	uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
	vl53l0x_sequence_step_enables_t enables;
	vl53l0x_sequence_step_timeouts_t timeouts;
	vl53l0x_get_sequence_enables(addr, &enables);
	vl53l0x_get_sequence_timeouts(addr, &enables, &timeouts);

	if(type == VL53L0X_VCSEL_PERIOD_PRERANGE){
		switch (period_pclks)
		{
			case 12:
				vl53l0x_write_register(addr, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
			break;

			case 14:
				vl53l0x_write_register(addr, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
			break;

			case 16:
				vl53l0x_write_register(addr, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
			break;

			case 18:
				vl53l0x_write_register(addr, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
			break;

			default:
				// invalid period
				return;
		}

		vl53l0x_write_register(addr, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

	    // apply new VCSEL period
	    vl53l0x_write_register(addr, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

	    // update timeouts

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

	    uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

	    vl53l0x_write_word(addr, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));

	    // set_sequence_step_timeout() end

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

	    uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

	    vl53l0x_write_word(addr, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

	}else if(type == VL53L0X_VCSEL_PERIOD_FINALRANGE){
		switch (period_pclks)
		{
			case 8:
				vl53l0x_write_register(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
				vl53l0x_write_register(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				vl53l0x_write_register(addr, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
				vl53l0x_write_register(addr, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
				vl53l0x_write_register(addr, 0xFF, 0x01);
				vl53l0x_write_register(addr, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x30);
				vl53l0x_write_register(addr, 0xFF, 0x00);
			break;

			case 10:
				vl53l0x_write_register(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
				vl53l0x_write_register(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				vl53l0x_write_register(addr, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				vl53l0x_write_register(addr, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
				vl53l0x_write_register(addr, 0xFF, 0x01);
				vl53l0x_write_register(addr, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
				vl53l0x_write_register(addr, 0xFF, 0x00);
			break;

			case 12:
				vl53l0x_write_register(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
				vl53l0x_write_register(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				vl53l0x_write_register(addr, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				vl53l0x_write_register(addr, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
				vl53l0x_write_register(addr, 0xFF, 0x01);
				vl53l0x_write_register(addr, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
				vl53l0x_write_register(addr, 0xFF, 0x00);
			break;

			case 14:
				vl53l0x_write_register(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
				vl53l0x_write_register(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				vl53l0x_write_register(addr, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				vl53l0x_write_register(addr, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
				vl53l0x_write_register(addr, 0xFF, 0x01);
				vl53l0x_write_register(addr, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
				vl53l0x_write_register(addr, 0xFF, 0x00);
			break;

			default:
				// invalid period
				return;
		}
		vl53l0x_write_register(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

		if(enables.pre_range){
			new_final_range_timeout_mclks += timeouts.pre_range_mclks;
		}

		vl53l0x_write_word(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));

	}else{
		return;
	}

	vl53l0x_set_timing_budget(addr, budget_us);

	// "Perform the phase calibration. This is needed after changing on vcsel period."
	// VL53L0X_perform_phase_calibration() begin

	uint8_t sequence_config = vl53l0x_read_register(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);
	vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
	vl53l0x_single_refcal(addr, 0x00);
	vl53l0x_write_register(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);


}

void vl53l0x_get_sequence_enables(uint8_t addr, vl53l0x_sequence_step_enables_t* enables){
	uint8_t sequence_config = vl53l0x_read_register(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);

	enables->tcc          = (sequence_config >> 4) & 0x1;
	enables->dss          = (sequence_config >> 3) & 0x1;
	enables->msrc         = (sequence_config >> 2) & 0x1;
	enables->pre_range    = (sequence_config >> 6) & 0x1;
	enables->final_range  = (sequence_config >> 7) & 0x1;
}

void vl53l0x_get_sequence_timeouts(uint8_t addr,  vl53l0x_sequence_step_enables_t* enables, vl53l0x_sequence_step_timeouts_t* timeouts){
	//PRERANGE
	timeouts->pre_range_vcsel_period_pclks = vl53l0x_get_vcsel(addr, VL53L0X_VCSEL_PERIOD_PRERANGE);

	timeouts->msrc_dss_tcc_mclks = vl53l0x_read_register(addr, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP) + 1;

	timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

	timeouts->pre_range_mclks = decodeTimeout(vl53l0x_read_word(addr, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));

	timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

	//FINALRANGE
	timeouts->final_range_vcsel_period_pclks = vl53l0x_get_vcsel(addr, VL53L0X_VCSEL_PERIOD_FINALRANGE);

	timeouts->final_range_mclks = decodeTimeout(vl53l0x_read_word(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

	if(enables->pre_range){
		timeouts->final_range_mclks -= timeouts->pre_range_mclks;
	}

	timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);

}


void vl53l0x_get_spadinfo(uint8_t addr, uint8_t* count, uint8_t* is_aperture){
	uint8_t temp;

	vl53l0x_write_register(addr, 0x80, 0x01);
	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x00, 0x00);

	vl53l0x_write_register(addr, 0xFF, 0x06);

	temp = vl53l0x_read_register(addr, 0x83);
	vl53l0x_write_register(addr, 0x83, temp | 0x04);
	vl53l0x_write_register(addr, 0xFF, 0x07);
	vl53l0x_write_register(addr, 0x81, 0x01);

	vl53l0x_write_register(addr, 0x80, 0x01);

	vl53l0x_write_register(addr, 0x94, 0x6B);
	vl53l0x_write_register(addr, 0x83, 0x00);

	while (vl53l0x_read_register(addr, 0x83) == 0x00);

	vl53l0x_write_register(addr, 0x83, 0x01);
	temp = vl53l0x_read_register(addr, 0x92);

	*count = temp & 0x7F;
	*is_aperture = (temp >> 7) & 0x01;


	vl53l0x_write_register(addr, 0x81, 0x00);
	vl53l0x_write_register(addr, 0xFF, 0x06);

	temp = vl53l0x_read_register(addr, 0x83);
	vl53l0x_write_register(addr, 0x83, temp & ~0x04);
	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x00, 0x01);

	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, 0x80, 0x00);
}

void vl53l0x_tune(uint8_t addr){
	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x00, 0x00);

	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, 0x09, 0x00);
	vl53l0x_write_register(addr, 0x10, 0x00);
	vl53l0x_write_register(addr, 0x11, 0x00);

	vl53l0x_write_register(addr, 0x24, 0x01);
	vl53l0x_write_register(addr, 0x25, 0xFF);
	vl53l0x_write_register(addr, 0x75, 0x00);

	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x4E, 0x2C);
	vl53l0x_write_register(addr, 0x48, 0x00);
	vl53l0x_write_register(addr, 0x30, 0x20);

	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, 0x30, 0x09);
	vl53l0x_write_register(addr, 0x54, 0x00);
	vl53l0x_write_register(addr, 0x31, 0x04);
	vl53l0x_write_register(addr, 0x32, 0x03);
	vl53l0x_write_register(addr, 0x40, 0x83);
	vl53l0x_write_register(addr, 0x46, 0x25);
	vl53l0x_write_register(addr, 0x60, 0x00);
	vl53l0x_write_register(addr, 0x27, 0x00);
	vl53l0x_write_register(addr, 0x50, 0x06);
	vl53l0x_write_register(addr, 0x51, 0x00);
	vl53l0x_write_register(addr, 0x52, 0x96);
	vl53l0x_write_register(addr, 0x56, 0x08);
	vl53l0x_write_register(addr, 0x57, 0x30);
	vl53l0x_write_register(addr, 0x61, 0x00);
	vl53l0x_write_register(addr, 0x62, 0x00);
	vl53l0x_write_register(addr, 0x64, 0x00);
	vl53l0x_write_register(addr, 0x65, 0x00);
	vl53l0x_write_register(addr, 0x66, 0xA0);

	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x22, 0x32);
	vl53l0x_write_register(addr, 0x47, 0x14);
	vl53l0x_write_register(addr, 0x49, 0xFF);
	vl53l0x_write_register(addr, 0x4A, 0x00);

	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, 0x7A, 0x0A);
	vl53l0x_write_register(addr, 0x7B, 0x00);
	vl53l0x_write_register(addr, 0x78, 0x21);

	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x23, 0x34);
	vl53l0x_write_register(addr, 0x42, 0x00);
	vl53l0x_write_register(addr, 0x44, 0xFF);
	vl53l0x_write_register(addr, 0x45, 0x26);
	vl53l0x_write_register(addr, 0x46, 0x05);
	vl53l0x_write_register(addr, 0x40, 0x40);
	vl53l0x_write_register(addr, 0x0E, 0x06);
	vl53l0x_write_register(addr, 0x20, 0x1A);
	vl53l0x_write_register(addr, 0x43, 0x40);

	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, 0x34, 0x03);
	vl53l0x_write_register(addr, 0x35, 0x44);

	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x31, 0x04);
	vl53l0x_write_register(addr, 0x4B, 0x09);
	vl53l0x_write_register(addr, 0x4C, 0x05);
	vl53l0x_write_register(addr, 0x4D, 0x04);

	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, 0x44, 0x00);
	vl53l0x_write_register(addr, 0x45, 0x20);
	vl53l0x_write_register(addr, 0x47, 0x08);
	vl53l0x_write_register(addr, 0x48, 0x28);
	vl53l0x_write_register(addr, 0x67, 0x00);
	vl53l0x_write_register(addr, 0x70, 0x04);
	vl53l0x_write_register(addr, 0x71, 0x01);
	vl53l0x_write_register(addr, 0x72, 0xFE);
	vl53l0x_write_register(addr, 0x76, 0x00);
	vl53l0x_write_register(addr, 0x77, 0x00);

	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x0D, 0x01);

	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, 0x80, 0x01);
	vl53l0x_write_register(addr, 0x01, 0xF8);

	vl53l0x_write_register(addr, 0xFF, 0x01);
	vl53l0x_write_register(addr, 0x8E, 0x01);
	vl53l0x_write_register(addr, 0x00, 0x01);
	vl53l0x_write_register(addr, 0xFF, 0x00);
	vl53l0x_write_register(addr, 0x80, 0x00);
}











