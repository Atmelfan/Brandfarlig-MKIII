//1. VL53L0X
//2. Sonar
//3. Start
//4. fancy
#include "gpa_twi_generic.h"
#include <stdint.h>

typedef enum{
	VL53L0X_MODE_SINGLE			= 0,
	VL53L0X_MODE_CONTINIUS 		= 1,
	VL53L0X_MODE_CONTINIUS_TIMED= 3
} vl53l0x_modes_enum;

typedef enum{
	VL53L0X_VCSEL_PERIOD_PRERANGE,
	VL53L0X_VCSEL_PERIOD_FINALRANGE
} vl53l0x_vcsel_type_enum;

typedef enum{
	VL53L0X_GPIO_OFF 		= 0x00,
	VL53L0X_GPIO_THRH_LOW 	= 0x01,
	VL53L0X_GPIO_THRH_HIGH 	= 0x02,
	VL53L0X_GPIO_THRH_OUT 	= 0x03,
	VL53L0X_GPIO_INTERRUPT	= 0x04,
} vl53l0x_gpio_enum;


typedef enum{
	VL53L0X_1V8_MODE,
	VL53L0X_2V8_MODE
} vl53l0x_voltage_enum;

typedef struct{
	uint8_t tcc, msrc, dss, pre_range, final_range;
} vl53l0x_sequence_step_enables_t;

typedef struct{
	uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

	uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
	uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} vl53l0x_sequence_step_timeouts_t;

typedef struct
{
	uint8_t stop_value;
	uint32_t measurement_timing_budget_us;
} vl53l0x_settings_t;

#define VL53L0X_GPIO_POLARITY_POS 0
#define VL53L0X_GPIO_POLARITY_NEG 1

#define VL53L0X_DEFAULT_ADDR 0x29

uint8_t vl53l0x_set_addr(uint8_t addr, uint8_t new_addr);

void vl53l0x_data_init(uint8_t addr, vl53l0x_voltage_enum voltage, vl53l0x_settings_t* settings);

void vl53l0x_static_init(uint8_t addr, vl53l0x_settings_t* settings);

void vl53l0x_start_continous(uint8_t addr, vl53l0x_settings_t* settings, uint32_t period_ms);

uint16_t vl53l0x_get_range(uint8_t addr);

void vl53l0x_set_gpio(uint8_t addr, vl53l0x_gpio_enum mode, uint8_t polarity);

void vl53l0x_single_refcal(uint8_t addr, uint8_t vhv);

uint32_t vl53l0x_get_timing_budget(uint8_t addr);

void vl53l0x_set_timing_budget(uint8_t addr, uint32_t budget_us);

void vl53l0x_set_refspad(uint8_t addr, uint8_t spad_count, uint8_t spad_is_aperture, uint8_t* spad_ref_map);

uint8_t vl53l0x_get_vcsel(uint8_t addr, vl53l0x_vcsel_type_enum type);

void vl53l0x_get_sequence_enables(uint8_t addr, vl53l0x_sequence_step_enables_t* enables);

void vl53l0x_get_sequence_timeouts(uint8_t addr,  vl53l0x_sequence_step_enables_t* enables, vl53l0x_sequence_step_timeouts_t* timeouts);

void vl53l0x_get_spadinfo(uint8_t addr, uint8_t* count, uint8_t* is_aperture);

/*********************************************************************************************/
/*                                          REGISTER MAP                                     */
/*********************************************************************************************/
//Stolen from pololus arduino library...
typedef enum {
	VL53L0X_REG_SYSRANGE_START                              = 0x00,

	VL53L0X_REG_SYSTEM_THRESH_HIGH                          = 0x0C,
	VL53L0X_REG_SYSTEM_THRESH_LOW                           = 0x0E,

	VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG                      = 0x01,
	VL53L0X_REG_SYSTEM_RANGE_CONFIG                         = 0x09,
	VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

	VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

	VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

	VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

	VL53L0X_REG_RESULT_INTERRUPT_STATUS                     = 0x13,
	VL53L0X_REG_RESULT_RANGE_STATUS                         = 0x14,

	VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
	VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
	VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
	VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
	VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

	VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

	VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

	VL53L0X_REG_MSRC_CONFIG_CONTROL                         = 0x60,

	VL53L0X_REG_PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
	VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
	VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
	VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

	VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
	VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
	VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
	VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

	VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
	VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

	VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
	VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
	VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

	VL53L0X_REG_SYSTEM_HISTOGRAM_BIN                        = 0x81,
	VL53L0X_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
	VL53L0X_REG_HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

	VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
	VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
	VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
	VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

	VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

	VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
	VL53L0X_REG_IDENTIFICATION_MODEL_ID                     = 0xC0,
	VL53L0X_REG_IDENTIFICATION_REVISION_ID                  = 0xC2,

	VL53L0X_REG_OSC_CALIBRATE_VAL                           = 0xF8,

	VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
	VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
	VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
	VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
	VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
	VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
	VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

	VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
	VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
	VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
	VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

	VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

	VL53L0X_REG_ALGO_PHASECAL_LIM                           = 0x30,
	VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30
} vl53l0x_registers;

