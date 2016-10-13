//1. VL53L0X
//2. Sonar
//3. Start
//4. fancy
#include <stdint.h>

typedef enum{
	VL53L0X_MODE_SINGLE			= 0,
	VL53L0X_MODE_CONTINIUS 		= 1,
	VL53L0X_MODE_CONTINIUS_TIMED= 3
} vl53l0x_modes_enum;

typedef enum{
	VL53L0X_GPIO_OFF 		= 0,
	VL53L0X_GPIO_THRH_LOW 	= 1,
	VL53L0X_GPIO_THRH_HIGH 	= 2,
	VL53L0X_GPIO_THRH_OUT 	= 3,
	VL53L0X_GPIO_INTERRUPT	= 4,
} vl53l0x_gpio_enum;

typedef enum{
	VL53L0X_GPIO_OFF 		= 0,
	VL53L0X_GPIO_THRH_LOW 	= 1,
	VL53L0X_GPIO_THRH_HIGH 	= 2,
	VL53L0X_GPIO_THRH_OUT 	= 3,
	VL53L0X_GPIO_INTERRUPT	= 4,
} vl53l0x_gpio_enum;

typedef enum{
	VL53L0X_1V8_MODE,
	VL53L0X_2V8_MODE
} vl53l0x_voltage_enum;

#define VL53L0X_GPIO_POLARITY_POS 0
#define VL53L0X_GPIO_POLARITY_NEG 1

#define VL53L0X_DEFAULT_ADDR 0x52

/*Low level i2c functions*/
void vl53l0x_write_register(uint8_t addr, uint8_t register,  uint8_t value);
uint8_t vl53l0x_read_register(uint8_t addr, uint8_t register);

void vl53l0x_write_multi(uint8_t addr, uint8_t register, uint8_t* data, uint8_t count);
uint8_t vl53l0x_read_multi(uint8_t addr, uint8_t register, uint8_t* data, uint8_t count);

/*Init and calibration functions*/

void vl53l0x_data_init(uint8_t addr, vl53l0x_voltage_enum voltage);

void vl53l0x_static_init(uint8_t addr);

/*Config functions*/

void vl53l0x_config_addr(uint8_t addr, uint8_t new_addr);

void vl53l0x_config_mode(uint8_t addr, vl53l0x_modes_enum mode);

void vl53l0x_config_gpio(uint8_t addr, vl53l0x_gpio_enum mode, uint8_t polarity);

//Returns true if the sensor is on fire.
uint8_t vl53l0x_is_on_fire(uint8_t addr);

/*Ranging functions*/

void vl53l0x_measurement_start(uint8_t addr);

uint8_t vl53l0x_measurement_ready(uint8_t addr);

uint16_t vl53l0x_measurement_data(uint8_t addr);

void vl53l0x_measurement_clear(uint8_t addr);

void vl53l0x_measurement_stop(uint8_t addr);

uint8_t vl53l0x_measurement_stopped(uint8_t addr);

/*********************************************************************************************/
/*                                          REGISTER MAP                                     */
/*********************************************************************************************/
//Stolen from pololus arduino library...
typedef enum {
	SYSRANGE_START                              = 0x00,

	SYSTEM_THRESH_HIGH                          = 0x0C,
	SYSTEM_THRESH_LOW                           = 0x0E,

	SYSTEM_SEQUENCE_CONFIG                      = 0x01,
	SYSTEM_RANGE_CONFIG                         = 0x09,
	SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

	SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

	GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

	SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

	RESULT_INTERRUPT_STATUS                     = 0x13,
	RESULT_RANGE_STATUS                         = 0x14,

	RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
	RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
	RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
	RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
	RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

	ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

	I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

	MSRC_CONFIG_CONTROL                         = 0x60,

	PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
	PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
	PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
	PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

	FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
	FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
	FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
	FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

	PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
	PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

	PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

	SYSTEM_HISTOGRAM_BIN                        = 0x81,
	HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
	HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

	FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
	CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

	MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

	SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
	IDENTIFICATION_MODEL_ID                     = 0xC0,
	IDENTIFICATION_REVISION_ID                  = 0xC2,

	OSC_CALIBRATE_VAL                           = 0xF8,

	GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

	GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
	DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
	DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
	POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

	VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

	ALGO_PHASECAL_LIM                           = 0x30,
	ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30
} vl53l0x_registers;