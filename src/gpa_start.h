#ifndef GPA_START_H_
#define GPA_START_H_
#include <stdint.h>

typedef enum
{
	START_POWER_ON 		= 0,//SAVE AS START_POWER_ON
	START_STARTED 		= 1,//SAVE AS START_STARTED
	START_STOPPED_SAFE 	= 2,//SAVE AS START_STOPPED_SAFE
	START_STOPPED 		= 3,//SAVE AS START_POWER_ON
	START_PROGRAMMING 	= 4 //SAVE AS START_POWER_ON
  //OTHER 					//SAVE AS START_POWER_ON
} start_state;

typedef struct __attribute__((__packed__))
{
	uint8_t status;
	uint8_t dohyo;
} start_savedata;

#define START_SIZEOF_SAVE (sizeof(start_savedata))

#define START_COOLDOWN 100

#define START_ADDR_PROGRAM 	0x0B	//Address for programming command
#define START_ADDR_START 	0x07	//Address for start command

//Position in memory to save last state (Not used here, use in implementation of get/set_saved_mode) 
//Nicer to collect all settings in the relevant file
#define START_SAVE_POSITION 0		

/*Called during init to retrieve mode from EEPROM*/
extern void start_get_saved(start_savedata* save);
/*Called during operation to save last mode in EEPROM*/
extern void start_set_saved(start_savedata* save);

extern void start_status_changed(start_state newstat);


/*Init from memory*/
void start_init();

/**/
void start_oncommand(uint8_t addr, uint8_t cmd);


start_state start_status();

uint8_t start_dohyo();

void start_update();

void start_reset();

#endif



















