#include "gpa_start.h"

start_savedata current = {
	.status = START_POWER_ON,
	.dohyo = 0
};

uint8_t cooldown;

void start_update(){
	if (current.status == START_STOPPED_SAFE && cooldown == 0){
		current.status = START_STOPPED;
		start_set_saved(&current);
	}else if (current.status == START_PROGRAMMING && cooldown == 0){
		current.status = START_POWER_ON;
		start_set_saved(&current);
	}

	if(cooldown > 0){
		cooldown--;
	}
}


void start_init(){
	start_savedata saved = current;
	start_get_saved(&saved); 
	start_state saved_status = saved.status;
	
	current.dohyo = saved.dohyo; 

	if (saved_status == START_STARTED){
		current.status = saved_status;		
	}else if(saved_status == START_STOPPED_SAFE){
		current.status = saved_status;		
		cooldown = START_COOLDOWN;
	}else{
		current.status = START_POWER_ON;
	}
}


/**/
void start_oncommand(uint8_t addr, uint8_t cmd){
	if (addr == START_ADDR_PROGRAM && (current.status == START_POWER_ON || current.status == START_STARTED)){
		uint8_t new_dohyo = cmd & 0xFE;
		if (new_dohyo != current.dohyo)
		{
			current.status = START_PROGRAMMING;
			cooldown = START_COOLDOWN;
			current.dohyo = new_dohyo;
			start_set_saved(&current);//Save new dohyo and 
		}
	}else if(addr == START_ADDR_START){
		if((cmd & 0xFE) == current.dohyo){
			if(cmd & 0x01){
				if (current.status == START_POWER_ON){
					current.status = START_STARTED;
				}
				
			}else{
				current.status = START_STOPPED_SAFE;
				cooldown = START_COOLDOWN;
			}
			start_set_saved(&current);//Save new status
		}
		//If not our dohyo, ignore it.
	}
	//Not a start command
}

void start_reset(){
	if(current.status == START_STOPPED){
		current.status = START_POWER_ON;
	}
}

start_state start_status(){
	return current.status;
}

uint8_t start_dohyo(){
	return current.dohyo;
}
