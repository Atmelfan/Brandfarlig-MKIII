#include "gpa_pca9534.h"

uint8_t send[2];
void pca9534_write_register(uint8_t addr, uint8_t reg, uint8_t value){
	send[0] = reg;
	send[1] = value;
	twi_send(addr, send, 2);
	while(!twi_ready());

}

uint8_t pca9534_read_register(uint8_t addr, uint8_t reg){
	send[0] = reg;
	uint8_t ret;
	twi_send_read(addr, send, 1, &ret, 1);
	while(!twi_ready());
	return ret;
}

void pca9534_init(uint8_t addr){

}

/*Clears ouput, sets all pins to inputs, clears polarity*/
void pca9534_reset(uint8_t addr){
	pca9534_out(addr, 0x00);
	pca9534_dir(addr, 0xFF);
	pca9534_polarity(addr, 0x00);
}

/*Reads input register*/
uint8_t pca9534_in(uint8_t addr){
	return pca9534_read_register(addr, PCA9534_REG_INPUT);
}

/*Writes $out to output*/
void pca9534_out(uint8_t addr, uint8_t out){
	pca9534_write_register(addr, PCA9534_REG_OUTPUT, out);
}

void pca9534_dir(uint8_t addr, uint8_t dir){
	pca9534_write_register(addr, PCA9534_REG_CONFIG, ~dir);//Bit high for output, low for input
}

/*Sets polarity*/
void pca9534_polarity(uint8_t addr, uint8_t pol){
	pca9534_write_register(addr, PCA9534_REG_POLARITY, pol);
}

/*Sets bits of $set on output*/
void pca9534_set(uint8_t addr, uint8_t set){
	uint8_t s = pca9534_read_register(addr, PCA9534_REG_OUTPUT);
	pca9534_out(addr, s | set);
}

/*Clears bits of $set on output*/
void pca9534_clr(uint8_t addr, uint8_t clr){
	uint8_t s = pca9534_read_register(addr, PCA9534_REG_OUTPUT);
	pca9534_out(addr, s & ~clr);
}

