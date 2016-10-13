#define GVM_STACK_SIZE 32



typedef gvm_type uint32_t

gvm_type stack[GVM_STACK_SIZE];

uint16_t sp, pc;

void gvm_execute(uint32_t* bytecode, uint16_t _len){

}

void gvm_clear(){

}

void gvm_push(gvm_type a){

}

gvm_type gvm_pop(){

}

gvm_type gvm_peek(uint8_t i){

}


void op_add(uint8_t i, uint16_t o){
	gvm_type b = gvm_pop();
	gvm_type a = gvm_pop();
	gvm_push(a + b);
}

void op_sub(uint8_t i, uint16_t o){
	gvm_type b = gvm_pop();
	gvm_type a = gvm_pop();
	gvm_push(a - b);
}

void op_mul(uint8_t i, uint16_t o){
	gvm_type b = gvm_pop();
	gvm_type a = gvm_pop();
	gvm_push(a * b);
}

void op_div(uint8_t i, uint16_t o){
	gvm_type b = gvm_pop();
	gvm_type a = gvm_pop();
	gvm_push(a / b);
}

void op_neg(uint8_t i, uint16_t o){
	gvm_type a = gvm_pop();
	gvm_push(-a);
}

void op_jmp(uint8_t i, uint16_t o){
	gvm_push((gvm_type)pc);
}

void op_ret(uint8_t i, uint16_t o){
	gvm_push((gvm_type)pc);
}



