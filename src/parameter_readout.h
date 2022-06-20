#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "registers.h"


union float_uint32_union_t {
    float f;
    unsigned u;
};

//setup memory map for register readout
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
void* map_base = (void*)(-1);
int fd = -1;



///////////////////////////////////////////////////////
//                 FUNCTIONS                         //
///////////////////////////////////////////////////////

/**
 * @brief Reads out the value of a register.
 *
 * @param a_addr Address to register (e.g. 0x40000030 for LED).
 * @return integer value saved in this register
 */
uint32_t RP_RegGet(uint32_t a_addr) {
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, a_addr & ~MAP_MASK);
	void* virt_addr = (uint8_t*)map_base + (a_addr & MAP_MASK);
    uint32_t read_result = *((uint32_t *) virt_addr);
	return read_result;
}

/**
 * @brief Sets a register to a certain value.
 *
 * @param a_addr Address to register (e.g. 0x40000030 for LED)
 * @param a_value unsigned long that is written to register
 */
void RP_RegSet(uint32_t a_addr, uint32_t a_value) {
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, a_addr & ~MAP_MASK);
	void* virt_addr = (uint8_t*)map_base + (a_addr & MAP_MASK);
    *((uint32_t *) virt_addr) = a_value;
}


/**
 * @brief Splits a 32-bit float into two 14-bit integers (v2 and v3) and one 4-bit integer (v1).
 * These bits are written to the 14-bit registers REG_PARAMS and REG_PARAMS2, and to the last 4 bits of REG_FLAGS
 *
 * @param value 32-bit input which is written to three different registers.
 */
void parameter_to_multireg(float value){
    float_uint32_union_t uf;
    uf.f = value;
    uint32_t v1 = (uf.u & 0xF0000000)>>2*14;
    uint32_t v2 = (uf.u & 0x0FFFC000)>>14;
    uint32_t v3 = (uf.u & 0x00003FFF);

    uint32_t tmp = RP_RegGet(REG_FLAGS);
    RP_RegSet(REG_FLAGS, (tmp&0x3FF) + (v1 << 10));
    RP_RegSet(REG_PARAMS, v2);
    RP_RegSet(REG_PARAMS2, v3);
}


/**
 * @brief Reads the 14-bit registers REG_PARAMS and REG_PARAMS2, and the last 4 bits of REG_FLAGS.
 * The three numbers are combined to one 32 bit float.
 * 
 * @return 32 bit float read from three different registers.
 */
float multireg_to_parameter(){
    uint32_t v1 = (RP_RegGet(REG_FLAGS) & 0x3C00)>>10;
    uint32_t v2 = RP_RegGet(REG_PARAMS);
    uint32_t v3 = RP_RegGet(REG_PARAMS2);

    //combine 4bit, 14bit, and 14bit value to integer value
    float_uint32_union_t uf;
    uf.u = (v1<<2*14) + (v2<<14) + v3;
    return uf.f;
}