#include <stdio.h>
#include <stdlib.h>
#include <iostream> 
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

#include "parameter_readout.h"



int main(int argc, char** argv){
    //initialize memory pointer to registers
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    uint32_t mode, a_addr, i_value, param_idx, flags;
    float f_value;

    if (argc > 1) {
        mode = strtoul(argv[1], 0, 0);
    }

    switch (mode)
    {
    case 0:
        //read value from register
        a_addr = strtoul(argv[2], 0, 0);
        i_value = RP_RegGet(a_addr);
        std::cout<< i_value <<std::endl;
        break;

    case 1:
        //write value to register
        a_addr = strtoul(argv[2], 0, 0);
        i_value =  strtoul(argv[3], 0, 0);
        RP_RegSet(a_addr,i_value);
        break;

    case 2:
        //readout multiple registers
        param_idx = strtoul(argv[2],0,0);
        //fill index register
        RP_RegSet(REG_PARAMS_INDEX, param_idx);
        //flip trigger
        flags = RP_RegGet(REG_FLAGS);
        flags |= (1 << FLAGS_READRQ_BIT);
        RP_RegSet(REG_FLAGS, flags);
        while(true){
            sleep(0.05);
            if (!(RP_RegGet(REG_FLAGS) & (1 << FLAGS_READRQ_BIT))) {
                break;
            }
        }
        f_value = multireg_to_parameter();
        std::cout<< f_value;
        break;

    case 3:
        //write multiple parameters
        f_value = atof(argv[2]);
        param_idx = strtoul(argv[3],0,0);
        //fill multireg
        parameter_to_multireg(f_value);
        //fill index register
        RP_RegSet(REG_PARAMS_INDEX, param_idx);
        //flip trigger
        flags = RP_RegGet(REG_FLAGS);
        flags |= (1 << FLAGS_UPDATERQ_BIT);
        RP_RegSet(REG_FLAGS, flags);
        while(true){
            sleep(0.05);
            if (!(RP_RegGet(REG_FLAGS) & (1 << FLAGS_UPDATERQ_BIT))) {
                break;
            }
        }
        break;
    
    default:
        break;
    }


    //close memory access
	if (map_base != (void*)(-1)) map_base = (void*)(-1);
	if (fd != -1) close(fd);

    return 0;
}