/**
 * @file registers.h
 * @brief Defines all register addresses used to setup and control the TD-KFM code.
 * @Author Christian Ritz Nanotechnology Group ETH Zurich 2020
 * 
 * The controller uses the four registers reserved for the PID setpoints of fpga_0.94.bit.
 * The PIDs must not be used while the TD-KFM loop is running.
 * Register map see: https://redpitaya.readthedocs.io/en/latest/developerGuide/software/build/fpga/regset_common.html
 * 
 * The four registers have a size of 14 bits.
 * 
 * A more elegant solution would be to make the FREE space on 0x406XXXXXX available.
 * However, this requires modification to the FPGA file.
 * 
 * The four available 14-bit registers are used as follows:
 * 
 *   - flags:               used for bool values             
 *                          --> uses bits 0-9 of REG_FLAGS
 * 
 *   - parameter transfer:  used to transfer a 32-bit float from and to the device
 *                          --> uses bits 10-13 of REG_FLAGS, and registers REG_PARAMS and REG_PARAMS2
 * 
 *   - parameter index:     used to specify which parameter is transferred
 *                          --> uses REG_PARAMS_INDEX
 */



#ifndef REGISTERS_H
#define REGISTERS_H




///////////////////////////////////////////////////////
//                 REGISTERS                         //
/////////////////////////////////////////////////////// 

//led register
#define REG_LED     0x40000030

//four bits of this register are reserved for parameter exchange, ten bits are reserved for controller flags
#define REG_FLAGS   0x40300010

//registers used to change/readout parameter on C-Code
#define REG_PARAMS     0x40300020
#define REG_PARAMS2    0x40300030

//register defining which parameter is read/written
#define REG_PARAMS_INDEX     0x40300040







///////////////////////////////////////////////////////
//                 FLAGS                             //
///////////////////////////////////////////////////////

//uses register REG_FLAGS to set the controller flags
//10 out of 14 bits are available for flags --> use 0-9

//terminate C++ program if zero
#define FLAGS_RUN_BIT 0

//update request bit
#define FLAGS_UPDATERQ_BIT 1 

//read request bit
#define FLAGS_READRQ_BIT 2

//enable TD-KFM
#define FLAGS_KFM_ENABLE_BIT 3

//activate bias modulation
#define FLAGS_ACTIVATE_BIAS_MOD_BIT 4

//reset output if flag is enabled
#define FLAGS_RESET_ESTIMATOR_BIT 5

//readout sample rate flag
#define FLAGS_UPDATE_SAMPLERATE 6





///////////////////////////////////////////////////////
//                 PARAMETER UPDATE RQ               //
///////////////////////////////////////////////////////

//uses register REG_PARAMS_INDEX
//defines parameter number which is updated (read out) upon update- (readout-) request
//size = 14 bit --> 2**14 numbers available 

//parameter registers
#define INDEX_QTOPO   0
#define INDEX_QCPD    1
#define INDEX_QCOEFFA 2

//input registers
#define INDEX_SAMPLINGRATE    3
#define INDEX_PLLBW   4

//noise registers
#define INDEX_RTH     5
#define INDEX_RD      6

//bias configuration
#define INDEX_UAC       7
#define INDEX_FMOD      8

//cpd limits
#define INDEX_CPD_CENTER      9
#define INDEX_CPD_RANGE      10

//gain registers
#define INDEX_RFIN_GAIN_DF     11
#define INDEX_RFIN_GAIN_BIAS   12
#define INDEX_RFOUT_GAIN_DF   13
#define INDEX_RFOUT_GAIN_BIAS    14
#define INDEX_ANALOGOUT_GAIN_DFTOPO    15
#define INDEX_ANALOGOUT_GAIN_CPD       16
#define INDEX_ANALOGOUT_GAIN_COEFF_A    17
#define INDEX_ANALOGOUT_GAIN_ERRORSQ    18

#endif