#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "rp.h"
#include "registers.h"
#include "config.h"
#include "OptimizedKelvinEstimatorEKF.h"
#include "parameter_readout.h"
#include <chrono>


///////////////////////////////////////////////////////
//                 BASIC                             //
///////////////////////////////////////////////////////

//setup KFM controller
typedef OptimizedKelvinEstimatorEKF<float> OptimizedKelvinEstimatorEKF_t;
OptimizedKelvinEstimatorEKF_t kelvin_estimator;


//sample counter and parameter update period
int update_count = 0;
int led_counter = 0;
//perform update roughly every 100ms -> 20k
int update_period_active = 20000;      
//increase update period if KFM loop is not running -> segmentation fault occurs otherwise
int update_period_passive = update_period_active*200;    
int led_status = 0;

//initialize global variables with default parameters

struct FilterParameters {
    float Q_topo = 0.3;
    float Q_cpd = 0.002; 
    float Q_a = 1.0;
    float Rth = 0.000738;
    float Rd = 0.0000001;
    float samplingrate = 140000;
    float pll_bw = 800;
} KalmanFilter;

struct TipVoltage {
    float ac = 1;
    float fmod = 1000; 
} Bias;

struct InputOutputGains {
    //output gains for RF channels
    float out_topo = 0.01; // V_output/Hz
    float out_bias = 0.1; // V_output/V_cpd

    //output gains for analog channels
    float analog_out_df = 0.1;
    float analog_out_cpd = 0.1;
    float analog_out_a = 0.05;
    float analog_out_error = 0.5;
    
    //input gains for RF channels
    float in_df = 10; // Hz/V_input
    float in_bias = 1; // V_bias/V_input
} IOGains;

//limit a variable in range
#define LIMIT_RANGE(x, a, b) ((x)>(b)?(b):((x)<(a)?(a):(x)))

//initialize limits for cpd
float cpd_center = 0.001; //V
float cpd_range = 1; //V
float cpd_upper_limit = cpd_center + cpd_range;
float cpd_lower_limit = cpd_center - cpd_range;

//setup global variables
float frequency_shift, df_topo;
float tip_voltage;
float cpd_out, cpd;
unsigned long  controller_flags;
unsigned long  controller_flags_old;

auto start = std::chrono::steady_clock::now();
auto end = std::chrono::steady_clock::now();

uint32_t buff_size = 1; 

///////////////////////////////////////////////////////
//                 FUNCTIONS                         //
///////////////////////////////////////////////////////

/**
 * @brief Lights up the LEDs three times.
 */
void led_signal_start() {
    for(int k=0;k<3;k++){
        led_status = 0;
        for(int i=0;i<=8;i++) {
            led_status += pow(2,i);
            RP_RegSet(REG_LED, led_status);
            usleep(30000);
        }
    }
}



///////////////////////////////////////////////////////
//                 MAIN                              //
///////////////////////////////////////////////////////

/**
 * @brief Initialize RedPitaya and setup RF Input and Output
 */
void initialize_redpitaya() {
    //initialize redpitaya - necessary when using rp.h
	if(rp_Init() != RP_OK){
		fprintf(stderr, "Rp api init failed!\n");
	}

    //setup RF outputs
    rp_GenReset();
    rp_GenWaveform(RP_RF_OUT_DF, RP_WAVEFORM_DC);
    rp_GenAmp(RP_RF_OUT_DF, 0);
    rp_GenOffset(RP_RF_OUT_DF, 0);
	rp_GenOutEnable(RP_RF_OUT_DF);
    rp_GenWaveform(RP_RF_OUT_BIAS, RP_WAVEFORM_DC);
    rp_GenAmp(RP_RF_OUT_BIAS, 0);
    rp_GenOffset(RP_RF_OUT_BIAS, 0);
	rp_GenOutEnable(RP_RF_OUT_BIAS);

    //setup RF inputs
    rp_AcqReset();
    rp_AcqSetArmKeep(1);
    rp_AcqSetGain(RP_RF_IN_DF, RP_HIGH);
    rp_AcqSetGain(RP_RF_IN_BIAS, RP_HIGH);
    rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW); //sets trigger to immediately
    rp_AcqSetSamplingRate(RP_SMP_122_070K); //set sampling rate to 122 kHz
    rp_AcqSetAveraging(1);
    rp_AcqStart();
}


/**
 * @brief Update the estimator parameters.
 */
void set_Kalman_parameters() {
    kelvin_estimator.set_system_noise(
        KalmanFilter.Q_topo/KalmanFilter.samplingrate,
        KalmanFilter.Q_cpd/KalmanFilter.samplingrate, 
        KalmanFilter.Q_a/KalmanFilter.samplingrate
    );
    kelvin_estimator.set_frequency_noise(KalmanFilter.Rth*KalmanFilter.samplingrate);
    kelvin_estimator.set_measurement_noise(KalmanFilter.Rd*KalmanFilter.samplingrate);
    kelvin_estimator.setup_filter(KalmanFilter.pll_bw, KalmanFilter.samplingrate);
}

void update_parameters() {
    //request which parameter needs to be updated
    unsigned long param_idx = RP_RegGet(REG_PARAMS_INDEX);
    float param = multireg_to_parameter();

    switch(param_idx){
        case INDEX_QTOPO:
            KalmanFilter.Q_topo = param;
            set_Kalman_parameters();
            break;

        case INDEX_QCPD:
            KalmanFilter.Q_cpd = param;
            set_Kalman_parameters();
            break;

        case INDEX_QCOEFFA:
            KalmanFilter.Q_a = param;
            set_Kalman_parameters();
            break;

        case INDEX_SAMPLINGRATE:
            KalmanFilter.samplingrate = param;
            set_Kalman_parameters();
            break;

        case INDEX_PLLBW:
            KalmanFilter.pll_bw = param;
            set_Kalman_parameters();
            break;

        case INDEX_RTH:
            KalmanFilter.Rth = param;
            set_Kalman_parameters();
            break;

        case INDEX_RD:
            KalmanFilter.Rd = param;
            set_Kalman_parameters();
            break;

        case INDEX_RFIN_GAIN_DF:
            IOGains.in_df = param;
            break;

        case INDEX_RFIN_GAIN_BIAS:
            IOGains.in_bias = param;
            break;

        case INDEX_RFOUT_GAIN_DF:
            IOGains.out_topo = param;
            break;

        case INDEX_RFOUT_GAIN_BIAS:
            IOGains.out_bias = param;
            if(controller_flags & (1 << FLAGS_ACTIVATE_BIAS_MOD_BIT)){
                rp_GenAmp(RP_RF_OUT_BIAS, Bias.ac*IOGains.out_bias);
            }
            break;

        case INDEX_ANALOGOUT_GAIN_DFTOPO:
            IOGains.analog_out_df = param;
            break;

        case INDEX_ANALOGOUT_GAIN_CPD:
            IOGains.analog_out_cpd = param;
            break;

        case INDEX_ANALOGOUT_GAIN_COEFF_A:
            IOGains.analog_out_a = param;
            break;

        case INDEX_ANALOGOUT_GAIN_ERRORSQ:
            IOGains.analog_out_error = param;
            break;

        case INDEX_UAC:
            Bias.ac = param;
            if(controller_flags & (1 << FLAGS_ACTIVATE_BIAS_MOD_BIT)){
                rp_GenAmp(RP_RF_OUT_BIAS, Bias.ac*IOGains.out_bias);
            }
            break;

        case INDEX_FMOD:
            Bias.fmod = param;
            rp_GenFreq(RP_RF_OUT_BIAS, Bias.fmod);
            break;

        case INDEX_CPD_CENTER:
            cpd_center = param;
            cpd_upper_limit = cpd_center + cpd_range;
            cpd_lower_limit = cpd_center - cpd_range;
            break;

        case INDEX_CPD_RANGE:
            cpd_range = param;
            cpd_upper_limit = cpd_center + cpd_range;
            cpd_lower_limit = cpd_center - cpd_range;
            break;

        default:
            printf("no update register found for index %lu\n", param_idx);
    }

}


void emit_parameters(){
    unsigned long param_idx = RP_RegGet(REG_PARAMS_INDEX);
    float param;
    switch(param_idx){
    
        case INDEX_QTOPO:
            param = KalmanFilter.Q_topo;
            break;

        case INDEX_QCPD:
            param = KalmanFilter.Q_cpd;
            break;

        case INDEX_QCOEFFA:
            param = KalmanFilter.Q_a;
            break;

        case INDEX_SAMPLINGRATE:
            param = KalmanFilter.samplingrate;
            break;

        case INDEX_PLLBW:
            param = KalmanFilter.pll_bw;
            break;

        case INDEX_RTH:
            param = KalmanFilter.Rth;
            break;

        case INDEX_RD:
            param = KalmanFilter.Rd;
            break;

        case INDEX_RFIN_GAIN_DF:
            param = IOGains.in_df;
            break;
        
        case INDEX_RFIN_GAIN_BIAS:
            param = IOGains.in_bias;
            break;

        case INDEX_RFOUT_GAIN_DF:
            param = IOGains.out_topo;
            break;

        case INDEX_RFOUT_GAIN_BIAS:
            param = IOGains.out_bias;
            break;

        case INDEX_ANALOGOUT_GAIN_DFTOPO:
            param = IOGains.analog_out_df;
            break;

        case INDEX_ANALOGOUT_GAIN_CPD:
            param = IOGains.analog_out_cpd;
            break;

        case INDEX_ANALOGOUT_GAIN_COEFF_A:
            param = IOGains.analog_out_a;
            break;

        case INDEX_ANALOGOUT_GAIN_ERRORSQ:
            param = IOGains.analog_out_error;
            break;

        case INDEX_UAC:
            param = Bias.ac;
            break;

        case INDEX_FMOD:
            param = Bias.fmod;
            break;

        case INDEX_CPD_CENTER:
            param = cpd_center;
            break;

        case INDEX_CPD_RANGE:
            param = cpd_range;
            break;

        default:
            printf("no update register found for index %lu\n", param_idx);

    }
    parameter_to_multireg(param);
}



/**
 * @brief Read out the controller flag. If requested, perform update, readout or reset.
 */
void check_controller_flags() {
    controller_flags_old = controller_flags;
    controller_flags = RP_RegGet(REG_FLAGS);
    
    if (((controller_flags_old & 0x3FF) == (controller_flags & 0x3FF)) &&
        !(controller_flags & (1 << FLAGS_UPDATE_SAMPLERATE))){
        //if controller flags have not changed, return
        //exception: update sample rate flag
        return;
    }

    //check if a parameter update is requested
    if (controller_flags & (1 << FLAGS_UPDATERQ_BIT)) {
        update_parameters();
        controller_flags = RP_RegGet(REG_FLAGS);
        controller_flags &= ~(1 << FLAGS_UPDATERQ_BIT);
        RP_RegSet(REG_FLAGS, controller_flags);
    }

    //check if a parameter readout is requested
    if (controller_flags & (1 << FLAGS_READRQ_BIT)) {
        emit_parameters();
        controller_flags = RP_RegGet(REG_FLAGS);
        controller_flags &= ~(1 << FLAGS_READRQ_BIT);
        RP_RegSet(REG_FLAGS, controller_flags);
    }

    //check if a reset of the estimator is requested
    if (controller_flags & (1 << FLAGS_RESET_ESTIMATOR_BIT)) {
        kelvin_estimator.reset_estimation();
        controller_flags = RP_RegGet(REG_FLAGS);
        controller_flags &= ~(1 << FLAGS_RESET_ESTIMATOR_BIT);
        RP_RegSet(REG_FLAGS, controller_flags);
    }

    //check if the KFM controller was just enabled
    if (!(controller_flags_old & (1 << FLAGS_KFM_ENABLE_BIT)) &&
        (controller_flags & (1 << FLAGS_KFM_ENABLE_BIT))) {
        //signal start with leds
        led_signal_start();
    }

    //check if KFM controller was just disabled
    if ((controller_flags_old & (1 << FLAGS_KFM_ENABLE_BIT)) &&
        !(controller_flags & (1 << FLAGS_KFM_ENABLE_BIT))) {
        //turn off leds
        RP_RegSet(REG_LED, 0);
    }

    //check if bias modulation was just enabled
    if (!(controller_flags_old & (1 << FLAGS_ACTIVATE_BIAS_MOD_BIT)) &&
        (controller_flags & (1 << FLAGS_ACTIVATE_BIAS_MOD_BIT))) {
        //activate bias modulation
        rp_GenWaveform(RP_RF_OUT_BIAS, RP_WAVEFORM_SINE);
        rp_GenFreq(RP_RF_OUT_BIAS, Bias.fmod);
        rp_GenAmp(RP_RF_OUT_BIAS, Bias.ac*IOGains.out_bias);
    }

    //check if bias modulation was just disabled
    if ((controller_flags_old & (1 << FLAGS_ACTIVATE_BIAS_MOD_BIT)) &&
        !(controller_flags & (1 << FLAGS_ACTIVATE_BIAS_MOD_BIT))) {
        //disable bias modulation
        rp_GenAmp(RP_RF_OUT_BIAS, 0);
        rp_GenWaveform(RP_RF_OUT_BIAS, RP_WAVEFORM_DC);
    }

    //measurement of samplerate is triggered
    if (controller_flags & (1 << FLAGS_UPDATE_SAMPLERATE)) {
        if (!(controller_flags_old & (1 << FLAGS_KFM_ENABLE_BIT))){
            //loop not enabled: skip
            controller_flags &= ~(1 << FLAGS_UPDATE_SAMPLERATE);
            RP_RegSet(REG_FLAGS, controller_flags);
            return;
        }

        if (!(controller_flags_old & (1 << FLAGS_UPDATE_SAMPLERATE))){
            //sample rate update was just started
            start = std::chrono::steady_clock::now();
        }
        else{
            //sample rate update has been started before, evaluate now
            end = std::chrono::steady_clock::now();
            std::chrono::duration<double> cycletime = end - start;
            KalmanFilter.samplingrate = update_period_active/cycletime.count();
            set_Kalman_parameters();
            controller_flags &= ~(1 << FLAGS_UPDATE_SAMPLERATE);
            controller_flags_old &= ~(1 << FLAGS_UPDATE_SAMPLERATE);
            RP_RegSet(REG_FLAGS, controller_flags);
        }
    }

}




/**
 * @brief Main loop
 */
int main(int argc, char **argv) {

    //initialize memory pointer to registers
    fd = open("/dev/mem", O_RDWR | O_SYNC);

    //initialize in- and outputs of the device
    initialize_redpitaya();
    
    //set all control register to false, except RUN
    controller_flags = controller_flags_old = 0;
    controller_flags |= (1 << FLAGS_RUN_BIT);
    RP_RegSet(REG_FLAGS,controller_flags);
    
    //setup Kalman filter
    set_Kalman_parameters();

    //enter infinity loop - ends if RUN is set to false
	while (controller_flags & (1 << FLAGS_RUN_BIT)) {

        //check if controller flags have changed once every update period
		if (((update_count > update_period_active) && (controller_flags & (1 << FLAGS_KFM_ENABLE_BIT))) ||
		   ((update_count > update_period_passive)  && !(controller_flags & (1 << FLAGS_KFM_ENABLE_BIT)))) {
			check_controller_flags();

            //led heartbeat
            if ((led_status == 0) && (controller_flags & (1 << FLAGS_KFM_ENABLE_BIT))) {
                led_status = 255;
                RP_RegSet(REG_LED, led_status);
            }
            if (led_counter > 10) {   
                led_status = 0;
                RP_RegSet(REG_LED, led_status);
                led_counter = 0;
            }
            update_count=0;
            led_counter++;
		}

		if (controller_flags & (1 << FLAGS_KFM_ENABLE_BIT)) {
			//get sample
			rp_AcqGetLatestDataV(RP_RF_IN_DF, &buff_size, &frequency_shift);
			rp_AcqGetLatestDataV(RP_RF_IN_BIAS, &buff_size, &tip_voltage);
            frequency_shift *= IOGains.in_df;
            tip_voltage *= IOGains.in_bias;

			//do update
			kelvin_estimator.update(frequency_shift, tip_voltage);

			//write RF output
            df_topo = kelvin_estimator.get_df_stat();
            cpd = kelvin_estimator.get_cpd();
            cpd_out = LIMIT_RANGE(cpd, cpd_lower_limit, cpd_upper_limit);
            rp_GenOffset(RP_RF_OUT_DF, df_topo * IOGains.out_topo);
            rp_GenOffset(RP_RF_OUT_BIAS, cpd_out * IOGains.out_bias);

            //write analog output
            rp_ApinSetValue(RP_AN_OUT_DFTOPO, df_topo * IOGains.analog_out_df);
            rp_ApinSetValue(RP_AN_OUT_CPD, cpd * IOGains.analog_out_cpd);
            rp_ApinSetValue(RP_AN_OUT_COEFF_A, kelvin_estimator.get_coeff_a() * IOGains.analog_out_a);
            rp_ApinSetValue(RP_AN_OUT_ERROR, kelvin_estimator.get_df_error() * IOGains.analog_out_error);
        }
		update_count++;
	}

    //releasing resources
    rp_GenOffset(RP_RF_OUT_DF, 0);
    rp_GenOffset(RP_RF_OUT_BIAS, 0);
    rp_ApinSetValue(RP_AN_OUT_DFTOPO, 0);
    rp_ApinSetValue(RP_AN_OUT_CPD, 0);
    rp_ApinSetValue(RP_AN_OUT_COEFF_A, 0);
    rp_ApinSetValue(RP_AN_OUT_ERROR, 0);
    
    //disable bias modulation
    rp_GenAmp(RP_RF_OUT_BIAS, 0);
    rp_GenWaveform(RP_RF_OUT_BIAS, RP_WAVEFORM_DC);

    rp_Release();

    //close memory access
	if (map_base != (void*)(-1)) map_base = (void*)(-1);
	if (fd != -1) close(fd);

    return 0;
}