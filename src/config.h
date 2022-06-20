/**
 * @file config.h
 * @brief Defines the input and output channels used by the TD-KFM code.
 * @Author Nanotechnology Group ETH Zurich 2020
 */


#ifndef CONFIG_H
#define CONFIG_H

//redpitaya channels
#define RP_RF_IN_DF RP_CH_1
#define RP_RF_IN_BIAS RP_CH_2

#define RP_RF_OUT_DF RP_CH_1
#define RP_RF_OUT_BIAS RP_CH_2

//redpitaya analog channels
#define RP_AN_OUT_DFTOPO RP_AOUT0
#define RP_AN_OUT_CPD RP_AOUT1
#define RP_AN_OUT_COEFF_A RP_AOUT2
#define RP_AN_OUT_ERROR RP_AOUT3

#endif
