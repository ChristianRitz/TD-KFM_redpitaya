from registers import *

gui_setup = {
    'title' : 'RedPitaya Control',
    'subtitle' : 'Time-Domain Controller for FM-KFM',
    'placeholder' : 'Nanotechnology Group ETH Zurich 2020',
    'maincolor' : 'firebrick',
}


gui_elements = {    
    'Start' : {
        'run program' : {
            'reg' : FLAGS_RUN_BIT,
            'type' : 'Checkbox',
        },

        'reset estimations' : {
            'reg' : FLAGS_RESET_ESTIMATOR_BIT,
            'type' : 'Checkbox',
        },
    },


    'Bias Configuration' : {
        'bias modulation' : {
            'reg' : FLAGS_ACTIVATE_BIAS_MOD_BIT,
            'type' : 'Checkbox',
        },

        'U ac' : {
            'reg': INDEX_UAC,
            'unit' : 'V',
            'type' : 'Entry',
        },
        'f mod' : {
            'reg': INDEX_FMOD,
            'unit' : 'Hz',
            'type' : 'Entry',
        },

        'U dc limit center' : {
            'reg' : INDEX_CPD_CENTER,
            'unit' : 'V',
            'type' : 'Entry',
        },  

        'U dc limit range' : {
            'reg' : INDEX_CPD_RANGE,
            'unit' : 'V',
            'type' : 'Entry',
        },  
    },

    'State Observer' : {
        'enable state observer' : {
            'reg' : FLAGS_KFM_ENABLE_BIT,
            'type' : 'Checkbox',
        },

        'Q topo' : {
            'reg' : INDEX_QTOPO,
            'unit' : 'Hz^2 Hz',
            'type' : 'Entry',
        },

        'Q cpd' : {
            'reg' : INDEX_QCPD,
            'unit' : 'V^2 Hz',
            'type' : 'Entry',
        },

        'Q a' : {
            'reg' : INDEX_QCOEFFA,
            'unit' : 'Hz^2/V^4 Hz',
            'type' : 'Entry',
        },

        'Rth' : {
            'reg' : INDEX_RTH,
            'unit' : 'Hz',
            'type' : 'Entry',
        },

        'Rd' : {
            'reg' : INDEX_RD,
            'unit' : 'Hz',
            'type' : 'Entry',
        },

        'measure sampling rate' : {
            'reg' : FLAGS_UPDATE_SAMPLERATE,
            'type' : 'Checkbox',
        },

        'sampling rate' : {
            'reg' : INDEX_SAMPLINGRATE,
            'unit' : 'Hz',
            'type' : 'Entry',
        },

        'PLL bw' : {
            'reg' : INDEX_PLLBW,
            'unit' : 'Hz',
            'type' : 'Entry',
        },
    },

    'Input/Output Gains':{
        'RF-in gain df' : {
            'reg' : INDEX_RFIN_GAIN_DF,
            'unit' : 'Hz/V',
            'type' : 'Entry',
        },

        'RF-in gain U bias' : {
            'reg' : INDEX_RFIN_GAIN_BIAS,
            'unit' : 'V/V',
            'type' : 'Entry',
        },

        'RF-out gain df topo' : {
            'reg' : INDEX_RFOUT_GAIN_DF,
            'unit' : 'V/Hz',
            'type' : 'Entry',
        },

        'RF-out gain U bias' : {
            'reg' : INDEX_RFOUT_GAIN_BIAS,
            'unit' : 'V/V',
            'type' : 'Entry',
        },

        'AnalogOut df topo' : {
            'reg' : INDEX_ANALOGOUT_GAIN_DFTOPO,
            'unit' : 'V/Hz',
            'type' : 'Entry',
        },

        'AnalogOut U cpd' : {
            'reg' : INDEX_ANALOGOUT_GAIN_CPD,
            'unit' : 'V/V',
            'type' : 'Entry',
        },

        'AnalogOut coeff a' : {
            'reg' : INDEX_ANALOGOUT_GAIN_COEFF_A,
            'unit' : 'V/(HzV^-2)',
            'type' : 'Entry',
        },

        'AnalogOut Error sq' : {
            'reg' : INDEX_ANALOGOUT_GAIN_ERRORSQ,
            'unit' : 'V/Hz^2',
            'type' : 'Entry',
        },
    }, 
}


