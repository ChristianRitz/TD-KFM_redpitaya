Time-domain KFM Controller for RedPitaya STEMLab 125-14
=========================================

This is a controller for time-domain Kelvin probe force microscopy (TD-KFM), designed for the RedPitaya STEMLab 125-14. 
For more information regarding TD-KFM, we would like to refer you to Beilstein J. Nanotechnol. 2020, 11, 911–921 https://doi.org/10.3762/bjnano.11.76.
Please consider citing this publication if you use the TD-KFM controller for your research.

    @article{ritz2020measurement,
        title={Measurement of electrostatic tip--sample interactions by time-domain Kelvin probe force microscopy},
        author={Ritz, Christian and Wagner, Tino and Stemmer, Andreas},
        journal={Beilstein journal of nanotechnology},
        volume={11},
        number={1},
        pages={911--921},
        year={2020},
        publisher={Beilstein-Institut}
    }

Structure
-----------

- **config/**: 
    - default location to save and load scan parameters; 
    - contains access information for ssh connection used by python gui
- **src/**: source code for the main loop
- **include/**: Kalman filter used as state observer
- **interface/**: python cli and gui used to start, setup, and terminate the program on the RedPitaya


Environment and Compilation
-----------

The controller has been tested on the STEMLab 125-xx OS 1.04-7.
The code uses the Eigen template library for all matrix arithmetics and heavily benefits from optimizations.
Tested with gcc 5.4.0, and `-O3` optimization.

The controller can be compiled directly on the RedPitaya, using its default gcc compiler.



Initial setup on RedPitaya
-----------

Set up the OS of the RedPitaya, and make sure that the input jumpers are set to HV (±20V).
You might need to calibrate your device.

Then, connect to the device via ssh, and perform the following steps.

1. Download the Eigen library to the root directory of the RedPitaya.
```
$ cd /root
$ git clone https://gitlab.com/libeigen/eigen.git
```

2. Download the files from this repository.
```
$ cd /root
$ git clone https://github.com/ChristianRitz/TD-KFM_redpitaya.git
```

3. Compile the source code.
```
$ cd /root/TD-KFM_redpitaya/src
$ make
```



Default configuration
---------------------

By default, the controller assumes the following configuration:

- **RF Input 1**: frequency shift obtained from the PLL, df
    - Note, that the correct input gain [Hz/V] needs to be set.
    - The program assumes that the jumpers are set to HV input (±20V).
- **RF Input 2**: tip-sample bias voltage U<sub>ts</sub>
    - The program assumes that the jumpers are set to HV input (±20V).
- **RF Output 1**: topography induced frequency shift df<sub>topo</sub>
    - Use df_topo for full electrostatic compensation. Note, that the tip may crash if the controller loses its stability. We strongly advise the use of a tip-protection algorithm.
    - The output gain [V/Hz] can be set in the gui. The output is limited to ±1V.
- **RF Output 2**: estimated surface potential U<sub>dc</sub> (default), or full bias voltage U<sub>ts</sub> = U<sub>dc</sub> + U<sub>ac</sub> sin(2pi f<sub>mod</sub> t)
    - If the tip-sample bias is modulated externally, U<sub>dc</sub> can be added to the tip-sample bias to reduce the electrostatic influence.
    - The output gain [V/V] can be set in the gui. Note, that the output is limited to ±1V. 
      Depending on the application, this output might require amplification. By default, the RF-out gain U<sub>bias</sub> is set to 0.1 V/V and requires external x10 amplification.
      If ±1V is sufficient for your application, you may set the output gain to 1 V/V.

- **Analog Output 1**: estimated topography induced frequency shift df<sub>topo</sub>
- **Analog Output 2**: estimated surface potential U<sub>dc</sub> = U<sub>cpd</sub>
- **Analog Output 3**: coefficient a = f<sub>0</sub>/4k d<sup>2</sup>C/dz<sup>2</sup>
- **Analog Output 4**: estimation error squared df<sub>error</sub><sup>2</sup>, with df<sub>error</sub> = df - (df<sub>topo</sub> - a (U<sub>ts</sub> - U<sub>cpd</sub>)<sup>2</sup>)

See the RedPitaya manual for the analog pin assignment.



Python interface
----------------

- `config/ssh_info.txt` enter the access information to your RedPitaya here
- `interface/gui.pyw` graphical user interface
- `interface/gui_config.py` config file for the GUI
- `interface/cli.py` command line interface



How to use the controller
-----------

Download this repository to a computer with ssh access to the RedPitaya, and open the python script `interface/gui.pyw`. Tested for Python 3.8.5. requires `numpy`, `json`, `time`, `tkinter`, `threading`, and `paramiko`.
Update the access information in `config/ssh_info.txt`. Depending on your network settings, either use the IP address of your device as host, or `rp-XXXXXX.local`.

Press `connect` to establish a ssh connection to your device. If the connection succeeded, the `connect` button on the top-right turns green.

Note, that changing a parameter in the gui will lead to a instantaneous update on the device. By pressing the arrow-keys an incremental 10% increase/decrease of the selected parameter is triggered.

**Start**:
- `run program` Start the main loop on the RedPitaya, and write all parameters to the device; the write process may take a few seconds.
- `reset estimations` This resets the estimation of state and covariance to the initial values. This function can be used when the controller lost its stability.

**Bias Configuration**:
- `enable bias modulation` Enable the bias modulation by the RedPitaya; 
    If this is turned off, external modulation is required, and RF Output 2 will only contain U<sub>dc</sub>.
    In any case, RF Input 2 will be used as tip-sample bias voltage U<sub>ts</sub> for the estimation.

**State Observer**:
- `enable state observer` Run the Kalman filter.
- `Q` models the state transition noise of each chanel, which can be interpreted as gain.
- `R` models the observation noise, which is split into detection and thermal noise. 
- `measure sampling rate` Perform a measurement of the of the sampling rate on the RedPitaya, and update its value; the sampling rate is used to adjust the gain and noise parameters of the Kalman filter.
- `Sampling rate` loop frequency of the controller (we measured roughly 140 kHz)
- `PLL bw` bandwidth of the detector (e.g. PLL)

**Input/Output gains**:
- All inputs and outputs are scaled with these values. 

**Other**:
- `read all` Read all parameters from the device and copy them to the gui.
- `write all` Write all parameters from the gui to the RedPitaya. The write process may take a few seconds.
- `load` Read the parameters from a json file stored on your computer. The parameters are not yet updated to the device and can be sent by `write all`.
- `save` Save the current settings.