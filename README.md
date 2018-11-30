# detumble

Implementation of detumbling for the Delfi-PQ.
Repository aims to perform some sensor fusion using BMX-055. It finally aims to implement the `B_dot` detumbling algorithm to stabilize the satellite. 
Some of the code might be inherited from `https://github.com/DelfiSpace` 

Branches to be diverged in future: 

1. Accelerometer: tbd - filters cutoff
2. Gyroscope: tbd - 
3. Magnetometer: IN PROGRESS: scroll below for status
4. Detumbling: IN PROGRESS: navigate [here](https://github.com/nilay994/detumble/tree/master/detumble_algo_pc)
5. Sensor fusion: tbd - understand quaternions, explore Kalman filter, sun sensor + magnetometer
6. Magnetorquer: tbd - perform self test closed loop (maybe even the thruster with acc)

## current progress:
### Interfaced BMX-055's magnetometer successfully with the MSP432-

- initialized in a good enough (alive) mode - 10 repetitions per axis
- factory trim read from flash and used in calibration
- temperature compensation for magnetometer fused with the readings (Outer Space doesn't comply, if Bosch provides temperature scaling, might as well use it)
- random and long expressions from Bosch's sensortec library implemented for magnetometer
- bias compensation done with Android's "draw a figure 8 method". i.e. bias = (min + max)/2
- one of the results illustrated below
 
Earth's magnetic field with BMX055, Soft and Hard Iron errors visible in plots below (ref: https://www.vectornav.com/support/library/magnetometer)

![iron_error](https://github.com/nilay994/detumble/blob/master/iron_calib.PNG)

### Muddy parts (not so sure - will reveal vulnerabilities in test)

- signed bit flip behaviour not checked strongly
- not still sure if resolutions and scaling is corresponding to actual uT values
- reset/i2c complications not studied yet
- soft reset yet to be implemented, in case ground station wants to give one more chance to ADCS
- API not coherently check for with MIPS and FLOPS
- possibilities of power saving modes not explored yet
- a strong + lite library far from implemented

## repository file structure:
1. [detumble_algo_msp](https://github.com/nilay994/detumble/tree/master/detumble_algo_msp): Source files for Hardware In Loop (HIL) tests of detumbling algo on MSP432.
2. [detumble_algo_pc](https://github.com/nilay994/detumble/tree/master/detumble_algo_pc): Source files for detumbling algo meant to be compiled on VS/GCC on PC.
3. [magnetometer_test](https://github.com/nilay994/detumble/tree/master/magnetometer_test): Source files for talking to the BMX using MSP432, (with trimming and temperature compensation)
4. [plotters](https://github.com/nilay994/detumble/tree/master/plotters): Python and MATLAB scripts for performing HIL tests, magnetometer calibrations, plotting comparisons between MATLAB, PC and MSP.
5. [uartecho_FPU](https://github.com/nilay994/detumble/tree/master/uartecho_FPU): Parsing floats and interpreting newlines might be different for Windows and Linux. Source files in this folder facilitate early stage debugging before moving to HIL tests. 

