# detumble

Implementation of detumbling for the Delfi-PQ.
Repository aims to perform some sensor fusion using BMX-055. It finally aims to implement the `B_dot` detumbling algorithm to stabilize the satellite. 
Some of the code might be inherited from `https://github.com/DelfiSpace` 

Branches: 

1. Accelerometer: tbd - filters cutoff
2. Gyroscope: tbd - 
3. Magnetometer: tbd - temperature offset resistance, factory trims
4. Detumbling: tbd - test matlab script, make plots, convert to c and make plots, finally burn it
5. Sensor fusion: tbd - understand quaternions, explore Kalman filter, sun sensor + magnetometer
6. Magnetorquer: tbd - perform self test (maybe even the thruster with acc)

![Earth's magnetic field with BMX055](https://github.com/nilay994/detumble/blob/master/earth_field_bias.png)