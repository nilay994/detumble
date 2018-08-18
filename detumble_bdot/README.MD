# Implementing detumble algorithm v0.2 on C
 - intended to be compiled on GCC 6.4.0 
 - uses Visual Studio 2017 for debugging. Will switch to gcc and gdb soon.
 - todo's mentioned in source

## not working at the moment
- t_on
- s_on
- MSP432 implementation feasibility not checked yet


## log 
- generated partially garbage outputs
- compiled with gcc, no gdb on msys, switch to visual studio for debugging
- abs changed to fabs, tumble parameter updates almost look perfect
- suspecting my macros are making sign errors, s_on and t_on don't match with robert
- plots attached below, matlab script committed.

![Tumbling parameter plots](https://github.com/nilay994/detumble/blob/master/detumble_robert_c/cmp1.jpg)
![Magnetorquers sign plots](https://github.com/nilay994/detumble/blob/master/detumble_robert_c/cmp2.jpg)
![Magnetorquers time plots](https://github.com/nilay994/detumble/blob/master/detumble_robert_c/cmp3.jpg)