# Implementing detumble algorithm v0.4 on C
 - intended to be compiled on GCC 6.4.0 
 - uses Visual Studio 2017 for debugging. Will switch to gcc and gdb soon.
 - todo's mentioned in source

## plots not exactly matching yet
- ~~t_on~~
- s_on (shouldn't matter if t_on is almost zero - but verify by subtracting)
- ~~tumbling parameter~~
- MSP432 implementation feasibility in progess
- IEEE-754 FPU under study for binary16 floats


## log 
- generated partially garbage outputs
- compiled with gcc, no gdb on msys, switch to visual studio for debugging
- abs changed to fabs, tumble parameter updates almost look perfect
- suspecting my macros are making sign errors, s_on and t_on don't match with robert
- plots attached below, matlab script committed.
- fixed t_on by some static variables, real reason for failure under study

![Tumbling parameter plots](https://github.com/nilay994/detumble/blob/master/detumble_bdot/cmp1.jpg)
![Magnetorquers sign plots](https://github.com/nilay994/detumble/blob/master/detumble_bdot/cmp2.jpg)
![Magnetorquers time plots](https://github.com/nilay994/detumble/blob/master/detumble_bdot/cmp3.jpg)
![Tumble count plots]
(https://github.com/nilay994/detumble/blob/master/detumble_bdot/cmp4.jpg)