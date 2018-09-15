## files description
--

## plots not exactly matching yet
- ~~t_on~~
- s_on (shouldn't matter if t_on is almost zero - but verify by subtracting)
- ~~tumbling parameter~~
- MSP432 implementation feasibility in progress
- IEEE-754 FPU under study for binary16 floats


## log 
- generated partially garbage outputs
- compiled with gcc, no gdb on msys, switch to visual studio for debugging
- abs changed to fabs, tumble parameter updates almost look perfect
- suspecting my macros are making sign errors, s_on and t_on don't match with robert
- plots attached below, matlab script committed.
- fixed t_on by some static variables, plots exactly match for data4test_02_in
- currently noisy `p_tumb` and incorrect tumbling counts for data4test_03_in
- changed scripts to also compare plots for MSP432 HIL tests

![Tumbling parameter plots](https://github.com/nilay994/detumble/blob/master/serial_plotter/cmp1.jpg)
![Magnetorquers sign plots](https://github.com/nilay994/detumble/blob/master/serial_plotter/cmp2.jpg)
![Magnetorquers time plots](https://github.com/nilay994/detumble/blob/master/serial_plotter/cmp3.jpg)
![Tumble count plots](https://github.com/nilay994/detumble/blob/master/serial_plotter/cmp4.jpg)