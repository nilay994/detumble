# Implementing detumble algorithm v0.4 on C
 - intended to be compiled on GCC 6.4.0 
 - uses Visual Studio 2017 for debugging. Will switch to gcc and gdb soon.
 - todo's mentioned in source
 
## log 
- generated partially garbage outputs
- compiled with gcc, no gdb on msys, switch to visual studio for debugging
- abs changed to fabs, tumble parameter updates almost look perfect
- suspecting my macros are making sign errors, s_on and t_on don't match with robert
- plots attached below, matlab script committed.
- fixed t_on by some static variables, plots exactly match for data4test_02_in
- currently noisy `p_tumb` and incorrect tumbling counts for data4test_03_in
- changed scripts to also compare plots for MSP432 HIL tests
