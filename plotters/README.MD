## files description
- `compare_plots.m` - compare plots between original matlab implementation, c implementation, and plots given by MSP432. Needs input csv files to run.
- `serial_plotter_bmx.m` - generates the plots for the magentic fields as shown in the root of this repo.
- `serial_plotter_bmx.py` - does the same as above, but in Python. (not intended to be updated anymore).
- `hil_pc.py` - Hardware in Loop test emulator, requires input CSV and the MSP432 in HIL test mode to work.

## plots not exactly matching yet (all others are passing for v0.4 with epsilon of 10^{-7})
- sign plots (last graph) are different. Wouldn't lead to unfavorable consequences, and its harmless. Verify using `readMatrix_r(find(b1), 7)` after running the the Matlab script - `compare_plots.m` 

![Tumbling parameter plots](https://github.com/nilay994/detumble/blob/master/plotters/cmp1.jpg)
![Magnetorquers sign plots](https://github.com/nilay994/detumble/blob/master/plotters/cmp2.jpg)
![Magnetorquers time plots](https://github.com/nilay994/detumble/blob/master/plotters/cmp3.jpg)
![Tumble count plots](https://github.com/nilay994/detumble/blob/master/plotters/cmp4.jpg)
![diff plots](https://github.com/nilay994/detumble/blob/master/plotters/cmp5.jpg)
