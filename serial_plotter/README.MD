## files description
- `compare_plots.m` - compare plots between original matlab implementation, c implementation, and plots given by MSP432. Needs input csv files to run.
- `serial_plotter_bmx.m` - generates the plots for the magentic fields as shown in the root of this repo.
- `serial_plotter_bmx.py` - does the same as above, but in Python. (not intended to be updated anymore).
- `hil_pc.py` - Hardware in Loop test emulator, requires input CSV and the MSP432 in HIL test mode to work.

## plots not exactly matching yet (all are failing, noise on v0.4, passing for v0.3)
- t_on
- s_on
- tumbling parameter

![Tumbling parameter plots](https://github.com/nilay994/detumble/blob/master/serial_plotter/cmp1.jpg)
![Magnetorquers sign plots](https://github.com/nilay994/detumble/blob/master/serial_plotter/cmp2.jpg)
![Magnetorquers time plots](https://github.com/nilay994/detumble/blob/master/serial_plotter/cmp3.jpg)
![Tumble count plots](https://github.com/nilay994/detumble/blob/master/serial_plotter/cmp4.jpg)