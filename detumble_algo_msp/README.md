# HIL test for detumbling

## log
1) UART reconfigured, skeleton files for HIL test pushed (uartecho) 
2) initial commit - python script: parses excel (csv), flushes to serial, parses serial, flushes back to excel (csv)
3) faced trouble with UART and a lot of new lines, need to sure about clean scratchpad after malloc on input buffer
4) fixed csv alternate lines, suspicious carriage return while `file.write()` on Windows10 and Python3
5) trying to parse csv via serial on MSP432, and flush output back to serial, FPU warmed up
6) HIL emulator + uartecho + parser + detumble algo tested 
7) plot matlab v/s visual studio v/s gcc v/s msp-gcc 
8) test took more than 3 hours, expected time was 1 1/2 hours, plots pushed [here](https://github.com/nilay994/detumble/tree/master/serial_plotter)
Whenever I write HIL tests, I mean something like this: 

![hil_image](https://github.com/nilay994/detumble/blob/master/hil_tests.jpg)

p = 45*2 = 90 minutes round trip time for the HIL data to flow through serial
q = FPU time 
total time for HIL test = (p+q) > 1 1/2 hours

## next steps 
1) Fix noise on original algorithm that migrated with algo v0.4
2) determine FLOPS and speed of FPU - check feasiblity with control loop of v0.4
3) timing and profiling the current HIL tests
4) consequeces of binary16 instead of floats
5) diverge from the current folder, proceed towards hardware timer of MSP432 - to proceed with actual detumbling






