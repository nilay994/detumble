# HIL test for detumbling

## log
1) initial commit - python script: parses excel (csv), flushes to serial, parses serial, flushes back to excel (csv)

2) faced trouble with UART and a lot of new lines, need to sure about clean scratchpad after malloc on input buffer

3) fixed csv alternate lines, suspicious carriage return while `file.write()` on Windows10 and Python3


## next steps 

- trying to parse csv via serial on MSP432, and flush output back to serial

- plot matlab v/s visual studio v/s gcc v/s msp-gcc 




