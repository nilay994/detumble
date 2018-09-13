---
# read MAG of BMX055 for PocketCube 

---

## to do list: 
1. implement interrupts, failure detection
2. implement overflows, software reset feature
3. calibrate to uT
4. test against loose wires, temperatures
5. implement library

## Example Summary


## Peripherals Exercised



## Resources & Jumper Settings


## Example Usage

The connection will have the following settings:
```
    Baud-rate:     115200
    Data bits:          8
    Stop bits:          1
    Parity:          None
    Flow Control:    None
```

* Run the example. `Board_GPIO_LED0` turns ON to indicate driver
initialization is complete.

## Application Design Details *(todo)

This application uses one task:

`'getTempTask'` - performs the following actions:

1. Opens and initializes an I2C driver object.

2. Uses the I2C driver to get data from the TMP007 sensor.

3. Extracts the temperature (in Celsius) and prints the value via the UART.

4. The task sleeps for 1000 system ticks.

5. After 20 temperature samples are recovered, the I2C peripheral is closed
and the example exits.

TI-RTOS:

* When building in Code Composer Studio, the kernel configuration project will
be imported along with the example. The kernel configuration project is
referenced by the example, so it will be built first. The "release" kernel
configuration is the default project used. It has many debug features disabled.
These feature include assert checking, logging and runtime stack checks. For a
detailed difference between the "release" and "debug" kernel configurations and
how to switch between them, please refer to the SimpleLink MCU SDK User's
Guide. The "release" and "debug" kernel configuration projects can be found
under &lt;SDK_INSTALL_DIR&gt;/kernel/tirtos/builds/&lt;BOARD&gt;/(release|debug)/(ccs|gcc).

FreeRTOS:

* Please view the `FreeRTOSConfig.h` header file for example configuration
information.


