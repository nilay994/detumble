#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Example/Board Header files */
#include "Board.h"

/* Driver Header files */
// ADD predefines in project settings - MSP432P401R family
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>  // incl. for finding the FPU
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

// TODO: check significance 
#define TASKSTACKSIZE       1000//640
#define UART_BUFFER_SIZE    200

void *mainThread(void *arg0)
{
    FPU_enableModule();

    int len = 0;

    /* Call driver init functions */
    GPIO_init();
    I2C_init();
    UART_init();

    /* Configure the LED pin - for output */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    usleep(100000); // warmup

    /* Turn on user LED, indicate start of process */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
    
    bmxGyr_init();

    while(1) {
        bmxGyr_get_xyz();
        usleep(100000);
    }

    return(NULL);
}
