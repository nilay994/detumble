#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "detumble_algo.h"

/* Example/Board Header files */
#include "Board.h"

/* Driver Header files */
// ADD predefines in project settings - MSP432P401R family
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>  // incl. for finding the FPU
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Driver handle shared between the task and the callback function */
UART_Handle uart0;

// TODO: check significance 
#define TASKSTACKSIZE       1000//640
#define UART_BUFFER_SIZE    200

void *mainThread(void *arg0)
{
    FPU_enableModule();

    /* give transaction strings some space */
    char *readBuf;
    char *writeBuf;
    writeBuf = (char *) malloc(sizeof(char)*UART_BUFFER_SIZE);
    readBuf = (char *) malloc(sizeof(char)*UART_BUFFER_SIZE);
    memset(writeBuf, 0, UART_BUFFER_SIZE);
    memset(readBuf, 0, UART_BUFFER_SIZE);
    int len = 0;

    /* Call driver init functions */
    GPIO_init();
    I2C_init();
    UART_init();

    /* Configure the LED pin - for output */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.writeDataMode  = UART_DATA_TEXT;
    uartParams.readDataMode   = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_NEWLINE;
    uartParams.writeMode      = UART_MODE_BLOCKING;
    uartParams.readMode       = UART_MODE_BLOCKING;
    uartParams.readEcho       = UART_ECHO_OFF;
    uartParams.baudRate       = 115200;
    uart0 = UART_open(Board_UART0, &uartParams);
    if (uart0 == NULL) {
        /* UART_open() failed */
        while (1);
    }
    // send sync string
    len = snprintf(writeBuf, UART_BUFFER_SIZE, "------SPIN em!-----\n");
    UART_write(uart0, writeBuf, len);

    usleep(100000); // warmup

    /* Turn on user LED, indicate start of process */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    // initialize detumble variables
    vector_t mag1Data = {0,0,0};
    vector_t mag2Data = {0,0,0};
    vector_t s_on     = {0,0,0};
    vector_t t_on     = {0,0,0};
    vector_t p_tumb   = {0,0,0};
    // init parsing variables
    char *token = 0;
    int column = 0;
    float parse_row[6];

    /* unidenitified registers from earlier code, study effect under large execution time on FPU*/
    // WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;      // stop watchdog timer
    int cnt = 0;
    
    while(1) {

        len = UART_read(uart0, readBuf, UART_BUFFER_SIZE);

        // if read is successful, parse the csv row
        if (len > 0) {
            // initialize buffer for each line and split the string by ","
            token = strtok(readBuf, ",");
            // populate the readings
            while(token) {
                parse_row[column] = atof(token);
                token = strtok(NULL, ",");  // reset the static pointer
                column ++;
            }

            // array is filled now, time for detumbling
            mag1Data.x = parse_row[0];
            mag1Data.y = parse_row[1];
            mag1Data.z = parse_row[2];
            mag2Data.x = parse_row[3];
            mag2Data.y = parse_row[4];
            mag2Data.z = parse_row[5];

            // make sure to leave counts below static, they aren't remembered by controlLoop, 
            // they are just incremented by controlLoop
            static unsigned int c_tumb = 0;
            static unsigned int c_detumb = 0;
            //GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
            controlLoop(mag1Data, mag2Data, &s_on, &t_on, &p_tumb, &c_tumb, &c_detumb);
            len = snprintf(writeBuf, UART_BUFFER_SIZE, 
                "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d\n",
                mag1Data.x, mag1Data.y, mag1Data.z,
                mag2Data.x, mag2Data.y, mag2Data.z,
                t_on.x, t_on.y, t_on.z,
                s_on.x, s_on.y, s_on.z,
                p_tumb.x, p_tumb.y, p_tumb.z,
                c_tumb, c_detumb);

            cnt ++;

            if (cnt % 50 == 0) {
                GPIO_toggle(Board_GPIO_LED0);
            }

            // flush result to python 
            UART_write(uart0, writeBuf, len);

            //GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
            // preliminaries before next csv read
            memset(writeBuf, 0, 200);
            memset(readBuf, 0, 200);
            column = 0;
        }
    }

    // break while loop here in case of exceptions
    free(readBuf);
    free(writeBuf);
    return(NULL);
}
