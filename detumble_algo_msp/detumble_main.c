#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "detumble_algo.h"

/* Example/Board Header files */
#include "Board.h"

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Driver handle shared between the task and the callback function */
UART_Handle uart0;
uint8_t  txBuffer[1];
uint8_t  rxBuffer[6];
char     *uartTxBuffer = NULL;
int len;

#define TASKSTACKSIZE       1000//640
#define UART_BUFFER_SIZE    200
/*
 * Callback function to use the UART in callback mode. It does nothing
 */
void uartCallback(UART_Handle handle, void *buf, size_t count) {
    return;
}
/**
 * main.c
 */
void *mainThread(void *arg0)
{
    uartTxBuffer = (char *) malloc(sizeof(char)*UART_BUFFER_SIZE);

    /* Call driver init functions */
    GPIO_init();
    I2C_init();
    UART_init();
    
    UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.writeMode     = UART_MODE_CALLBACK; //TODO: I2C blocking mode significance
    uartParams.writeCallback = uartCallback;
    uartParams.readEcho      = UART_ECHO_OFF;
    uartParams.baudRate      = 115200;
    uart0 = UART_open(Board_UART0, &uartParams);


    /* Create a UART with data processing off. */
    /*
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    */

    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, " \n \n------Starting BMX test----- \n \n");
    UART_write(uart0, uartTxBuffer, len);

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
    usleep(100000);
#if 1
    vector_t mag1Data = {32.7, -7.8, -29.7};
    vector_t mag2Data = {32.7, -8.7, -30.3};

    vector_t s_on;
    vector_t t_on;
    vector_t p_tumb;

    static unsigned int c_tumb = 0;
    static unsigned int c_detumb = 0;
    // WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;      // stop watchdog timer
    controlLoop(mag1Data, mag2Data, &s_on, &t_on, &p_tumb, &c_tumb, &c_detumb);
    len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d\n",
        mag1Data.x, mag1Data.y, mag1Data.z,
        mag2Data.x, mag2Data.y, mag2Data.z,
        t_on.x, t_on.y, t_on.z,
        s_on.x, s_on.y, s_on.z,
        p_tumb.x, p_tumb.y, p_tumb.z,
        c_tumb, c_detumb);
#endif
    UART_write(uart0, uartTxBuffer, len);
    free(uartTxBuffer);
    return (NULL);
}

