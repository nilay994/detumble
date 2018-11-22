/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2ctmp007.c ========
 */
#include "bmx_gyr.h"
/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define SGN(a)   ((a)<(0)?(-1):(1))


#define TASKSTACKSIZE       640
#define UART_BUFFER_SIZE    100

#define BMX_ACC 0x18
#define BMX_GYR 0x68
#define BMX_MAG 0x10


I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;


UART_Handle uart0;
// /* Driver handle shared between the task and the callback function */
// UART_Handle uart0;

uint8_t         txBuffer[1];
uint8_t         rxBuffer[6];
char uartTxBuffer[50];

int len;

// void uartCallback(UART_Handle handle, void *buf, size_t count) {
//     return;
// }

// void *mainThread(void *arg0)
// {
//     uartTxBuffer = (char *) malloc(sizeof(char)*UART_BUFFER_SIZE);

//     // Call driver init functions
//     GPIO_init();
//     I2C_init();
//     UART_init();

//     UART_Params uartParams;
//     UART_Params_init(&uartParams);
//     uartParams.writeDataMode = UART_DATA_TEXT;
//     uartParams.writeMode     = UART_MODE_CALLBACK; //TODO: I2C blocking mode significance
//     uartParams.writeCallback = uartCallback;
//     uartParams.baudRate      = 115200;
//     uart0 = UART_open(Board_uart0, &uartParams);

//     /* Configure the LED pin */
//     GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//     len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, " \n \n------Starting BMX test----- \n \n");
//     UART_write(uart0, uartTxBuffer, len);

//     /* Turn on user LED */
//     GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

//     bmxMag_init();
//     bmxMag_read_trim();
//     bmxMag_get_bias();
//     bmxMag_read_data();
//     // done with crude settings, time to read the raw values
//     //bmxMag_get_xyz();

//     /* Deinitialized I2C */
//     I2C_close(i2c);
//     UART_write(uart0, "I2C closed!\n", 10);
//     free(uartTxBuffer);
//     return (NULL);
// }

/*
 * TODO: data rate acq: 0x13 HBW, filtered data is selected by default
 * TODO: Interrupts: ANYMOTION (high ang acc) and HIGHRATE (high tumbling)
 * reroute to INT3 and INT4 pins if you think interupts are necessary
 * */
void bmxGyr_init()
{

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
    len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "------SPIN em!-----\n");
    UART_write(uart0, uartTxBuffer, len);

    /* Create I2C for usage */
    i2cParams.bitRate = I2C_400kHz;
    I2C_Params_init(&i2cParams); // try blocking mode

    i2c = I2C_open(MSP_EXP432P401R_I2CB1, &i2cParams);
    usleep(200000);

    /* check if i2c handle is assigned */
    bmxGyr_check_i2c();
    usleep(200000);

    /*check the GYRO by self test */
    bmxGyr_bist_test();
    usleep(200000);

    /* currently not functional */
    //bmxGyr_normal_mode();
    //usleep(200000);

    /* read chip ID, should return 0x32 if everything is correct */
    bmxGyr_read_id();
    usleep(200000);

    /* (BMM050_NORMAL) data rate is 10 Hz, aka refresh rate */
    bmxGyr_set_datarange();
    usleep(200000);

    /* Robert asks for 10 repetitions before flushing */
    bmxGyr_set_odrfilter();
    usleep(200000);
}

void bmxGyr_check_i2c()
{
    if (i2c == NULL) {
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "Error Initializing I2C \n");
        UART_write(uart0, uartTxBuffer, len);
    }
    else {
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "I2C Initialized! \n");
        UART_write(uart0, uartTxBuffer, len);
    }
}

void bmxGyr_bist_test()
{
    /* Point to the gyro's test registers */
    txBuffer[0] = 0x3C;
    txBuffer[1] = 0x01;   // trig_bist

    i2cTransaction.slaveAddress = BMX_GYR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        // wait for the test,
        usleep(200000);
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "GYR test success check result bits!\n");
        UART_write(uart0, uartTxBuffer, len);
    }


    /* Point to the gyro's test registers */
    txBuffer[0] = 0x3C;

    i2cTransaction.slaveAddress = BMX_GYR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        /*
        int bist_rdy  = 0x02 & rxBuffer[0];
        int bist_fail = 0x04 & rxBuffer[0];
        int rate_okay = 0x10 & rxBuffer[0];
        */
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "(0x12) GYR test result: 0x%02x\n", rxBuffer[0]);
        UART_write(uart0, uartTxBuffer, len);
    }
}
/*
 * DONT use this, default value is normal mode, and bits aren't masked yet
 * */
void bmxMag_normal_mode()
{
    /* Point to the gyro's mode registers */
    txBuffer[0] = 0x11;
    txBuffer[1] = 0x00;

    i2cTransaction.slaveAddress = BMX_GYR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, 30, "sleep mode active! \n");
        UART_write(uart0, uartTxBuffer, len);
    }
}

/*
 * redundant: alive checks
 * */
void bmxGyr_read_id()
{
    bool match = false;
    txBuffer[0] = 0x00;
    i2cTransaction.slaveAddress = BMX_GYR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;

    if(I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, 30, "chipID (0x0F): 0x%02x \n", rxBuffer[0]);
        UART_write(uart0, uartTxBuffer, len);
    }
    if (rxBuffer[0] == 0x0F) {
        match = true;
    }
}

/*
 * redundant since default is plus minus 2000 deg/s
 */
void bmxGyr_set_datarange()
{
    txBuffer[0] = 0x0F;
    txBuffer[1] = 0x00;   // pm 2000 deg/s

    //txBuffer[1] = 0x01; // pm 1000 deg/s
    //txBuffer[1] = 0x02; // pm 500 deg/s
    //txBuffer[1] = 0x03; // pm 250 deg/s
    //txBuffer[1] = 0x04; // pm 125 deg/s

    i2cTransaction.slaveAddress = BMX_GYR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "GYR data rate: 2000 deg/s\n");
        UART_write(uart0, uartTxBuffer, len);
    }
}

/*
 * set the refresh rate and ODR filter bandwidth
 * */
void bmxGyr_set_odrfilter()
{
    /* set the rate data filter bandwidth to lowest ODR and BW */
    txBuffer[0] = 0x10;
    txBuffer[1] = 0x87;  // decimation of 20, ODR: 100 Hz, Filter BW: 32 Hz

    i2cTransaction.slaveAddress = BMX_GYR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "filter cutoff at 32 Hz, ODR of 100 Hz\n");
        UART_write(uart0, uartTxBuffer, len);
    }
}

/*
 * get [internally] filtered GYR values
 */
void bmxGyr_get_xyz()
{
    int16_t yaw_rate, pitch_rate, roll_rate;
    /* read the axes */
    txBuffer[0] = 0x02;
    i2cTransaction.slaveAddress = BMX_GYR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 6;
    if (I2C_transfer(i2c, &i2cTransaction)) {

        yaw_rate   = rxBuffer[1] << 8 | (rxBuffer[0]);  // lim: pm 4095
        pitch_rate = rxBuffer[3] << 8 | (rxBuffer[2]);  // lim: pm 4095
        roll_rate  = rxBuffer[5] << 8 | (rxBuffer[4]);  // lim: pm 16383

        len = snprintf(uartTxBuffer, 50, "[GYR rate] yaw: %d pitch: %d roll: %d \n", yaw_rate, pitch_rate, roll_rate);
        UART_write(uart0, uartTxBuffer, len);
    }
}

/* test if merge word works without typecasting */
/*
int16_t var;
uint8_t var1 = 0xAA;
uint8_t var2 = 0x55;
var = var1 << 8 | var2;
len = snprintf(uartTxBuffer, 30, "var: 0x%04x, %x, %d \n", var, var, var);
UART_write(uart0, uartTxBuffer, len);
*/

/*
 * temperature as per as: https://github.com/kriswiner/BMX-055/blob/master/BMX055_MS5637_BasicAHRS_t3.ino
 * also study the Madwick filters used to get orientation
 *
int16_t readACCTempData()

{

  uint8_t c =  readByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_TEMP);  // Read the raw data register

  return ((int16_t)((int16_t)c << 8)) >> 8 ;  // Turn the byte into a signed 8-bit integer
 temperature = ((float) tempCount) / 2.0 + 23.0; // Gyro chip temperature in degrees Centigrade
}
 * **/

