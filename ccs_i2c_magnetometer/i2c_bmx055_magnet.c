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
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"

#define TASKSTACKSIZE       640
#define UART_BUFFER_SIZE    100

#define BMX_ACC 0x18
#define BMX_GYR 0x68
#define BMX_MAG 0x10


I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;

void bmxMag_check_i2c();
void bmxMag_get_repetitions();
void bmxMag_goto_sleep();
void bmxMag_init();
void bmxMag_read_data();
bool bmxMag_read_id();
void bmxMag_read_trim();
void bmxMag_set_datarate();
void bmxMag_set_repetitions();

/* Driver handle shared between the task and the callback function */
UART_Handle uart0;

uint8_t         txBuffer[1];
uint8_t         rxBuffer[6];
char            *uartTxBuffer = NULL;

int len;
/*
 * Callback function to use the UART in callback mode. It does nothing
 */
void uartCallback(UART_Handle handle, void *buf, size_t count) {
    return;
}
/*
 *  ======== mainThread ========
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
    uartParams.baudRate      = 115200;
    uart0 = UART_open(Board_UART0, &uartParams);

    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, " \n \n------Starting BMX test----- \n \n");
    UART_write(uart0, uartTxBuffer, len);

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    bmxMag_init();
    bmxMag_read_trim();
    bmxMag_read_data();
    // done with crude settings, time to read the raw values
    //bmxMag_get_xyz();

    /* Deinitialized I2C */
    I2C_close(i2c);
    UART_write(uart0, "I2C closed!\n", 10);
    free(uartTxBuffer);
    return (NULL);
}

void bmxMag_init()
{
    /* Create I2C for usage */
    i2cParams.bitRate = I2C_400kHz;
    I2C_Params_init(&i2cParams); // try blocking mode

    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    usleep(200000);

    /* check if i2c handle is assigned */
    bmxMag_check_i2c();
    usleep(200000);

    /* bmx can only goto normal mode via sleep mode.
     * currently, sensor was just powered on and is in suspend mode */
    bmxMag_goto_sleep();
    usleep(200000);

    /* read chip ID, should return 0x32 if everything is correct */
    bmxMag_read_id();
    usleep(200000);

    /* (BMM050_NORMAL) data rate is 10 Hz, aka refresh rate */
    bmxMag_set_datarate();
    usleep(200000);

    /* Robert asks for 10 repetitions before flushing */
    bmxMag_set_repetitions();
    usleep(200000);

    /* example to read the number of repetitions */
    /* read the xy repetitions */
    bmxMag_get_repetitions();
    usleep(200000);
}

void bmxMag_check_i2c()
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

bool bmxMag_read_id()
{
    bool match = false;
    txBuffer[0] = 0x40;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;

    if(I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, 30, "chipID: 0x%02x \n", rxBuffer[0]);
        UART_write(uart0, uartTxBuffer, len);
    }
    if (rxBuffer[0] == 0x32) {
        match = true;
    }
    return match;
}

void bmxMag_goto_sleep()
{
    /* Point to the magnetometer's registers */
    txBuffer[0] = 0x4B;
    txBuffer[1] = 0x01;

    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, 30, "sleep mode active! \n");
        UART_write(uart0, uartTxBuffer, len);
    }
}

void bmxMag_set_datarate()
{
    txBuffer[0] = 0x4C;
    txBuffer[1] = 0x00;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "Data rate: 10 Hz, Sensor Mode: Normal \n");
        UART_write(uart0, uartTxBuffer, len);
    }
}

void bmxMag_set_repetitions()
{
    /* Set the XY-repetitions number for Normal mode */
    txBuffer[0] = 0x51;
    txBuffer[1] = 0x04;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "XY repetitions: 9 \n");
        UART_write(uart0, uartTxBuffer, len);
    }
    usleep(200000);

    /* Set the Z-repetitions number for Normal mode */
    txBuffer[0] = 0x52;
    txBuffer[1] = 0x0F;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "Z repetitions: 15 \n");
        UART_write(uart0, uartTxBuffer, len);
    }
}

void bmxMag_get_repetitions()
{
    txBuffer[0] = 0x51;
    //txBuffer[1] = 0x40;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, 30, "(should be 0x04) 0x%02x \n", rxBuffer[0]);
        UART_write(uart0, uartTxBuffer, len);
    }
}

void bmxMag_get_xyz()
{
    int16_t bh, bv, bz;
    /* read the axes */
    txBuffer[0] = 0x42;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 6;
    while(1) {
        if (I2C_transfer(i2c, &i2cTransaction)) {

            bh = rxBuffer[1] << 8 | (rxBuffer[0] & 0xF8);  // lim: pm 4095
            bv = rxBuffer[3] << 8 | (rxBuffer[2] & 0xF8);  // lim: pm 4095
            bz = rxBuffer[5] << 8 | (rxBuffer[4] & 0xFE);  // lim: pm 16383

            len = snprintf(uartTxBuffer, 30, "bh: %d bv: %d bz: %d \n", bh, bv, bz);
            UART_write(uart0, uartTxBuffer, len);
        }
        usleep(100002);     // 10 Hz = 100 ms
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

/*!
 * @brief Structure containing mag initial parameters
 */
struct bmm050_t {
    int8_t   dig_x1;/**< trim x1 data */
    int8_t   dig_y1;/**< trim y1 data */

    int8_t   dig_x2;/**< trim x2 data */
    int8_t   dig_y2;/**< trim y2 data */

    uint16_t dig_z1;/**< trim z1 data */
    int16_t  dig_z2;/**< trim z2 data */
    int16_t  dig_z3;/**< trim z3 data */
    int16_t  dig_z4;/**< trim z4 data */

    uint8_t  dig_xy1;/**< trim xy1 data */
    int8_t   dig_xy2;/**< trim xy2 data */

    uint16_t dig_xyz1;/**< trim xyz1 data */
};



/* Trim Extended Registers */
#define BMM050_DIG_X1                      (0x5D)
#define BMM050_DIG_Y1                      (0x5E)
#define BMM050_DIG_Z4_LSB                  (0x62)
#define BMM050_DIG_Z4_MSB                  (0x63)
#define BMM050_DIG_X2                      (0x64)
#define BMM050_DIG_Y2                      (0x65)
#define BMM050_DIG_Z2_LSB                  (0x68)
#define BMM050_DIG_Z2_MSB                  (0x69)
#define BMM050_DIG_Z1_LSB                  (0x6A)
#define BMM050_DIG_Z1_MSB                  (0x6B)
#define BMM050_DIG_XYZ1_LSB                (0x6C)
#define BMM050_DIG_XYZ1_MSB                (0x6D)
#define BMM050_DIG_Z3_LSB                  (0x6E)
#define BMM050_DIG_Z3_MSB                  (0x6F)
#define BMM050_DIG_XY2                     (0x70)
#define BMM050_DIG_XY1                     (0x71)

struct bmm050_t bmm;
/* time for challenging factory trim */
void bmxMag_read_trim()
{
    usleep(100000);

    txBuffer[0] = BMM050_DIG_X1;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, 50, "DIG_X1: 0x%02x, DIG_Y1: 0x%02x \n", rxBuffer[0], rxBuffer[1]);
        UART_write(uart0, uartTxBuffer, len);
    }
    bmm.dig_x1 = rxBuffer[0];
    bmm.dig_y1 = rxBuffer[1];

    usleep(100000);

    txBuffer[0] = BMM050_DIG_X2;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, 50, "DIG_X2: 0x%02x, DIG_Y2: 0x%02x \n", rxBuffer[0], rxBuffer[1]);
        UART_write(uart0, uartTxBuffer, len);
    }
    bmm.dig_x2 = rxBuffer[0];
    bmm.dig_y2 = rxBuffer[1];

    usleep(100000);

    /* be careful big endian */
    txBuffer[0] = BMM050_DIG_XY2;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        len = snprintf(uartTxBuffer, 50, "DIG_XY2: 0x%02x, DIG_XY1: 0x%02x \n", rxBuffer[0], rxBuffer[1]);
        UART_write(uart0, uartTxBuffer, len);
    }
    bmm.dig_xy2 = rxBuffer[0];
    bmm.dig_xy1 = rxBuffer[1];

    usleep(100000);

    txBuffer[0] = BMM050_DIG_Z1_LSB;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        bmm.dig_z1 = (uint16_t) (((uint16_t)(rxBuffer[1] << 8))|rxBuffer[0]); // TODO: maybe use this to read from Mag as well
        len = snprintf(uartTxBuffer, 50, "DIG_Z1_LSB: 0x%02x, DIG_Z1_MSB: 0x%02x, Cmb: 0x%04x \n", rxBuffer[0], rxBuffer[1], bmm.dig_z1);
        UART_write(uart0, uartTxBuffer, len);
    }

    usleep(100000);

    txBuffer[0] = BMM050_DIG_Z2_LSB;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        bmm.dig_z2 = (uint16_t) (((uint16_t)(rxBuffer[1] << 8))|rxBuffer[0]); // TODO: maybe use this to read from Mag as well
        len = snprintf(uartTxBuffer, 50, "DIG_Z2_LSB: 0x%02x, DIG_Z2_MSB: 0x%02x, Cmb: 0x%04x \n", rxBuffer[0], rxBuffer[1], bmm.dig_z2);
        UART_write(uart0, uartTxBuffer, len);
    }

    usleep(100000);

    txBuffer[0] = BMM050_DIG_XYZ1_LSB;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        bmm.dig_xyz1 = (uint16_t) (((uint16_t)(rxBuffer[1] << 8))|rxBuffer[0]); // TODO: maybe use this to read from Mag as well
        len = snprintf(uartTxBuffer, 50, "DIG_XYZ1_LSB: 0x%02x, DIG_XYZ2_MSB: 0x%02x, Cmb: 0x%04x \n", rxBuffer[0], rxBuffer[1], bmm.dig_xyz1);
        UART_write(uart0, uartTxBuffer, len);
    }
    usleep(100000);

    txBuffer[0] = BMM050_DIG_Z3_LSB;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
       bmm.dig_z3 = (uint16_t) (((uint16_t)(rxBuffer[1] << 8))|rxBuffer[0]); // TODO: maybe use this to read from Mag as well
       len = snprintf(uartTxBuffer, 50, "DIG_Z3_LSB: 0x%02x, DIG_Z3_MSB: 0x%02x, Cmb: 0x%04x \n", rxBuffer[0], rxBuffer[1], bmm.dig_z3);
       UART_write(uart0, uartTxBuffer, len);
    }

    usleep(100000);

    txBuffer[0] = BMM050_DIG_Z4_LSB;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        bmm.dig_z4 = (uint16_t) (((uint16_t)(rxBuffer[1] << 8))|rxBuffer[0]); // TODO: maybe use this to read from Mag as well
        len = snprintf(uartTxBuffer, 50, "DIG_Z4_LSB: 0x%02x, DIG_Z4_MSB: 0x%02x, Cmb: 0x%04x \n", rxBuffer[0], rxBuffer[1], bmm.dig_z4);
        UART_write(uart0, uartTxBuffer, len);
    }
    usleep(100000);
}


void bmxMag_read_data()//(int16_t * magData)
{
    int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
    uint16_t data_r = 0;
    uint8_t rxBuffer[8];  // x/y/z hall magnetic field data, and Hall resistance data
    int16_t magData[3];
    txBuffer[0] = 0x42;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 8;
    while(1) {
        if (I2C_transfer(i2c, &i2cTransaction)) {
            if(rxBuffer[6] & 0x01) { // Check if data ready status bit is set
                /*
                mdata_x = (int16_t) (((int16_t)rxBuffer[1] << 8)  | rxBuffer[0]) >> 3;  // 13-bit signed integer for x-axis field
                mdata_y = (int16_t) (((int16_t)rxBuffer[3] << 8)  | rxBuffer[2]) >> 3;  // 13-bit signed integer for y-axis field
                mdata_z = (int16_t) (((int16_t)rxBuffer[5] << 8)  | rxBuffer[4]) >> 1;  // 15-bit signed integer for z-axis field
                data_r  = (uint16_t) (((uint16_t)rxBuffer[7] << 8)| rxBuffer[6]) >> 2;  // 14-bit unsigned integer for Hall resistance
                */
                // 5 5 7 6
                mdata_x = (int16_t) ((((int32_t)((int8_t) rxBuffer[1])) << 5)  | ((rxBuffer[0] & 0xF8) >> 3));
                mdata_y = (int16_t) ((((int32_t)((int8_t) rxBuffer[3])) << 5)  | ((rxBuffer[2] & 0xF8) >> 3));
                mdata_z = (int16_t) ((((int32_t)((int8_t) rxBuffer[5])) << 7)  | ((rxBuffer[4] & 0xFE) >> 1));
                data_r  = (uint16_t)((((uint32_t)          rxBuffer[7]) << 6)  | ((rxBuffer[6] & 0xFC) >> 2));
                // to verify with Bosch email
                // mdata_x = -88; mdata_y = -17; mdata_z = 113; data_r = 6290;

                // calculate temperature compensated 16-bit magnetic fields
                temp = ((int16_t)(((uint16_t)((((int32_t)bmm.dig_xyz1) << 14)/(data_r != 0 ? data_r : bmm.dig_xyz1))) - ((uint16_t)0x4000)));

                magData[0] = ((int16_t)((((int32_t)mdata_x) *
                ((((((((int32_t)bmm.dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
                (((int32_t)temp) * ((int32_t)(((int16_t)bmm.dig_xy1) << 7)))) >> 9) +
                ((int32_t)0x100000)) * ((int32_t)(((int16_t)bmm.dig_x2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
                (((int16_t)bmm.dig_x1) << 3);

                /* comment/uncomment */
                //temp = ((int16_t)(((uint16_t)((((int32_t)bmm.dig_xyz1) << 14)/(data_r != 0 ? data_r : bmm.dig_xyz1))) - ((uint16_t)0x4000)));

                magData[1] = ((int16_t)((((int32_t)mdata_y) *
                ((((((((int32_t)bmm.dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
                (((int32_t)temp) * ((int32_t)(((int16_t)bmm.dig_xy1) << 7)))) >> 9) +
                ((int32_t)0x100000)) * ((int32_t)(((int16_t)bmm.dig_y2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
                (((int16_t)bmm.dig_y1) << 3);

                magData[2] = (((((int32_t)(mdata_z - bmm.dig_z4)) << 15) - ((((int32_t)bmm.dig_z3) * ((int32_t)(((int16_t)data_r) -
                ((int16_t)bmm.dig_xyz1))))>>2))/(bmm.dig_z2 + ((int16_t)(((((int32_t)bmm.dig_z1) * ((((int16_t)data_r) << 1)))+(1<<15))>>16))));

                len = snprintf(uartTxBuffer, 50, "bh: %d bv: %d bz: %d\n", magData[0], magData[1], magData[2]);
                UART_write(uart0, uartTxBuffer, len);

                usleep(100000);
            }
        }
    }
}



