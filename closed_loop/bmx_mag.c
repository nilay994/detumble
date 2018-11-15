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
#include "bmx_mag.h"
/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "ADCS_Board.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define SGN(a)   ((a)<(0)?(-1):(1))


#define TASKSTACKSIZE       640
#define UART_BUFFER_SIZE    100

#define BMX_ACC 0x19
#define BMX_GYR 0x68
#define BMX_MAG 0x11


I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;


extern UART_Handle uart_dbg_bus;
// /* Driver handle shared between the task and the callback function */
// UART_Handle uart_dbg_bus;

uint8_t         txBuffer[1];
uint8_t         rxBuffer[6];
char uartTxBuffer[UART_BUFFER_SIZE];

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
//     uart_dbg_bus = UART_open(Board_uart_dbg_bus, &uartParams);

//     /* Configure the LED pin */
//     GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//     len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, " \n \n------Starting BMX test----- \n \n");
//     UART_write(uart_dbg_bus, uartTxBuffer, len);

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
//     UART_write(uart_dbg_bus, "I2C closed!\n", 10);
//     free(uartTxBuffer);
//     return (NULL);
// }

bool bmxMag_init()
{
    /* Create I2C for usage */
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    I2C_Params_init(&i2cParams); // try blocking mode

    i2c = I2C_open(I2C_SEN2, &i2cParams);

    usleep(200000);
    bool check = 0;

    /* check if i2c handle is assigned */
    check |= bmxMag_check_i2c();
    usleep(200000);

    /* bmx can only goto normal mode via sleep mode.
     * currently, sensor was just powered on and is in suspend mode */
    check |= bmxMag_goto_sleep();
    usleep(200000);

    /* read chip ID, should return 0x32 if everything is correct */
    check |= bmxMag_read_id();
    usleep(200000);

    /* (BMM050_NORMAL) data rate is 10 Hz, aka refresh rate */
    check |= bmxMag_set_datarate();
    usleep(200000);

    /* Robert asks for 10 repetitions before flushing */
    check |= bmxMag_set_repetitions();
    usleep(200000);

    /* example to read the number of repetitions */
    /* read the xy repetitions */
    check |= bmxMag_get_repetitions();
    usleep(200000);

    /* init done. caliberate now */
    check |= bmxMag_read_trim();
    check |= bmxMag_get_bias();
    return check;
}

//#define DBG

bool bmxMag_check_i2c()
{
	/* 
	// check accelerometer 
    txBuffer[0] = 0x08;
    i2cTransaction.slaveAddress = BMX_ACC;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;
    
    check = I2C_transfer(i2c1, &i2cTransaction);
    check = I2C_transfer(i2c, &i2cTransaction);
	*/
    #ifdef DBG
    bool check = 0;
    len = snprintf(uartTxBuffer, 30, "temp: 0x%02x, %d\n", rxBuffer[0], check);
    UART_write(uart_dbg_bus, uartTxBuffer, len);
    #endif

    if (i2c == NULL) {
    #ifdef DBG
            len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "Error Initializing I2C \n");
            UART_write(uart_dbg_bus, uartTxBuffer, len);
    #endif
        return 1;
    }
    else {
    #ifdef DBG
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "I2C Initialized! \n");
        UART_write(uart_dbg_bus, uartTxBuffer, len);
    #endif
        return 0;
    }
}

bool bmxMag_goto_sleep()
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
		#ifdef DBG
		len = snprintf(uartTxBuffer, 30, "sleep mode active! \n");
		UART_write(uart_dbg_bus, uartTxBuffer, len);
		#endif
        return 0;
    }
    else {
        return 1;
    }
}


/*
 * should return 0x32 indefinite of the alternate i2c address
 */
bool bmxMag_read_id()
{
    txBuffer[0] = 0x40;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf     = txBuffer;
    i2cTransaction.writeCount   = 1;
    i2cTransaction.readBuf      = rxBuffer;
    i2cTransaction.readCount    = 1;

    bool check = 0;
    if (I2C_transfer(i2c, &i2cTransaction)) {
    #ifdef DBG
		len = snprintf(uartTxBuffer, 30, "ID: 0x%02x, %d\n", rxBuffer[0], check);
		UART_write(uart_dbg_bus, uartTxBuffer, len);
	#endif
		return 0;
	}
	else {
		return 1;
	}
}

bool bmxMag_set_datarate()
{
    txBuffer[0] = 0x4C;
    txBuffer[1] = 0x00;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
		#ifdef DBG
			len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "Data rate: 10 Hz, Sensor Mode: Normal \n");
			UART_write(uart_dbg_bus, uartTxBuffer, len);
		#endif
        return 0;
    }
    else {
        return 1;
    }
}

bool bmxMag_set_repetitions()
{
    bool check;
    /* Set the XY-repetitions number for Normal mode */
    txBuffer[0] = 0x51;
    txBuffer[1] = 0x04;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
		#ifdef DBG
			len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "XY repetitions: 9 \n");
			UART_write(uart_dbg_bus, uartTxBuffer, len);
		#endif
        check = 0;
    }
    else {
        check = 1;
    }

    usleep(200000);

    /* Set the Z-repetitions number for Normal mode */
    txBuffer[0] = 0x52;
    txBuffer[1] = 0x0F;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
#ifdef DBG
        len = snprintf(uartTxBuffer, UART_BUFFER_SIZE, "Z repetitions: 15 \n");
        UART_write(uart_dbg_bus, uartTxBuffer, len);
#endif
        check |= 0;
    }
    else {
        check = 1;
    }
    return check;
}

bool bmxMag_get_repetitions()
{
    txBuffer[0] = 0x51;
    //txBuffer[1] = 0x40;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;

    if (I2C_transfer(i2c, &i2cTransaction)) {
		#ifdef DBG
			len = snprintf(uartTxBuffer, 30, "(should be 0x04) 0x%02x \n", rxBuffer[0]);
			UART_write(uart_dbg_bus, uartTxBuffer, len);
		#endif
		return 0;
    }
    else {
        return 1;
    }
}

void bmxMag_get_raw_data()
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
			#ifdef DBG
				len = snprintf(uartTxBuffer, 30, "bh: %d bv: %d bz: %d \n", bh, bv, bz);
				UART_write(uart_dbg_bus, uartTxBuffer, len);
			#endif
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
UART_write(uart_dbg_bus, uartTxBuffer, len);
*/

struct bmm050_t bmm;
/* time for challenging factory trim */
bool bmxMag_read_trim()
{
    usleep(100000);
    bool check;
    txBuffer[0] = BMM050_DIG_X1;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
		#ifdef DBG
			len = snprintf(uartTxBuffer, 50, "DIG_X1: 0x%02x, DIG_Y1: 0x%02x \n", rxBuffer[0], rxBuffer[1]);
			UART_write(uart_dbg_bus, uartTxBuffer, len);
		#endif
        check = 0;
    }
    else {
        check = 1;
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
#ifdef DBG
        len = snprintf(uartTxBuffer, 50, "DIG_X2: 0x%02x, DIG_Y2: 0x%02x \n", rxBuffer[0], rxBuffer[1]);
        UART_write(uart_dbg_bus, uartTxBuffer, len);
#endif
        check |= 0;
    }
    else {
        check = 1;
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
#ifdef DBG
        len = snprintf(uartTxBuffer, 50, "DIG_XY2: 0x%02x, DIG_XY1: 0x%02x \n", rxBuffer[0], rxBuffer[1]);
        UART_write(uart_dbg_bus, uartTxBuffer, len);
#endif
        check |= 0;
    }
    else {
        check = 1;
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
#ifdef DBG
        len = snprintf(uartTxBuffer, 50, "DIG_Z1_LSB: 0x%02x, DIG_Z1_MSB: 0x%02x, Cmb: 0x%04x \n", rxBuffer[0], rxBuffer[1], bmm.dig_z1);
        UART_write(uart_dbg_bus, uartTxBuffer, len);
#endif
        check |= 0;
    }
    else {
        check = 1;
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
#ifdef DBG
        len = snprintf(uartTxBuffer, 50, "DIG_Z2_LSB: 0x%02x, DIG_Z2_MSB: 0x%02x, Cmb: 0x%04x \n", rxBuffer[0], rxBuffer[1], bmm.dig_z2);
        UART_write(uart_dbg_bus, uartTxBuffer, len);
#endif
        check |= 0;
    }
    else {
        check = 1;
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
		#ifdef DBG
			len = snprintf(uartTxBuffer, 50, "DIG_XYZ1_LSB: 0x%02x, DIG_XYZ2_MSB: 0x%02x, Cmb: 0x%04x \n", rxBuffer[0], rxBuffer[1], bmm.dig_xyz1);
			UART_write(uart_dbg_bus, uartTxBuffer, len);
		#endif
        check |= 0;
    }
    else {
        check = 1;
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
		#ifdef DBG
			len = snprintf(uartTxBuffer, 50, "DIG_Z3_LSB: 0x%02x, DIG_Z3_MSB: 0x%02x, Cmb: 0x%04x \n", rxBuffer[0], rxBuffer[1], bmm.dig_z3);
			UART_write(uart_dbg_bus, uartTxBuffer, len);
		#endif
       check |= 0;
    }
    else {
        check = 1;
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
		#ifdef DBG
			len = snprintf(uartTxBuffer, 50, "DIG_Z4_LSB: 0x%02x, DIG_Z4_MSB: 0x%02x, Cmb: 0x%04x \n", rxBuffer[0], rxBuffer[1], bmm.dig_z4);
			UART_write(uart_dbg_bus, uartTxBuffer, len);
		#endif
        check |= 0;
    }
    else {
        check = 1;
    }
    usleep(100000);
    return check;
}

int16_t bias_magData[3];
/* Make sure you do a figure of eight */
bool bmxMag_get_bias()//(int16_t * magData)
{
    bool check = 0;
	#ifdef DBG
		len = snprintf(uartTxBuffer, 50, "starting bias calib\n");
		UART_write(uart_dbg_bus, uartTxBuffer, len);
	#endif
    int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
    uint16_t data_r = 0;
    uint8_t rxBuffer[8];  // x/y/z hall magnetic field data, and Hall resistance data
    int16_t magData[3];

    int16_t min_magData[3] = {0,0,0}; // maybe use some other initialization
    int16_t max_magData[3] = {0,0,0};


    txBuffer[0] = 0x42;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 8;
    int bias_cnt;
    for(bias_cnt = 0; bias_cnt < 100; bias_cnt++) {
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

                /* looks redundant */
                //temp = ((int16_t)(((uint16_t)((((int32_t)bmm.dig_xyz1) << 14)/(data_r != 0 ? data_r : bmm.dig_xyz1))) - ((uint16_t)0x4000)));

                magData[1] = ((int16_t)((((int32_t)mdata_y) *
                ((((((((int32_t)bmm.dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
                (((int32_t)temp) * ((int32_t)(((int16_t)bmm.dig_xy1) << 7)))) >> 9) +
                ((int32_t)0x100000)) * ((int32_t)(((int16_t)bmm.dig_y2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
                (((int16_t)bmm.dig_y1) << 3);

                magData[2] = (((((int32_t)(mdata_z - bmm.dig_z4)) << 15) - ((((int32_t)bmm.dig_z3) * ((int32_t)(((int16_t)data_r) -
                ((int16_t)bmm.dig_xyz1))))>>2))/(bmm.dig_z2 + ((int16_t)(((((int32_t)bmm.dig_z1) * ((((int16_t)data_r) << 1)))+(1<<15))>>16))));
                int i;
                for (i=0; i<3; i++) {
                    max_magData[i] = MAX(max_magData[i], magData[i]);
                    min_magData[i] = MIN(min_magData[i], magData[i]);
                }
                usleep(100000);
            }
            check |= 0;
        }
        else {
            check = 1;
        }
    }
    int j;
    // comment later: no need to plot min max, go for bias
    for (j=0; j<3; j++) {
		#ifdef DBG
			len = snprintf(uartTxBuffer, 50, "axis: %d, max: %d, min: %d\n", j, max_magData[j], min_magData[j]);
			UART_write(uart_dbg_bus, uartTxBuffer, len);
		#endif
        usleep(100000);
    }

    j = 0;
    // explain why average of min max is a good way to go, and not average of all readings
    for (j=0; j<3; j++) {
        bias_magData[j] = (min_magData[j] + max_magData[j]) >> 1;
    }
	#ifdef DBG
		len = snprintf(uartTxBuffer, 50, "bias: x: %d y: %d z: %d\n", bias_magData[0], bias_magData[1], bias_magData[2]);
		UART_write(uart_dbg_bus, uartTxBuffer, len);
	#endif
    usleep(100000);
    return check;
}


void bmxMag_read_calib_data(int16_t comp_MagData[3])//(int16_t * magData)
{
    int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
    uint16_t data_r = 0;
    uint8_t rxBuffer[8];  // x/y/z hall magnetic field data, and Hall resistance data
    int16_t magData[3];
    int16_t comp_magData[3];
    txBuffer[0] = 0x42;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 8;
    
    if (I2C_transfer(i2c, &i2cTransaction)) {
        if(rxBuffer[6] & 0x01) { // Check if data ready status bit is set
            /*
            mdata_x = (int16_t) (((int16_t)rxBuffer[1] << 8)  | rxBuffer[0]) >> 3;  // 13-bit signed integer for x-axis field
            mdata_y = (    while(!start_flag) {
        usleep(1000);
    }int16_t) (((int16_t)rxBuffer[3] << 8)  | rxBuffer[2]) >> 3;  // 13-bit signed integer for y-axis field
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
            int cnt = 0;
            for (cnt = 0; cnt<3; cnt++) {
                comp_magData[cnt] = magData[cnt] - bias_magData[cnt];
            }
			#ifdef DBG
				len = snprintf(uartTxBuffer, 50, "bh: %d bv: %d bz: %d\n", comp_magData[0], comp_magData[1], comp_magData[2]);
				UART_write(uart_dbg_bus, uartTxBuffer, len);
			#endif
            // usleep(100000);
        }
    }
}
