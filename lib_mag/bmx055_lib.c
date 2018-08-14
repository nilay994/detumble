/*
 * reference: Robert's algorithm doc v0.2
 *----------------------------------------------------Detumble initialization----------------------------------------------
 *--------------------------------------------------+-----------------------+------------------------+-----------+---------+
 * description                                      | variable              | initial value          | dimension | unit    |
 * ctrl loop freq preset                            | detumble_ctrl_freq    | 4                      | 1         | Hz      |
 * control loop period                              | detumble_period_Tc    | 250                    | 1         | ms      |
 * duty cycle of mtq                                | detumble_duty_mtq     | 0.6                    | 1         | none    |
 * max actuation period                             | detumble_period_Ta    | deltaTc (TBD?)         | 1         | sec     |
 * B-dot gain                                       | detumble_kw           | 1.1422*10^-6           | 1         | Am^2s/T |
 * max dipole moment x, y, z                        | detumble_max_moment   | 0.002                  | 1         | Am^2    |
 * torque rod polarity x, y, z {-1, 1}              | detumble_rod_polarity | 1                      | 1         | -       |
 * weight factor mtm1,2                             | detumble_mag1_weight  | 0.5                    | 1         | -       |
 * bias vector mtm1,2                               | detumble_x/y/zbias    | {0,0,0}/code o/p       | 3x1       | T       |
 * scaling/orientation                              | detumble_T1           | [1 0 0; 0 1 0; 0 0 -1] | 3x3       | -       |
 * tumble parameter (complementary) filter constant | detumble_alpha        | 0.01                   | 1         | -       |
 * tumble parameter threshold                       | detumble_pbar         | 2*10^-6                | 1         | T/s     |
 * detumble confirmation time                       | detumble_tconf        | 3600                   | 1         | sec     |
 * tumble parameter                                 | detuble_p_tumb        | {10^-5, 10^-5, 10^-5}  | 3x1       | T/s     |
 * counter                                          | detumble_count_tumb   | 0                      | 1         | -       |
 * desired magnetic dipole moment                   | detumble_m_desired    | (code o/p)             | 3x1       | Am2     |
 * corrected vector (mag1 and mag2)                 | detumble_b1/b2_curr   | (code o/p)             | 3x1       | T       |
 * current weighted vector                          | detumble_b_curr       | (code o/p)             | 3x1       | T       |
 * current normalized vector                        | detumble_bnorm_curr   | (code o/p)             | 3x1       | T       |
 * previous meas. vector                            | detumble_b_prev       | (code o/p)             | 3x1       | T       |
 * bdot vector                                      | detumble_b_dot        | (code o/p)             | 3x1       | T       |
 * normalized_bdot                                  | detumble_b_normdot    | (code o/p)             | 3x1       | T       |
 * hold period                                      | detumble_t_hold       | N/A                    | N/A       | sec     |
 * raw measurement vector (mtm1/2)                  | detumble_raw_mtm1/2   | (code o/p)             | 3x1       | T       |
 * vector of torque rods on times                   | detumble_t_on         | (code o/p)             | 3x1       | s       |
 * vector of torque rods on directions              | detumble_s_on {-1, 1} | (gnd station flip)     | 1         | -       |
 *--------------------------------------------------+-----------------------+------------------------+-----------+---------+
 *
 **/


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
    if (i2c_trans(&txBuffer, 2, &rxBuffer, 0) == 1) {
        len = snprintf(uartTxBuffer, 30, "sleep mode active! \n");
    }
    else {
        len = snprintf(uartTxBuffer, 30, "ERR: sleep mode set failed\n");
    }
    UART_write(uart0, uartTxBuffer, len);
}


/*
 * i2c transactions functions
 *
 * **/
bool i2c_send(I2C_Handle i2c, uint8_t* txBuffer, uint8_t writeLen, uint8_t* rxBuffer, uint8_t readLen)
{
    I2C_Transaction i2cTransaction;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = writeLen;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = readLen;
    return I2C_transfer(i2c, &i2cTransaction);
}

#if 0
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

void bmxMag_get_bias()
{
    len = snprintf(uartTxBuffer, 30, "rotate the board along all axis till next print \n");
    UART_write(uart0, uartTxBuffer, len);

    int16_t bh, bv, bz;
    /* read the axes */
    txBuffer[0] = 0x42;
    i2cTransaction.slaveAddress = BMX_MAG;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 6;
    int i;
    for(i = 0; i < 513; i++) {
        if (I2C_transfer(i2c, &i2cTransaction)) {

            bh = rxBuffer[1] << 8 | (rxBuffer[0] & 0xF8);  // lim: pm 4095
            bv = rxBuffer[3] << 8 | (rxBuffer[2] & 0xF8);  // lim: pm 4095
            bz = rxBuffer[5] << 8 | (rxBuffer[4] & 0xFE);  // lim: pm 16383

            xbias += bh;
            ybias += bv;
            zbias += bz;
        }
        usleep(100002);     // 10 Hz = 100 ms
    }
    xbias = xbias >> 9;
    ybias = ybias >> 9;
    zbias = zbias >> 9;

    len = snprintf(uartTxBuffer, 30, "xbias: %d ybias: %d zbias: %d \n", xbias, ybias, zbias);
    UART_write(uart0, uartTxBuffer, len);
}

void bmxMag_get_raw()
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

            bh = bh - xbias;
            bv = bv - ybias;
            bz = bz - zbias;

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

/* TODO
/* 1) time for challenging factory trim
 * 2) Actual uT values
 * */

void bmxAcc_get_temperature()
{
    float temperature;
    txBuffer[0] = 0x08;
    i2cTransaction.slaveAddress = BMX_ACC;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;
    while(1) {
        if (I2C_transfer(i2c, &i2cTransaction)) {
            int16_t tempCount = ((int16_t)((int16_t)rxBuffer[0] << 8)) >> 8 ;  // Turn the byte into a signed 8-bit integer
            temperature = ((((float)tempCount) * 0.5) + 23.0);   // temperature in degrees Centigrade
            len = snprintf(uartTxBuffer, 30, "temp: %f \n", temperature);
            UART_write(uart0, uartTxBuffer, len);
        }
    }
}
#endif

/* Description:
 * bmx magnetometer is in suspend mode when powered on
 * force it to sleep mode to be able to go to normal mode
 * later set the repititions for better readings
 * note: z axis is a bit different
 * also possible: pre-calculate the bias on each axis when on satellite
 * **/
void bmxMag_init(I2C_Handle i2c_handle)
{
    /* bmx can only goto normal mode via sleep mode.
     * currently, sensor was just powered on and is in suspend mode */
    bmxMag_goto_sleep(i2c_handle);
    usleep(200000);
#if 0
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

    bmxMag_get_bias();
    usleep(200000);
#endif
}

//void detumble_step1()
