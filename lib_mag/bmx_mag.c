#include "bmx_mag.h"

static struct trim_mag_t *trim_mag;

void read_factory_trim(struct trim_mag_t *arg_trim_mag)
{
	trim_mag = arg_trim_mag;

	/*

	com_rslt  = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr, BMM050_DIG_X1,  (u8 *)&p_bmm050->dig_x1,  1);
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr, BMM050_DIG_Y1,  (u8 *)&p_bmm050->dig_y1,  1);
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr, BMM050_DIG_X2,  (u8 *)&p_bmm050->dig_x2,  1);
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr, BMM050_DIG_Y2,  (u8 *)&p_bmm050->dig_y2,  1);
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr, BMM050_DIG_XY1, (u8 *)&p_bmm050->dig_xy1, 1);
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr, BMM050_DIG_XY2, (u8 *)&p_bmm050->dig_xy2, 1);

	*/

	/* tbd: non consecutive i2c writes */
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
	(uint8_t *)&p_bmm050->dig_x1 = rxBuffer[0];
	(uint8_t *)&p_bmm050->dig_y1 = rxBuffer[1];

	usleep(100000);

	/* tbd: non consecutive i2c writes */
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
	(uint8_t *)&p_bmm050->dig_x2 = rxBuffer[0];
	(uint8_t *)&p_bmm050->dig_y2 = rxBuffer[1];

	usleep(100000);

	/* tbd: non consecutive i2c writes */
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
	(uint8_t *)&p_bmm050->dig_xy2 = rxBuffer[0];
	(uint8_t *)&p_bmm050->dig_xy1 = rxBuffer[1];

}