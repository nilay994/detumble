#ifndef __BMX055_LIB_H
#define __BMX055_LIB_H

#include <ti/drivers/I2C.h>

bool bmxMag_init(I2C_Handle i2c_handle);
bool i2c_send(uint8_t* txBuffer, uint8_t writeLen, uint8_t* rxBuffer, uint8_t readLen);

#endif
