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

#ifndef _I2C_BMX_H
#define _I2C_BMX_H

#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

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


bool bmxMag_init();

bool bmxMag_check_i2c();
bool bmxMag_read_id();

bool bmxMag_goto_sleep();
bool bmxMag_set_datarate();

bool bmxMag_set_repetitions();
bool bmxMag_get_repetitions();

void bmxMag_get_raw_data();

bool bmxMag_read_trim();
bool bmxMag_get_bias();
void bmxMag_read_calib_data(int16_t comp_MagData[3]);

#endif
