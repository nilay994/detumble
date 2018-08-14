#ifndef __BMM050_H__
#define __BMM050_H__

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

#define BMM050_TRIM_DATA_SIZE 	2
#define BMM050_INIT_VALUE 		0

/*!
 * @brief Structure containing mag initial parameters
 */
struct trim_mag_t {
	int8_t dig_x1;		/**< trim x1 data */
	int8_t dig_y1;		/**< trim y1 data */
	int8_t dig_x2;		/**< trim x2 data */
	int8_t dig_y2;		/**< trim y2 data */

	uint16_t dig_z1;	/**< trim z1 data */
	int16_t  dig_z2;	/**< trim z2 data */
	int16_t  dig_z3;	/**< trim z3 data */
	int16_t  dig_z4;	/**< trim z4 data */

	uint8_t dig_xy1;	/**< trim xy1 data */
	int8_t  dig_xy2; 	/**< trim xy2 data */

	uint16_t dig_xyz1;  /**< trim xyz1 data */
};

void read_factory_trim(struct trim_mag_t *arg_trim_mag);

#endif