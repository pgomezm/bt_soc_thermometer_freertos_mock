/*
 * i2c_low_level_driver.h
 *
 *  Created on: 11 Aug 2023
 *      Author: Pablo Gomez Martino
 */

#ifndef DRIVERS_I2C_LOW_LEVEL_DRIVER_I2C_LOW_LEVEL_DRIVER_H_
#define DRIVERS_I2C_LOW_LEVEL_DRIVER_I2C_LOW_LEVEL_DRIVER_H_

#include "sl_i2cspm.h"

I2C_TransferReturn_TypeDef i2c_slave_read  (I2C_TypeDef *i2c, uint8_t addr, uint8_t command, uint8_t *val, uint8_t len);
I2C_TransferReturn_TypeDef i2c_slave_write (I2C_TypeDef *i2c, uint8_t addr, uint8_t command, uint8_t *val, uint8_t len);

#endif /* DRIVERS_I2C_LOW_LEVEL_DRIVER_I2C_LOW_LEVEL_DRIVER_H_ */
