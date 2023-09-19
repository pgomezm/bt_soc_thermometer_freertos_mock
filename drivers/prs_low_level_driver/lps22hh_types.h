/**
 * Driver for LIS2D 3-axes digital accelerometer connected to I2C or SPI.
 *
 */

#ifndef __LPS22HH_TYPES_H__
#define __LPS22HH_TYPES_H__

#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @brief   lis2d sensor device data structure type
 */
typedef struct {

    int          error_code;        // error code of last operation

    I2C_TypeDef *bus;               // I2C = x, SPI = 1
    uint8_t     addr;               // I2C = slave address, SPI = 0

    uint8_t     cs;                 // ESP8266, ESP32: GPIO used as SPI CS
                                    // __linux__: device index
} lps22hh_sensor_t;
                                 

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* __LPS22HH_TYPES_H__ */
