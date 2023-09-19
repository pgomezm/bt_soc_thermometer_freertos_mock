/**
  ******************************************************************************
  * @file    lis2mdl_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          lis2mdl.c driver.
  ******************************************************************************
  *
  ******************************************************************************
  */
#ifndef __LIS2MDL_H__
#define __LIS2MDL_H__

/**
  ******************************************************************************
  * @file    lis2mdl_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          lis2mdl_reg.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "lis2mdl_types.h"

/** @addtogroup LIS2MDL
  * @{
  *
  */

/** @defgroup  Endianness definitions
  * @{
  *
  */

#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN  __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN     __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER     __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

/**
  * @}
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_mg_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */



/**
  * @}
  *
  */

/** @defgroup LSM9DS1_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format **/
#define LIS2MDL_I2C_ADD                 (0x3DU >> 1)

/** Device Identification (Who am I) **/
#define LIS2MDL_ID                      0x40U

/**
  * @}
  *
  */

#define LIS2MDL_OFFSET_X_REG_L          0x45U
#define LIS2MDL_OFFSET_X_REG_H          0x46U
#define LIS2MDL_OFFSET_Y_REG_L          0x47U
#define LIS2MDL_OFFSET_Y_REG_H          0x48U
#define LIS2MDL_OFFSET_Z_REG_L          0x49U
#define LIS2MDL_OFFSET_Z_REG_H          0x4AU
#define LIS2MDL_WHO_AM_I                0x4FU
#define LIS2MDL_CFG_REG_A               0x60U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t md                     : 2;
  uint8_t odr                    : 2;
  uint8_t lp                     : 1;
  uint8_t soft_rst               : 1;
  uint8_t reboot                 : 1;
  uint8_t comp_temp_en           : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t comp_temp_en           : 1;
  uint8_t reboot                 : 1;
  uint8_t soft_rst               : 1;
  uint8_t lp                     : 1;
  uint8_t odr                    : 2;
  uint8_t md                     : 2;
#endif /* DRV_BYTE_ORDER */
} lis2mdl_cfg_reg_a_t;

#define LIS2MDL_CFG_REG_B               0x61U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lpf                    : 1;
  uint8_t set_rst                : 2; /* OFF_CANC + Set_FREQ */
  uint8_t int_on_dataoff         : 1;
  uint8_t off_canc_one_shot      : 1;
  uint8_t not_used_01            : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01            : 3;
  uint8_t off_canc_one_shot      : 1;
  uint8_t int_on_dataoff         : 1;
  uint8_t set_rst                : 2; /* OFF_CANC + Set_FREQ */
  uint8_t lpf                    : 1;
#endif /* DRV_BYTE_ORDER */
} lis2mdl_cfg_reg_b_t;

#define LIS2MDL_CFG_REG_C               0x62U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t drdy_on_pin            : 1;
  uint8_t self_test              : 1;
  uint8_t _4wspi                 : 1;
  uint8_t ble                    : 1;
  uint8_t bdu                    : 1;
  uint8_t i2c_dis                : 1;
  uint8_t int_on_pin             : 1;
  uint8_t not_used_02            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02            : 1;
  uint8_t int_on_pin             : 1;
  uint8_t i2c_dis                : 1;
  uint8_t bdu                    : 1;
  uint8_t ble                    : 1;
  uint8_t _4wspi                 : 1;
  uint8_t self_test              : 1;
  uint8_t drdy_on_pin            : 1;
#endif /* DRV_BYTE_ORDER */
} lis2mdl_cfg_reg_c_t;

#define LIS2MDL_INT_CRTL_REG            0x63U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ien                    : 1;
  uint8_t iel                    : 1;
  uint8_t iea                    : 1;
  uint8_t not_used_01            : 2;
  uint8_t zien                   : 1;
  uint8_t yien                   : 1;
  uint8_t xien                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t xien                   : 1;
  uint8_t yien                   : 1;
  uint8_t zien                   : 1;
  uint8_t not_used_01            : 2;
  uint8_t iea                    : 1;
  uint8_t iel                    : 1;
  uint8_t ien                    : 1;
#endif /* DRV_BYTE_ORDER */
} lis2mdl_int_crtl_reg_t;

#define LIS2MDL_INT_SOURCE_REG          0x64U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t _int                   : 1;
  uint8_t mroi                   : 1;
  uint8_t n_th_s_z               : 1;
  uint8_t n_th_s_y               : 1;
  uint8_t n_th_s_x               : 1;
  uint8_t p_th_s_z               : 1;
  uint8_t p_th_s_y               : 1;
  uint8_t p_th_s_x               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_th_s_x               : 1;
  uint8_t p_th_s_y               : 1;
  uint8_t p_th_s_z               : 1;
  uint8_t n_th_s_x               : 1;
  uint8_t n_th_s_y               : 1;
  uint8_t n_th_s_z               : 1;
  uint8_t mroi                   : 1;
  uint8_t _int                   : 1;
#endif /* DRV_BYTE_ORDER */
} lis2mdl_int_source_reg_t;

#define LIS2MDL_INT_THS_L_REG           0x65U
#define LIS2MDL_INT_THS_H_REG           0x66U
#define LIS2MDL_STATUS_REG              0x67U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xda                    : 1;
  uint8_t yda                    : 1;
  uint8_t zda                    : 1;
  uint8_t zyxda                  : 1;
  uint8_t _xor                   : 1;
  uint8_t yor                    : 1;
  uint8_t zor                    : 1;
  uint8_t zyxor                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t zyxor                  : 1;
  uint8_t zor                    : 1;
  uint8_t yor                    : 1;
  uint8_t _xor                   : 1;
  uint8_t zyxda                  : 1;
  uint8_t zda                    : 1;
  uint8_t yda                    : 1;
  uint8_t xda                    : 1;
#endif /* DRV_BYTE_ORDER */
} lis2mdl_status_reg_t;

#define LIS2MDL_OUTX_L_REG              0x68U
#define LIS2MDL_OUTX_H_REG              0x69U
#define LIS2MDL_OUTY_L_REG              0x6AU
#define LIS2MDL_OUTY_H_REG              0x6BU
#define LIS2MDL_OUTZ_L_REG              0x6CU
#define LIS2MDL_OUTZ_H_REG              0x6DU
#define LIS2MDL_TEMP_OUT_L_REG          0x6EU
#define LIS2MDL_TEMP_OUT_H_REG          0x6FU

/**
  * @defgroup LIS2MDL_Register_Union
  * @brief    This union group all the registers having a bit-field
  *           description.
  *           This union is useful but it's not needed by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union
{
  lis2mdl_cfg_reg_a_t            cfg_reg_a;
  lis2mdl_cfg_reg_b_t            cfg_reg_b;
  lis2mdl_cfg_reg_c_t            cfg_reg_c;
  lis2mdl_int_crtl_reg_t         int_crtl_reg;
  lis2mdl_int_source_reg_t       int_source_reg;
  lis2mdl_status_reg_t           status_reg;
  bitwise_mg_t                   bitwise;
  uint8_t                        byte;
} lis2mdl_reg_t;

/**
  * @}
  *
  */

float_t lis2mdl_from_lsb_to_mgauss(int16_t lsb);

float_t lis2mdl_from_lsb_to_celsius(int16_t lsb);

int32_t lis2mdl_mag_user_offset_set(lis2mdl_sensor_t* dev, int16_t *val);
int32_t lis2mdl_mag_user_offset_get(lis2mdl_sensor_t* dev, int16_t *val);

typedef enum
{
  LIS2MDL_CONTINUOUS_MODE  = 0,
  LIS2MDL_SINGLE_TRIGGER   = 1,
  LIS2MDL_POWER_DOWN       = 2,
} lis2mdl_md_t;
int32_t lis2mdl_operating_mode_set(lis2mdl_sensor_t* dev,
                                   lis2mdl_md_t val);
int32_t lis2mdl_operating_mode_get(lis2mdl_sensor_t* dev,
                                   lis2mdl_md_t *val);

typedef enum
{
  LIS2MDL_ODR_10Hz   = 0,
  LIS2MDL_ODR_20Hz   = 1,
  LIS2MDL_ODR_50Hz   = 2,
  LIS2MDL_ODR_100Hz  = 3,
} lis2mdl_odr_t;
int32_t lis2mdl_data_rate_set(lis2mdl_sensor_t* dev, lis2mdl_odr_t val);
int32_t lis2mdl_data_rate_get(lis2mdl_sensor_t* dev, lis2mdl_odr_t *val);

typedef enum
{
  LIS2MDL_HIGH_RESOLUTION  = 0,
  LIS2MDL_LOW_POWER        = 1,
} lis2mdl_lp_t;
int32_t lis2mdl_power_mode_set(lis2mdl_sensor_t* dev, lis2mdl_lp_t val);
int32_t lis2mdl_power_mode_get(lis2mdl_sensor_t* dev, lis2mdl_lp_t *val);

int32_t lis2mdl_offset_temp_comp_set(lis2mdl_sensor_t* dev, uint8_t val);
int32_t lis2mdl_offset_temp_comp_get(lis2mdl_sensor_t* dev, uint8_t *val);

typedef enum
{
  LIS2MDL_ODR_DIV_2  = 0,
  LIS2MDL_ODR_DIV_4  = 1,
} lis2mdl_lpf_t;
int32_t lis2mdl_low_pass_bandwidth_set(lis2mdl_sensor_t* dev,
                                       lis2mdl_lpf_t val);
int32_t lis2mdl_low_pass_bandwidth_get(lis2mdl_sensor_t* dev,
                                       lis2mdl_lpf_t *val);

typedef enum
{
  LIS2MDL_SET_SENS_ODR_DIV_63        = 0,
  LIS2MDL_SENS_OFF_CANC_EVERY_ODR    = 1,
  LIS2MDL_SET_SENS_ONLY_AT_POWER_ON  = 2,
} lis2mdl_set_rst_t;

int32_t lis2mdl_set_rst_mode_set(lis2mdl_sensor_t* dev, lis2mdl_set_rst_t val);
int32_t lis2mdl_set_rst_mode_get(lis2mdl_sensor_t* dev, lis2mdl_set_rst_t *val);

int32_t lis2mdl_set_rst_sensor_single_set(lis2mdl_sensor_t* dev, uint8_t val);
int32_t lis2mdl_set_rst_sensor_single_get(lis2mdl_sensor_t* dev, uint8_t *val);

int32_t lis2mdl_block_data_update_set(lis2mdl_sensor_t* dev, uint8_t val);
int32_t lis2mdl_block_data_update_get(lis2mdl_sensor_t* dev, uint8_t *val);

int32_t lis2mdl_mag_data_ready_get(lis2mdl_sensor_t* dev, uint8_t *val);

int32_t lis2mdl_mag_data_ovr_get(lis2mdl_sensor_t* dev, uint8_t *val);

int32_t lis2mdl_magnetic_raw_get(lis2mdl_sensor_t* dev, int16_t *val);

int32_t lis2mdl_temperature_raw_get(lis2mdl_sensor_t* dev,  int16_t *val);

int32_t lis2mdl_device_id_get(lis2mdl_sensor_t* dev, uint8_t *buff);

int32_t lis2mdl_reset_set(lis2mdl_sensor_t* dev, uint8_t val);
int32_t lis2mdl_reset_get(lis2mdl_sensor_t* dev, uint8_t *val);

int32_t lis2mdl_boot_set(lis2mdl_sensor_t* dev, uint8_t val);
int32_t lis2mdl_boot_get(lis2mdl_sensor_t* dev, uint8_t *val);

int32_t lis2mdl_self_test_set(lis2mdl_sensor_t* dev, uint8_t val);
int32_t lis2mdl_self_test_get(lis2mdl_sensor_t* dev, uint8_t *val);

typedef enum
{
  LIS2MDL_LSB_AT_LOW_ADD  = 0,
  LIS2MDL_MSB_AT_LOW_ADD  = 1,
} lis2mdl_ble_t;

int32_t lis2mdl_data_format_set(lis2mdl_sensor_t* dev, lis2mdl_ble_t val);
int32_t lis2mdl_data_format_get(lis2mdl_sensor_t* dev, lis2mdl_ble_t *val);

int32_t lis2mdl_status_get(lis2mdl_sensor_t* dev, lis2mdl_status_reg_t *val);

typedef enum
{
  LIS2MDL_CHECK_BEFORE  = 0,
  LIS2MDL_CHECK_AFTER   = 1,
} lis2mdl_int_on_dataoff_t;
int32_t lis2mdl_offset_int_conf_set(lis2mdl_sensor_t* dev,
                                    lis2mdl_int_on_dataoff_t val);
int32_t lis2mdl_offset_int_conf_get(lis2mdl_sensor_t* dev,
                                    lis2mdl_int_on_dataoff_t *val);

int32_t lis2mdl_drdy_on_pin_set(lis2mdl_sensor_t* dev, uint8_t val);
int32_t lis2mdl_drdy_on_pin_get(lis2mdl_sensor_t* dev, uint8_t *val);

int32_t lis2mdl_int_on_pin_set(lis2mdl_sensor_t* dev, uint8_t val);
int32_t lis2mdl_int_on_pin_get(lis2mdl_sensor_t* dev, uint8_t *val);

int32_t lis2mdl_int_gen_conf_set(lis2mdl_sensor_t* dev,
                                 lis2mdl_int_crtl_reg_t *val);
int32_t lis2mdl_int_gen_conf_get(lis2mdl_sensor_t* dev,
                                 lis2mdl_int_crtl_reg_t *val);

int32_t lis2mdl_int_gen_source_get(lis2mdl_sensor_t* dev,
                                   lis2mdl_int_source_reg_t *val);

int32_t lis2mdl_int_gen_treshold_set(lis2mdl_sensor_t* dev, uint16_t val);
int32_t lis2mdl_int_gen_treshold_get(lis2mdl_sensor_t* dev,
                                     uint16_t *val);

typedef enum
{
  LIS2MDL_SPI_4_WIRE   = 1,
  LIS2MDL_SPI_3_WIRE   = 0,
} lis2mdl_sim_t;
int32_t lis2mdl_spi_mode_set(lis2mdl_sensor_t* dev, lis2mdl_sim_t val);
int32_t lis2mdl_spi_mode_get(lis2mdl_sensor_t* dev, lis2mdl_sim_t *val);

typedef enum
{
  LIS2MDL_I2C_ENABLE   = 0,
  LIS2MDL_I2C_DISABLE  = 1,
} lis2mdl_i2c_dis_t;
int32_t lis2mdl_i2c_interface_set(lis2mdl_sensor_t* dev,
                                  lis2mdl_i2c_dis_t val);
int32_t lis2mdl_i2c_interface_get(lis2mdl_sensor_t* dev,
                                  lis2mdl_i2c_dis_t *val);

/**
  *@}
  *
  */

// ---- Low level interface functions -----------------------------
/**
 * @brief   Initialize the sensor
 *
 * Reset the sensor and switch to power down mode. All registers are reset to
 * default values. FIFO is cleared.
 *
 * @param   bus     I2C or SPI bus at which LI2MDL sensor is connected
 * @param   addr    I2C addr of the LIS2MDL sensor, 0 for using SPI
 * @param   cs      SPI CS GPIO, ignored for I2C
 * @return          pointer to sensor data structure, or NULL on error
 */
lis2mdl_sensor_t* lis2mdl_init_sensor (I2C_TypeDef *bus, uint8_t addr, uint8_t cs);
/**
 * @brief   Direct write to register
 *
 * PLEASE NOTE: This function should only be used to do something special that
 * is not covered by the high level interface AND if you exactly know what you
 * do and what effects it might have. Please be aware that it might affect the
 * high level interface.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   reg      address of the first register to be changed
 * @param   data     pointer to the data to be written to the register
 * @param   len      number of bytes to be written to the register
 * @return           true on success, false on error
 */
bool lis2mdl_reg_write (lis2mdl_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief   Direct read from register
 *
 * PLEASE NOTE: This function should only be used to do something special that
 * is not covered by the high level interface AND if you exactly know what you
 * do and what effects it might have. Please be aware that it might affect the
 * high level interface.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   reg      address of the first register to be read
 * @param   data     pointer to the data to be read from the register
 * @param   len      number of bytes to be read from the register
 * @return           true on success, false on error
 */
bool lis2mdl_reg_read (lis2mdl_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#endif /* __LIS2MDL_H__ */
