/***************************************************************************//**
 * @file
 * @brief Application init for FreeRTOS.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "sl_status.h"
#include "sl_sensor_rht.h"
#include "app_log.h"
#include "app_assert.h"
#include "app.h"

#include "sl_i2cspm_devices_config.h"
#include "rotation_driver.h"
#include "lis3dh.h"
#include "lis2d.h"
#include "lis2mdl.h"
#include "lps22hh.h"
#include "tof_driver.h"
#include "VL53L1X_api.h"

#define APP_INIT_TASK_NAME          "app_init"
#define APP_INIT_TASK_STACK_SIZE    400
#define APP_INIT_TASK_STATIC        0

#if (APP_INIT_TASK_STATIC == 1)
StackType_t app_init_task_stack[APP_INIT_TASK_STACK_SIZE];
StackType_t lis3d_interrupt_task_stack[APP_INIT_TASK_STACK_SIZE];
StaticTask_t app_init_task_handle;
StaticTask_t lis3d_interrupt_task_handle;
#else // configSUPPORT_STATIC_ALLOCATION
TaskHandle_t app_init_task_handle = NULL;
#endif // configSUPPORT_STATIC_ALLOCATION

/**************************************************************************//**
 * FreeRTOS Task for Application Init.
 *****************************************************************************/
void app_init_task(void *p_arg)
{
  (void)p_arg;
  sl_status_t sc;
  app_log_info("Health thermometer initialised\n");
  // Init temperature sensor.
  sc = sl_sensor_rht_init();
  if (sc != SL_STATUS_OK) {
    app_log_warning("Relative Humidity and Temperature sensor initialization failed.");
    app_log_nl();
  }

  #ifdef LIS3D
  static lis3dh_sensor_t* sensor = 0;
  // init the sensor with slave address LIS3DH_I2C_ADDRESS_2 connected to I2C_BUS.
  sensor = lis3dh_init_sensor (I2C0, LIS3DH_I2C_ADDRESS_2, 0);
  if (sensor)
  {
    app_log_info("LIS3D initialised\n");
    rotation_init (sensor);
  }
  #endif

  //#ifdef LIS2D
  static lis2d_sensor_t* sensor_acc = 0;
  // init the sensor with slave address LIS2D_I2C_ADDRESS_2 connected to I2C_BUS.
  sensor_acc = lis2d_init_sensor (I2C0, LIS2DTW12_I2C_ADD_L, 0);
  if (sensor_acc)
  {
    app_log_info("LIS2D initialised\n");
  }
  //#endif

  //#ifdef LIS2MDL
  static lis2mdl_sensor_t* sensor_mg = 0;
  // init the sensor with slave address LIS2D_I2C_ADDRESS_2 connected to I2C_BUS.
  sensor_mg = lis2mdl_init_sensor (I2C0, LIS2MDL_I2C_ADD, 0);
  if (sensor_mg)
  {
    app_log_info("LIS2MDL initialised\n");
  }
  //#endif

  //#ifdef LPS22HH
  static lps22hh_sensor_t* sensor_pr = 0;
  // init the sensor with slave address LIS2D_I2C_ADDRESS_2 connected to I2C_BUS.
  sensor_pr = lps22hh_init_sensor (I2C0, LPS22HH_I2C_ADD_L, 0);
  if (sensor_pr)
  {
    app_log_info("LPS22HH initialised\n");
  }
  //#endif

  //#ifdef ToF
  if (tof_init () == 0)
  {
    app_log_info("ToF initialised\n");
  }
  //#endif
  vTaskDelete(NULL);
}

/**************************************************************************//**
 * Application Init, overrides weak implementation
 *****************************************************************************/
void app_init(void)
{
  // Create a task for init to be started by the scheduler.
  #if (APP_INIT_TASK_STATIC == 1)
  xTaskCreateStatic(app_init_task,
                    APP_INIT_TASK_NAME,
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY,
                    app_init_task_stack,
                    &app_init_task_handle);

  // Create the interrupt task
//  xTaskCreateStatic(lis3d_interrupt_task,
//               "LIS3D_interrupt_task",
//               configMINIMAL_STACK_SIZE,
//               NULL,
//               tskIDLE_PRIORITY + 1,
//               lis3d_interrupt_task_stack,
//               &lis3d_interrupt_task_handle);
   vTaskStartScheduler();
  #else // configSUPPORT_STATIC_ALLOCATION
  xTaskCreate(app_init_task,
              APP_INIT_TASK_NAME,
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY,
              &app_init_task_handle);
  // Create the interrupt task
//  xTaskCreate(lis3d_interrupt_task,
//               "LIS3D_interrupt_task",
//               configMINIMAL_STACK_SIZE,
//               NULL,
//               tskIDLE_PRIORITY + 1,
//               NULL);   // Start the FreeRTOS scheduler
  #endif // configSUPPORT_STATIC_ALLOCATION
}
