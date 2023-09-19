/*
 * tof_driver.c
 *
 *  Created on: 5 Sept 2023
 *      Author: Pablo Gomez Martino
 */
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "em_cmu.h"
#include "em_gpio.h"
//#include "task.h"
#include "app_log.h"

#include "queue.h"
#include "semphr.h"
#include "portmacro.h"
#include "projdefs.h"
#include "pin_config.h"
#include "gpiointerrupt.h"

#include "sl_assert.h"
#include "sl_i2cspm.h"
#include "sl_udelay.h"

#include "sl_i2cspm_devices_config.h"
#include "i2c_low_level_driver.h"

#include "VL53L1_platform.h"
#include "VL53L1_types.h"
#include "VL53L1X_api.h"
#include "VL53l1X_calibration.h"
//#include "X-NUCLEO-53L1A1.h"

#define VL53L1X_INT_PIN       4
#define VL53L1X_INT_PORT      gpioPortB

#define VL53L1X_I2C_ADDRESS_1           0x52  // SDO pin is low
#define VL53L1X_I2C_ADDRESS_2           0x53  // SDO pin is high

/* Private variables ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
I2C_TypeDef i2c0;

uint16_t       dev = VL53L1X_I2C_ADDRESS_1;
VL53L1X_ERROR  status = 0;
volatile int   IntCount;

static QueueHandle_t tof_evt_queue = NULL;
// User task that fetches the sensor values

void tof_task_interrupt (void *pvParameters)
{
  (void)*pvParameters;
  uint8_t gpio_num;

  while (1)
  {
    if (xQueueReceive(tof_evt_queue, &gpio_num, portMAX_DELAY))
    {
      IntCount++;
    }
  }
}

// Interrupt handler which resumes user_task_interrupt on interrupt
//static void int_signal_handler(uint8_t interrupt_no, void *ctx)
static void tof_int_signal_handler(uint8_t interrupt_no)
{
  //(void)*ctx;
  // send an event with GPIO to the interrupt user task
  xQueueSendFromISR(tof_evt_queue, &interrupt_no, NULL);
}


int8_t tof_init (void)
{
  uint8_t  byte_data, sensor_state = 0;
  uint16_t word_data;
  //uint8_t  ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum;
  uint8_t  RangeStatus;
  uint8_t  dataReady;
//  int16_t offset;
//  uint16_t xtalk;
  /* MCU Configuration----------------------------------------------------------*/

  CMU_ClockEnable(cmuClock_GPIO, true);

  /** --- INTERRUPT CONFIGURATION PART ---- */

  // Interrupt configuration has to be done before the sensor is set
  // into measurement mode to avoid losing interrupts

  // create an event queue to send interrupt events from interrupt
  // handler to the interrupt task
  tof_evt_queue = xQueueCreate(10, sizeof(uint8_t));

  GPIO_PinModeSet(VL53L1X_INT_PORT, VL53L1X_INT_PIN, gpioModeInput, 0);

  // configure interrupt pins for *INT1* and *INT2* signals and set the interrupt handler
  GPIOINT_Init();
  GPIOINT_CallbackRegister(VL53L1X_INT_PIN, (GPIOINT_IrqCallbackPtr_t) tof_int_signal_handler);
  //acc_interrupt_idx = GPIOINT_CallbackRegisterExt(INT_ACC_PIN,(GPIOINT_IrqCallbackPtrExt_t)int_signal_handler, (void*) NULL);
  //EFM_ASSERT(acc_interrupt_idx != INTERRUPT_UNAVAILABLE);
  GPIO_ExtIntConfig(VL53L1X_INT_PORT, VL53L1X_INT_PIN, VL53L1X_INT_PIN, true, true, true);
  GPIO_IntEnable(1 << VL53L1X_INT_PIN);

  /* Initialize all configured peripherals */

  //status = XNUCLEO53L1A1_ResetId(ToFSensor, 0); // Reset ToF sensor
  //vTaskDelay (2 / portTICK_PERIOD_MS);
//  status = XNUCLEO53L1A1_ResetId(ToFSensor, 1); // Reset ToF sensor
//  Delay(2);

/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, VL53L1_IDENTIFICATION__MODEL_ID, &byte_data);
  if (status) { return (status); }
  app_log_info("VL53L1X Model_ID: %X\n", byte_data);
  status = VL53L1_RdByte(dev, VL53L1_IDENTIFICATION__MODEL_TYPE, &byte_data);
  app_log_info("VL53L1X Module_Type: %X\n", byte_data);
  status = VL53L1_RdWord(dev, VL53L1_IDENTIFICATION__MODEL_ID, &word_data);
  app_log_info("VL53L1X: %X\n", word_data);

  while(sensor_state == 0)
  {
    status = VL53L1X_BootState(dev, &sensor_state);
    vTaskDelay (2 / portTICK_PERIOD_MS);
  }
  app_log_info("Chip booted\n");

  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
  status = VL53L1X_SetInterMeasurementInMs(dev, 100); /* in ms, IM must be > = TB */
//  status = VL53L1X_SetOffset(dev,20); /* offset compensation in mm */
//  status = VL53L1X_SetROI(dev, 16, 16); /* minimum ROI 4,4 */
//  status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
//  status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); /* may take few second to perform the xtalk cal */
  app_log_info("VL53L1X Ultra Lite Driver Example running ...\n");

  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
  while(1)
  {
    /* read and display data */
    while (dataReady == 0)
    {
      status = VL53L1X_CheckForDataReady(dev, &dataReady);
      vTaskDelay (2 / portTICK_PERIOD_MS);
    }
    dataReady = 0;
    status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
    status = VL53L1X_GetDistance(dev, &Distance);
    status = VL53L1X_GetSignalRate(dev, &SignalRate);
    status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
    status = VL53L1X_GetSpadNb(dev, &SpadNum);
    status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
    app_log_info("%u, %u, %u, %u, %u\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
  }

  // create a task that is triggered only in case of interrupts to fetch the data
  xTaskCreate(tof_task_interrupt, "tof_task_interrupt", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  return (status);
}
/** System Clock Configuration **/
