
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "sl_assert.h"
#include "sl_i2cspm.h"
#include "sl_udelay.h"
#include "app_log.h"

#include "queue.h"
#include "semphr.h"
#include "portmacro.h"
#include "projdefs.h"
#include "sl_i2cspm_devices_config.h"
#include "pin_config.h"
#include "gpiointerrupt.h"

#include "lis2mdl.h"
#include "lis2mdl_types.h"
#include "lis2mdl_module.h"
#include "i2c_low_level_driver.h"

/*
 * MORE EXAMPLES ON THIS PAGE
 * https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lis2mdl_STdC
*/

// LIS2MDL magnetometer
#define LIS2MDL_INT_PIN       INT_MG_PIN
#define LIS2MDL_INT_PORT      INT_MG_PORT
//#define INT_EVENT

// Global variables
SemaphoreHandle_t  lis2mdl_interrupt_semaphore;
lis2mdl_sensor_t   *lis2mdl_mg_dev;

/**
 * In this case, any of the possible interrupts on interrupt signal *INT1* is
 * used to fetch the data.
 *
 * When interrupts are used, the user has to define interrupt handlers that
 * either fetches the data directly or triggers a task which is waiting to
 * fetch the data. In this example, the interrupt handler sends an event to
 * a waiting task to trigger the data gathering.
 */

static QueueHandle_t gpio_evt_queue = NULL;

/**
 * Common function used to get sensor data.
 */
void read_data_lis2mdl (void)
{

}

// User task that fetches the sensor values.

void lis2mdl_interrupt_task (void *pvParameters)
{
  (void)*pvParameters;
  uint8_t gpio_num;

  while (1)
  {
    if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
    {

    }
  }
}

// Interrupt handler which resumes user_task_interrupt on interrupt
//static void int_signal_handler(uint8_t interrupt_no, void *ctx)
static void int_signal_handler(uint8_t interrupt_no)
{
  //(void)*ctx;
  // send an event with GPIO to the interrupt user task
  xQueueSendFromISR(gpio_evt_queue, &interrupt_no, NULL);
}

// LIS2MDL magnetometer initialisation function
void lis2mdl_init (lis2mdl_sensor_t *dev)
{
  //unsigned int acc_interrupt_idx;
  lis2mdl_mg_dev = dev;

  CMU_ClockEnable(cmuClock_GPIO, true);

  /** --- INTERRUPT CONFIGURATION PART ---- */

  // Interrupt configuration has to be done before the sensor is set
  // into measurement mode to avoid losing interrupts

  // create an event queue to send interrupt events from interrupt
  // handler to the interrupt task
  gpio_evt_queue = xQueueCreate(10, sizeof(uint8_t));

  GPIO_PinModeSet(LIS2MDL_INT_PORT, LIS2MDL_INT_PIN, gpioModeInput, 0);

  // configure interrupt pins for *INT1* and *INT2* signals and set the interrupt handler
  GPIOINT_Init();
  GPIOINT_CallbackRegister(LIS2MDL_INT_PIN, (GPIOINT_IrqCallbackPtr_t) int_signal_handler);
  //acc_interrupt_idx = GPIOINT_CallbackRegisterExt(INT_ACC_PIN,(GPIOINT_IrqCallbackPtrExt_t)int_signal_handler, (void*) NULL);
  //EFM_ASSERT(acc_interrupt_idx != INTERRUPT_UNAVAILABLE);
  GPIO_ExtIntConfig(LIS2MDL_INT_PORT, LIS2MDL_INT_PIN, LIS2MDL_INT_PIN, true, true, true);
  GPIO_IntEnable(1 << LIS2MDL_INT_PIN);

  // create a task that is triggered only in case of interrupts to fetch the data
  xTaskCreate(lis2mdl_interrupt_task, "lis2mgl_interrupt_task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  return;
}



