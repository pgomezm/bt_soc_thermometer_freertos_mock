
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

#include "lis3dh.h"
#include "lis3dh_types.h"
#include "rotation_driver.h"
#include "i2c_low_level_driver.h"

// LIS3D accelerometer interrupt pin
#define LIS3D_INT_PIN       INT_ACC_PIN
#define LIS3D_INT_PORT      INT_ACC_PORT
#define ROTATION_THRESHOLD  (1)
#define INT_EVENT

// Global variables
SemaphoreHandle_t  lis3d_interrupt_semaphore;
rotation_direction current_rotation_dir;
lis3dh_sensor_t    *acc_dev;

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
void read_data_lis3dh (void)
{
    #ifdef FIFO_MODE

    lis3dh_float_data_fifo_t fifo;

    if (lis3dh_new_data (sensor))
    {
        uint8_t num = lis3dh_get_float_data_fifo (sensor, fifo);

        printf("%.3f LIS3DH num=%d\n", (double)sdk_system_get_time()*1e-3, num);

        for (int i=0; i < num; i++)
            // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
          app_log_info("%.3f LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
                   (double)sdk_system_get_time()*1e-3,
                   fifo[i].ax, fifo[i].ay, fifo[i].az);
    }

    #else

    lis3dh_float_data_t  data;

    if (lis3dh_new_data (acc_dev) &&
        lis3dh_get_float_data (acc_dev, &data))
      {
        // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
        //printf("%.3f LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
        app_log_info("LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
        //       (double)sdk_system_get_time()*1e-3,
                data.ax, data.ay, data.az);
      }

    #endif // FIFO_MODE
}

// User task that fetches the sensor values.

void lis3d_interrupt_task (void *pvParameters)
{
  (void)*pvParameters;
  uint8_t gpio_num;

  while (1)
  {
    if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
    {
      lis3dh_float_data_t  data;
      lis3dh_int_data_source_t  data_src  = {};
      lis3dh_int_event_source_t event_src = {};
      lis3dh_int_click_source_t click_src = {};

      // get the source of the interrupt and reset *INTx* signals
      #ifdef INT_DATA
      lis3dh_get_int_data_source  (sensor, &data_src);
      #endif
      #ifdef INT_EVENT
      lis3dh_get_int_event_source (acc_dev, &event_src, lis3dh_int_event1_gen);
      #endif
      #ifdef INT_CLICK
      lis3dh_get_int_click_source (sensor, &click_src);
      #endif

      // in case of DRDY interrupt or inertial event interrupt read one data sample
      if (data_src.data_ready)
      {
        read_data_lis3dh ();
      }

      // in case of FIFO interrupts read the whole FIFO
      else  if (data_src.fifo_watermark || data_src.fifo_overrun)
      {
        read_data_lis3dh ();
      }
      // in case of event interrupt
      else if (event_src.active)
      {
        //printf("%.3f LIS3DH ", (double)sdk_system_get_time()*1e-3);
        if (event_src.x_low)  app_log_info("x is lower than threshold\n");
        if (event_src.y_low)  app_log_info("y is lower than threshold\n");
        if (event_src.z_low)  app_log_info("z is lower than threshold\n");
        if (event_src.x_high) app_log_info("x is higher than threshold\n");
        if (event_src.y_high) app_log_info("y is higher than threshold\n");
        if (event_src.z_high) app_log_info("z is higher than threshold\n");

        if (lis3dh_new_data (acc_dev) &&
            lis3dh_get_float_data (acc_dev, &data))
        {
          // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
          //printf("%.3f LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
          app_log_info("LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
          //       (double)sdk_system_get_time()*1e-3,
                  data.ax, data.ay, data.az);
        }
      }
      else if (click_src.active)       // in case of click detection interrupt
      {
        //app_log_info("%.3f LIS3DH %s\n", (double)sdk_system_get_time()*1e-3,
        app_log_info("LIS3DH %s\n",
                click_src.s_click ? "single click" : "double click");
      }

      // Process the rotation direction
      // current_rotation_dir = detect_rotation_dir (data.ax, data.ay, data.az);
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

// LIS3D accelerometer initialization function
void rotation_init (lis3dh_sensor_t *dev)
{
  lis3dh_int_event_config_t event_config;
  //unsigned int acc_interrupt_idx;
  acc_dev = dev;

  CMU_ClockEnable(cmuClock_GPIO, true);

  /** --- INTERRUPT CONFIGURATION PART ---- */

  // Interrupt configuration has to be done before the sensor is set
  // into measurement mode to avoid losing interrupts

  // create an event queue to send interrupt events from interrupt
  // handler to the interrupt task
  gpio_evt_queue = xQueueCreate(10, sizeof(uint8_t));

  GPIO_PinModeSet(INT_ACC_PORT, INT_ACC_PIN, gpioModeInput, 0);

  // configure interrupt pins for *INT1* and *INT2* signals and set the interrupt handler
  GPIOINT_Init();
  GPIOINT_CallbackRegister(INT_ACC_PIN, (GPIOINT_IrqCallbackPtr_t) int_signal_handler);
  //acc_interrupt_idx = GPIOINT_CallbackRegisterExt(INT_ACC_PIN,(GPIOINT_IrqCallbackPtrExt_t)int_signal_handler, (void*) NULL);
  //EFM_ASSERT(acc_interrupt_idx != INTERRUPT_UNAVAILABLE);
  GPIO_ExtIntConfig(INT_ACC_PORT, INT_ACC_PIN, INT_ACC_PIN, true, true, true);
  GPIO_IntEnable(1 << INT_ACC_PIN);

  // set polarity of INT signals if necessary
  lis3dh_config_int_signals (acc_dev, lis3dh_high_active); //crtl6

  //event_config.mode = lis3dh_wake_up;
  // event_config.mode = lis3dh_free_fall;
  event_config.mode = lis3dh_6d_movement;
  // event_config.mode = lis3dh_6d_position;
  //event_config.mode = lis3dh_4d_movement;
  // event_config.mode = lis3dh_4d_position;
  event_config.threshold = 0x8;
  event_config.x_low_enabled  = true;
  event_config.x_high_enabled = true;
  event_config.y_low_enabled  = true;
  event_config.y_high_enabled = true;
  event_config.z_low_enabled  = true;
  event_config.z_high_enabled = true;
  event_config.duration = 0;
  event_config.latch = true;

  lis3dh_set_int_event_config (acc_dev, &event_config, lis3dh_int_event1_gen);
  lis3dh_enable_int (acc_dev, lis3dh_int_event1, lis3dh_int1_signal, true);

  // configure HPF and reset the reference by dummy read
  lis3dh_config_hpf (acc_dev, lis3dh_hpf_normal, 0, true, false, true, false);
  lis3dh_get_hpf_ref (acc_dev);

  // enable ADC inputs and temperature sensor for ADC input 3
  lis3dh_enable_adc (acc_dev, true, true);
  uint16_t temp;
  lis3dh_get_adc(acc_dev, NULL, NULL, &temp);
  app_log_info("LIS3DH temperature %d C\n", temp);

  // LAST STEP: Finally set scale and mode to start measurements
  lis3dh_set_scale(acc_dev, lis3dh_scale_2_g);
  lis3dh_set_mode (acc_dev, lis3dh_odr_100, lis3dh_high_res, true, true, true);

  // create a task that is triggered only in case of interrupts to fetch the data
  xTaskCreate(lis3d_interrupt_task, "user_task_interrupt", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  return;
}

// Function to detect rotation direction based on accelerometer data
rotation_direction detect_rotation_dir (int16_t x, int16_t y, int16_t z)
{
  static int16_t prevX = 0;
  static int16_t prevY = 0;
  static int16_t prevZ = 0;

  int16_t deltaX = x - prevX;
  int16_t deltaY = y - prevY;
  int16_t deltaZ = z - prevZ;

  // Update previous readings
  prevX = x;
  prevY = y;
  prevZ = z;

  // Calculate the magnitude of the deltas
  uint32_t deltaMagnitude = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;

  // Detect rotation direction based on the magnitude of the deltas
  if (deltaMagnitude > ROTATION_THRESHOLD)
  {
    if (deltaY > 0)
    {
      return ROTATION_CLOCKWISE;
    }
    else if (deltaY < 0)
    {
      return ROTATION_COUNTERCLOCKWISE;
    }
  }

  return ROTATION_NONE;
}
