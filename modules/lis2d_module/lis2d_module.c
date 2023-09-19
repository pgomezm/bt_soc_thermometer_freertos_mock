
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

#include "lis2d.h"
#include "lis2d_types.h"
#include "lis2d_module.h"
#include "i2c_low_level_driver.h"

/*
 * MORE EXAMPLES ON THIS PAGE
 * https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lis2dtw12_STdC
*/

// LIS3D accelerometer interrupt pin
#define LIS2D_INT_PIN       INT_ACC_PIN
#define LIS2D_INT_PORT      INT_ACC_PORT
#define ROTATION_THRESHOLD  (1)
//#define INT_EVENT

// Global variables
SemaphoreHandle_t  lis2d_interrupt_semaphore;
lis2d_sensor_t    *lis2d_acc_dev;

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
void read_data_lis2d (void)
{
//    #ifdef FIFO_MODE
//
//    lis2d_float_data_fifo_t fifo;
//
//    if (lis2d_new_data (sensor))
//    {
//        uint8_t num = lis2d_get_float_data_fifo (sensor, fifo);
//
//        printf("%.3f lis2d num=%d\n", (double)sdk_system_get_time()*1e-3, num);
//
//        for (int i=0; i < num; i++)
//            // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
//          app_log_info("%.3f lis2d (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
//                   (double)sdk_system_get_time()*1e-3,
//                   fifo[i].ax, fifo[i].ay, fifo[i].az);
//    }
//
//    #else
//
//    lis2d_float_data_t  data;
//
//    if (lis2d_new_data (lis2d_acc_dev) &&
//        lis2d_get_float_data (lis2d_acc_dev, &data))
//      {
//        // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
//        //printf("%.3f lis2d (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
//        app_log_info("lis2d (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
//        //       (double)sdk_system_get_time()*1e-3,
//                data.ax, data.ay, data.az);
//      }
//
//    #endif // FIFO_MODE
}

// User task that fetches the sensor values.

void lis2d_interrupt_task (void *pvParameters)
{
  (void)*pvParameters;
  uint8_t gpio_num;

  while (1)
  {
    if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
    {
//      lis2d_float_data_t  data;
//      lis2d_int_data_source_t  data_src  = {};
//      lis2d_int_event_source_t event_src = {};
//      lis2d_int_click_source_t click_src = {};
//
//      // get the source of the interrupt and reset *INTx* signals
//      #ifdef INT_DATA
//      lis2d_get_int_data_source  (sensor, &data_src);
//      #endif
//      #ifdef INT_EVENT
//      lis2d_get_int_event_source (acc_dev, &event_src, lis2d_int_event1_gen);
//      #endif
//      #ifdef INT_CLICK
//      lis2d_get_int_click_source (sensor, &click_src);
//      #endif
//
//      // in case of DRDY interrupt or inertial event interrupt read one data sample
//      if (data_src.data_ready)
//      {
//        read_data_lis2d ();
//      }
//
//      // in case of FIFO interrupts read the whole FIFO
//      else  if (data_src.fifo_watermark || data_src.fifo_overrun)
//      {
//        read_data_lis2d ();
//      }
//      // in case of event interrupt
//      else if (event_src.active)
//      {
//        //printf("%.3f lis2d ", (double)sdk_system_get_time()*1e-3);
//        if (event_src.x_low)  app_log_info("x is lower than threshold\n");
//        if (event_src.y_low)  app_log_info("y is lower than threshold\n");
//        if (event_src.z_low)  app_log_info("z is lower than threshold\n");
//        if (event_src.x_high) app_log_info("x is higher than threshold\n");
//        if (event_src.y_high) app_log_info("y is higher than threshold\n");
//        if (event_src.z_high) app_log_info("z is higher than threshold\n");
//
//        if (lis2d_new_data (lis2d_acc_dev) &&
//            lis2d_get_float_data (lis2d_acc_dev, &data))
//        {
//          // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
//          //printf("%.3f lis2d (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
//          app_log_info("lis2d (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
//          //       (double)sdk_system_get_time()*1e-3,
//                  data.ax, data.ay, data.az);
//        }
//      }
//      else if (click_src.active)       // in case of click detection interrupt
//      {
//        //app_log_info("%.3f lis2d %s\n", (double)sdk_system_get_time()*1e-3,
//        app_log_info("lis2d %s\n",
//                click_src.s_click ? "single click" : "double click");
//      }
//
//      // Process the rotation direction
//      // current_rotation_dir = detect_rotation_dir (data.ax, data.ay, data.az);
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

// LIS2DTW accelerometer initialisation function
void rotation_lis2d_init (lis2d_sensor_t *dev)
{
  //unsigned int acc_interrupt_idx;
  lis2d_acc_dev = dev;

  CMU_ClockEnable(cmuClock_GPIO, true);

  /** --- INTERRUPT CONFIGURATION PART ---- */

  // Interrupt configuration has to be done before the sensor is set
  // into measurement mode to avoid losing interrupts

  // create an event queue to send interrupt events from interrupt
  // handler to the interrupt task
  gpio_evt_queue = xQueueCreate(10, sizeof(uint8_t));

  GPIO_PinModeSet(LIS2D_INT_PORT, LIS2D_INT_PIN, gpioModeInput, 0);

  // configure interrupt pins for *INT1* and *INT2* signals and set the interrupt handler
  GPIOINT_Init();
  GPIOINT_CallbackRegister(LIS2D_INT_PIN, (GPIOINT_IrqCallbackPtr_t) int_signal_handler);
  //acc_interrupt_idx = GPIOINT_CallbackRegisterExt(INT_ACC_PIN,(GPIOINT_IrqCallbackPtrExt_t)int_signal_handler, (void*) NULL);
  //EFM_ASSERT(acc_interrupt_idx != INTERRUPT_UNAVAILABLE);
  GPIO_ExtIntConfig(LIS2D_INT_PORT, LIS2D_INT_PIN, LIS2D_INT_PIN, true, true, true);
  GPIO_IntEnable(1 << LIS2D_INT_PIN);

  lis2dtw12_orientation (dev);
  lis2dtw12_activity (dev);
  lis2dtw12_wake_up (dev);
  lis2dtw12_read_data_fifo (dev);

  // create a task that is triggered only in case of interrupts to fetch the data
  xTaskCreate(lis2d_interrupt_task, "lis2d_interrupt_task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  return;
}

/* Main Example --------------------------------------------------------------*/
void lis2dtw12_orientation (lis2d_sensor_t *dev)
{
  static uint8_t tx_buffer[1000];
  lis2dtw12_reg_t int_route;

  /* Set full scale */
  lis2dtw12_full_scale_set(dev, LIS2DTW12_2g);
  /* Configure power mode */
  lis2dtw12_power_mode_set(dev, LIS2DTW12_CONT_LOW_PWR_LOW_NOISE_12bit);
  /* Set threshold to 60 degrees */
  lis2dtw12_6d_threshold_set(dev, 0x02);
  /* LPF2 on 6D function selection. */
  lis2dtw12_6d_feed_data_set(dev, LIS2DTW12_ODR_DIV_2_FEED);
  /* Enable interrupt generation on 6D INT1 pin. */
  lis2dtw12_pin_int1_route_get(dev, &int_route.ctrl4_int1_pad_ctrl);
  int_route.ctrl4_int1_pad_ctrl.int1_6d = PROPERTY_ENABLE;
  lis2dtw12_pin_int1_route_set(dev, &int_route.ctrl4_int1_pad_ctrl);
  /* Set Output Data Rate */
  lis2dtw12_data_rate_set(dev, LIS2DTW12_XL_ODR_200Hz);

  /* Wait Events. */
  while (1)
  {
    lis2dtw12_all_sources_t all_source;
    lis2dtw12_all_sources_get(dev, &all_source);

    /* Check 6D Orientation events */
    if (all_source.sixd_src._6d_ia) {
      sprintf((char *)tx_buffer, "6D Or. switched to ");

      if (all_source.sixd_src.xh) {
        strcat((char *)tx_buffer, "XH");
      }

      if (all_source.sixd_src.xl) {
        strcat((char *)tx_buffer, "XL");
      }

      if (all_source.sixd_src.yh) {
        strcat((char *)tx_buffer, "YH");
      }

      if (all_source.sixd_src.yl) {
        strcat((char *)tx_buffer, "YL");
      }

      if (all_source.sixd_src.zh) {
        strcat((char *)tx_buffer, "ZH");
      }

      if (all_source.sixd_src.zl) {
        strcat((char *)tx_buffer, "ZL");
      }

      app_log_info("%s\r\n", tx_buffer);
    }
  }
}

void lis2dtw12_activity (lis2d_sensor_t *dev)
{
  /* Initialize mems driver interface */
  lis2dtw12_reg_t int_route;
  static uint8_t tx_buffer[1000];

  /* Set full scale */
  lis2dtw12_full_scale_set(dev, LIS2DTW12_2g);
  /* Configure filtering chain
   * Accelerometer - filter path / bandwidth */
  lis2dtw12_filter_path_set(dev, LIS2DTW12_LPF_ON_OUT);
  lis2dtw12_filter_bandwidth_set(dev, LIS2DTW12_ODR_DIV_4);
  /* Configure power mode */
  lis2dtw12_power_mode_set(dev,
                           LIS2DTW12_CONT_LOW_PWR_LOW_NOISE_12bit);
  /* Set wake-up duration
   * Wake up duration event 1LSb = 1 / ODR
   */
  lis2dtw12_wkup_dur_set(dev, 2);
  /* Set sleep duration
   * Duration to go in sleep mode (1 LSb = 512 / ODR)
   */
  lis2dtw12_act_sleep_dur_set(dev, 2);
  /* Set Activity wake-up threshold
   * Threshold for wake-up 1 LSB = FS_XL / 64
   */
  lis2dtw12_wkup_threshold_set(dev, 2);
  /* Data sent to wake-up interrupt function */
  lis2dtw12_wkup_feed_data_set(dev, LIS2DTW12_HP_FEED);
  /* Config activity / inactivity or stationary / motion detection */
  lis2dtw12_act_mode_set(dev, LIS2DTW12_DETECT_ACT_INACT);
  /* Enable activity detection interrupt */
  lis2dtw12_pin_int1_route_get(dev, &int_route.ctrl4_int1_pad_ctrl);
  int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_ENABLE;
  lis2dtw12_pin_int1_route_set(dev, &int_route.ctrl4_int1_pad_ctrl);
  /* Set Output Data Rate */
  lis2dtw12_data_rate_set(dev, LIS2DTW12_XL_ODR_200Hz);

  /* Wait Events */
  while (1)
  {
    lis2dtw12_all_sources_t all_source;
    /* Read status register */
    lis2dtw12_all_sources_get(dev, &all_source);

    /* Check if Activity/Inactivity events */
    if (all_source.wake_up_src.sleep_state_ia) {
      sprintf((char *)tx_buffer, "Inactivity Detected\r\n");
      app_log_info("%s\r\n", tx_buffer);
    }

    if (all_source.wake_up_src.wu_ia) {
      sprintf((char *)tx_buffer, "Activity Detected\r\n");
      app_log_info("%s\r\n", tx_buffer);
    }
  }
}

void lis2dtw12_wake_up (lis2d_sensor_t *dev)
{
  /* Initialize mems driver interface */
  lis2dtw12_reg_t int_route;
  static uint8_t tx_buffer[1000];

  /* Set full scale */
  lis2dtw12_full_scale_set(dev, LIS2DTW12_2g);
  /* Configure power mode */
  lis2dtw12_power_mode_set(dev,
                           LIS2DTW12_CONT_LOW_PWR_LOW_NOISE_12bit);
  /* Set Output Data Rate */
  lis2dtw12_data_rate_set(dev, LIS2DTW12_XL_ODR_200Hz);
  /* Apply high-pass digital filter on Wake-Up function */
  lis2dtw12_filter_path_set(dev, LIS2DTW12_HIGH_PASS_ON_OUT);
  /* Apply high-pass digital filter on Wake-Up function
   * Duration time is set to zero so Wake-Up interrupt signal
   * is generated for each X,Y,Z filtered data exceeding the
   * configured threshold */
  lis2dtw12_wkup_dur_set(dev, 0);
  /* Set wake-up threshold
   *
   * Set Wake-Up threshold: 1 LSb corresponds to FS_XL/2^6 */
  lis2dtw12_wkup_threshold_set(dev, 2);
  /* Enable interrupt generation on Wake-Up INT1 pin
   * */
  lis2dtw12_pin_int1_route_get(dev,
                               &int_route.ctrl4_int1_pad_ctrl);
  int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_ENABLE;
  lis2dtw12_pin_int1_route_set(dev,
                               &int_route.ctrl4_int1_pad_ctrl);

  /* Wait Events */
  while (1) {
    lis2dtw12_all_sources_t all_source;
    /*
     * Check Wake-Up events
     */
    lis2dtw12_all_sources_get(dev, &all_source);

    if (all_source.wake_up_src.wu_ia) {
      sprintf((char *)tx_buffer, "Wake-Up event on ");

      if (all_source.wake_up_src.x_wu) {
        strcat((char *)tx_buffer, "X");
      }

      if (all_source.wake_up_src.y_wu) {
        strcat((char *)tx_buffer, "Y");
      }

      if (all_source.wake_up_src.z_wu) {
        strcat((char *)tx_buffer, "Z");
      }

      strcat((char *)tx_buffer, " direction\r\n");
      app_log_info("%s\r\n", tx_buffer);
    }
  }
}

void lis2dtw12_read_data_fifo (lis2d_sensor_t *dev)
{
  /* Initialize mems driver interface */
  static int16_t data_raw_acceleration[3];
  static float acceleration_mg[3];

  /* Enable Block Data Update */
  lis2dtw12_block_data_update_set(dev, PROPERTY_ENABLE);
  /* Set full scale */
  //lis2dtw12_full_scale_set(dev, LIS2DTW12_8g);
  lis2dtw12_full_scale_set(dev, LIS2DTW12_2g);
  /* Configure filtering chain
   * Accelerometer - filter path / bandwidth
   */
  lis2dtw12_filter_path_set(dev, LIS2DTW12_LPF_ON_OUT);
  lis2dtw12_filter_bandwidth_set(dev, LIS2DTW12_ODR_DIV_4);
  /* Configure FIFO */
  lis2dtw12_fifo_watermark_set(dev, 10);
  lis2dtw12_fifo_mode_set(dev, LIS2DTW12_STREAM_MODE);
  /* Configure power mode */
  lis2dtw12_power_mode_set(dev, LIS2DTW12_HIGH_PERFORMANCE);
  //lis2dtw12_power_mode_set(dev, LIS2DTW12_CONT_LOW_PWR_LOW_NOISE_12bit);
  /* Set Output Data Rate */
  lis2dtw12_data_rate_set(dev, LIS2DTW12_XL_ODR_25Hz);

  /* Read samples in polling mode (no int) */
  while (1)
  {
    uint8_t val, i;
    /* Read output only if new value is available */
    lis2dtw12_fifo_wtm_flag_get(dev, &val);

    if (val)
    {
      lis2dtw12_fifo_data_level_get(dev, &val);

      for (i = 0; i < val; i++)
      {
        /* Read acceleration data */
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        lis2dtw12_acceleration_raw_get(dev, data_raw_acceleration);
        //acceleration_mg[0] = lis2dtw12_from_fs8_lp1_to_mg(data_raw_acceleration[0]);
        //acceleration_mg[1] = lis2dtw12_from_fs8_lp1_to_mg(data_raw_acceleration[1]);
        //acceleration_mg[2] = lis2dtw12_from_fs8_lp1_to_mg(data_raw_acceleration[2]);
        acceleration_mg[0] = lis2dtw12_from_fs2_to_mg(  data_raw_acceleration[0]);
        acceleration_mg[1] = lis2dtw12_from_fs2_to_mg(data_raw_acceleration[1]);
        acceleration_mg[2] = lis2dtw12_from_fs2_to_mg(data_raw_acceleration[2]);
        app_log_info("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      }
    }
  }
}


