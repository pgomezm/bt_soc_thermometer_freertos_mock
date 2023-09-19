/*
 * rotation_driver.h
 *
 *  Created on: 16 Aug 2023
 *      Author: Pablo Gomez Martino
 */

#ifndef MODULES_LIS2D_MODULE_LIS2D_MODULE_H_
#define MODULES_LIS2D_MODULE_LIS2D_MODULE_H_

#include <stdlib.h>
#include "lis2d.h"

void lis2d_interrupt_handler  (void);
void lis2d_interrupt_task     (void *pvParameters);
void lis2dtw12_orientation    (lis2d_sensor_t *dev);
void lis2dtw12_activity       (lis2d_sensor_t *dev);
void lis2dtw12_wake_up        (lis2d_sensor_t *dev);
void lis2dtw12_read_data_fifo (lis2d_sensor_t *dev);

#endif /* MODULES_ROTATION_DRIVER_ROTATION_DRIVER_H_ */
