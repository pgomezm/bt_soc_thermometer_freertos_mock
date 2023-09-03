/*
 * rotation_driver.h
 *
 *  Created on: 16 Aug 2023
 *      Author: Pablo Gomez Martino
 */

#ifndef MODULES_ROTATION_DRIVER_ROTATION_DRIVER_H_
#define MODULES_ROTATION_DRIVER_ROTATION_DRIVER_H_

#include <stdlib.h>
#include "lis3dh.h"

// Rotation direction enumeration
typedef enum
{
  ROTATION_NONE,
  ROTATION_CLOCKWISE,
  ROTATION_COUNTERCLOCKWISE
}rotation_direction;

void               rotation_init           (lis3dh_sensor_t *acc_dev);
void               lis3d_interrupt_handler (void);
void               lis3d_interrupt_task    (void *pvParameters);
rotation_direction detect_rotation_dir     (int16_t x, int16_t y, int16_t z);

#endif /* MODULES_ROTATION_DRIVER_ROTATION_DRIVER_H_ */
