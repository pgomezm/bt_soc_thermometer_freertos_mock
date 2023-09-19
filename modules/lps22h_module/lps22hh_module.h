/*
 * rotation_driver.h
 *
 *  Created on: 16 Aug 2023
 *      Author: Pablo Gomez Martino
 */

#ifndef MODULES_LIS2D_MODULE_LIS2D_MODULE_H_
#define MODULES_LIS2D_MODULE_LIS2D_MODULE_H_

#include <stdlib.h>
#include "lis2mdl.h"

void lis2mdl_interrupt_handler  (void);
void lis2mdl_interrupt_task     (void *pvParameters);

#endif /* MODULES_ROTATION_DRIVER_ROTATION_DRIVER_H_ */
