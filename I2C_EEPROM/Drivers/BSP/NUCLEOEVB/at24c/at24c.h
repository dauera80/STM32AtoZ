/*
 * at24_hal_i2c.h
 *
 *  Created on: Sep,11,2015
 *      Author: Sina Darvishi
 */

#ifndef __AT24C_H_
#define __AT24C_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f1xx_hal.h"

#define EEPROM_ID         0x50<<1
#define EEPROM_PAGE_SIZE  16

HAL_StatusTypeDef AT24C_Write (I2C_HandleTypeDef * hi2c, int address, void *data, size_t length);
HAL_StatusTypeDef AT24C_Read (I2C_HandleTypeDef * hi2c, int address, void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* __AT24_H_ */
