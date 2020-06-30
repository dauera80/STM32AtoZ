#ifndef __HDC1080_H
#define __HDC1080_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f1xx_hal.h"

#define HDC1080_ADDR 		    	0x40

/* Register addresses */
#define HDC1080_TEMPERATURE		0x00
#define HDC1080_HUMIDITY 	    0x01
#define HDC1080_CONFIG		  	0x02
#define HDC1080_SERIAL_ID1		0xFB
#define HDC1080_SERIAL_ID2		0xFC
#define HDC1080_SERIAL_ID3		0xFD
#define HDC1080_ID_MANU		  	0xFE
#define HDC1080_ID_DEV		  	0xFF

#define HDC1080_RH_RES_14	  	0x00
#define HDC1080_RH_RES_11	  	0x01
#define HDC1080_RH_RES8		  	0x02

#define HDC1080_T_RES_14	  	0x00
#define HDC1080_T_RES_11	  	0x01

HAL_StatusTypeDef hdc1080_read_reg (I2C_HandleTypeDef * hi2c, uint8_t reg, uint16_t * val);
HAL_StatusTypeDef hdc1080_write_reg (I2C_HandleTypeDef * hi2c, uint8_t reg, uint16_t val);
HAL_StatusTypeDef hdc1080_init (I2C_HandleTypeDef * hi2c, uint8_t temp_res, uint8_t humid_res, uint8_t heater, uint8_t * bat_stat);
HAL_StatusTypeDef hdc1080_measure (I2C_HandleTypeDef * hi2c, float *temperature, uint8_t * humidity);

#ifdef __cplusplus
}
#endif

#endif
