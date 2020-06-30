
#include "at24c.h"

HAL_StatusTypeDef
AT24C_Write (I2C_HandleTypeDef * hi2c, int address, void *data, size_t length)
{
	HAL_StatusTypeDef error;
	size_t start_address = address;
	size_t start_i = 0;

	for (int i = 0; i < length; i++)
	{
		if ((address + i + 1) % EEPROM_PAGE_SIZE == 0)
		{
			error =
					HAL_I2C_Mem_Write (hi2c, EEPROM_ID, start_address,
							I2C_MEMADD_SIZE_8BIT,
							((uint8_t *) data) + start_i,
							(i + 1) - start_i, 1000);

			if(error != HAL_OK)
				return error;


			start_address = address + i + 1;
			start_i = i + 1;
			HAL_Delay (10);
		}
	}

	if (start_i != length)
	{
		error =
				HAL_I2C_Mem_Write (hi2c, EEPROM_ID, start_address,
						I2C_MEMADD_SIZE_8BIT, ((uint8_t *) data) + start_i,
						length - start_i, 1000);
		HAL_Delay (10);
	}

	return error;
}

HAL_StatusTypeDef
AT24C_Read (I2C_HandleTypeDef * hi2c, int address, void *data, size_t length)
{
	HAL_StatusTypeDef error;

	error =HAL_I2C_Mem_Read
			(hi2c, EEPROM_ID, address, I2C_MEMADD_SIZE_8BIT, (uint8_t *) data,
					length, 1000);

	return error;
}






