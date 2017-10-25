
#include "I2C.h"
#include "stm32f4xx_hal.h"

I2C_Result_t I2C_Write(I2C_HandleTypeDef* hi2c, uint8_t device_address, uint8_t register_address, uint8_t data) {
	uint8_t d[2];
		
	/* Format array to send */
	d[0] = register_address;
	d[1] = data;
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)device_address, (uint8_t *)d, 2, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return I2C_Result_Error;
	} 
	
	/* Return OK */
	return I2C_Result_Ok;
}

I2C_Result_t I2C_ReadMulti(I2C_HandleTypeDef* hi2c, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count) {
		
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return I2C_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(hi2c, device_address, data, count, 1000) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return I2C_Result_Error;
	}
	
	/* Return OK */
	return I2C_Result_Ok;

}