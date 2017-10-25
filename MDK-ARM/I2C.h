
#include "stm32f4xx_hal.h"

typedef enum _I2C_Result_t {
    I2C_Result_Ok = 0x00,
    I2C_Result_Error,    
} I2C_Result_t;

I2C_Result_t I2C_Write(I2C_HandleTypeDef* hi2c, uint8_t device_address, uint8_t register_address, uint8_t data);
I2C_Result_t I2C_ReadMulti(I2C_HandleTypeDef* hi2c, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);