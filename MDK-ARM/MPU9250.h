
#include "stm32f4xx_hal.h"

#define MPU9250_I2C_ADDR        0xD0  
#define MPU9250_I2C_ADDR_MAG    (0x0C << 1)

typedef enum _MPU9250_Result_t {
     MPU9250_Result_Ok = 0x00,
     MPU9250_Result_Error,
     MPU9250_Result_DeviceNotConnected
} MPU9250_Result_t;

typedef enum _MPU9250_Device_t {
     MPU9250_Device_0 = 0x00,
     MPU9250_Device_1 = 0x02
} MPU9250_Device_t;

typedef enum _MPU9250_AcceSens_t {
     MPU9250_AcceSens_2G = 0x00,
     MPU9250_AcceSens_4G = 0x01,
     MPU9250_AcceSens_8G = 0x02,
     MPU9250_AcceSens_16G = 0x03
} MPU9250_AcceSens_t;

typedef enum _MPU9250_GyroSens_t {
     MPU9250_GyroSens_250DPS = 0x00,
     MPU9250_GyroSens_500DPS = 0x01,
     MPU9250_GyroSens_1000DPS = 0x02,
     MPU9250_GyroSens_2000DPS = 0x03
} MPU9250_GyroSens_t;

typedef enum _MPU9250_MagSens_t {
     MPU9250_MagSens_14Bit = 0x00,    // 0.6 mG per LSB
     MPU9250_MagSens_16Bit            // 0.15 mG per LSB
} MPU9250_MagSens_t;

typedef struct {
    float Ax, Ay, Az;         /*!< Accelerometer raw data */
    float Gx, Gy, Gz;         /*!< Gyroscope raw data */
    float Mx, My, Mz;         /*!< Magnetometer raw data */
    int16_t Ax_Raw, Ay_Raw, Az_Raw;         /*!< Accelerometer raw data */
    int16_t Gx_Raw, Gy_Raw, Gz_Raw;         /*!< Gyroscope raw data */
    int16_t Mx_Raw, My_Raw, Mz_Raw;         /*!< Magnetometer raw data */
	
   
    float AMult, GMult, MMult;
    float magCalibrationX, magCalibrationY, magCalibrationZ;
		float mx, my, mz;
		float	magbias[3];
		float accele_local[3];
		float gyro_local[3];
		float filtered_ax, filtered_ay, filtered_az;
		float filtered_gx, filtered_gy, filtered_gz;
		float filtered_mx, filtered_my, filtered_mz;
		float	magscale[3];
		float	mag_avg_rad;
	
    uint8_t I2C_Addr;
    uint8_t I2C_Addr_Mag;
} MPU9250_t;


MPU9250_Result_t MPU9250_Init(I2C_HandleTypeDef* hi2c, MPU9250_t* MPU9250,  MPU9250_Device_t dev);
MPU9250_Result_t MPU9250_ReadAcce(I2C_HandleTypeDef* hi2c, MPU9250_t* MPU9250);
MPU9250_Result_t MPU9250_ReadGyro(I2C_HandleTypeDef* hi2c, MPU9250_t* MPU9250);
MPU9250_Result_t MPU9250_ReadMag(I2C_HandleTypeDef* hi2c, MPU9250_t* MPU9250);
MPU9250_Result_t MPU9250_DataReady(I2C_HandleTypeDef* hi2c, MPU9250_t* MPU9250);

