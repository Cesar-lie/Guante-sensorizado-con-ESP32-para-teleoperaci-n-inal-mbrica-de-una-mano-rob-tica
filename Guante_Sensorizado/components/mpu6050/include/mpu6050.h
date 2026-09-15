#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#define MPU6050_ADDRESS_AD0_LOW 0x68
#define MPU6050_ADDRESS_AD0_HIGH 0x69

#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0X75
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

#define MPU6050_CONFIG_PW_MGMT_1 0x01
#define MPU6050_DLPF_CFG_44HZ 0x03
#define MPU6050_ACCEL_RANGE_2G 0x00
#define MPU6050_GYRO_RANGE_250DPS 0x00
#define I2C_TIMEOUT_MS 50
#define SCL_SPEED_HZ (100 * 1000)
#define SCL_WAIT_US (50 * 1000)

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

esp_err_t init_mpu6050(uint16_t address, gpio_num_t sda, gpio_num_t scl);

void mpu6050_accel(int16_t*x, int16_t*y, int16_t*z);
void mpu6050_gyro(int16_t*x, int16_t*y, int16_t*z);
void mpu6050_temp(float*tem);