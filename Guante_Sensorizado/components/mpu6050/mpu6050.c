#include <stdio.h>
#include "mpu6050.h"

const static char *tag = "IMU";

i2c_master_bus_handle_t i2c_handle;
i2c_master_dev_handle_t mpu_handle;

static int16_t gyro_x_offset = -2315;
static int16_t gyro_y_offset = 100;
static int16_t accelx_offset = 750;
static int16_t accely_offset = -400;

static esp_err_t mpu6050_register(uint8_t reg_addr, uint8_t data){
    uint8_t buff[2] = {reg_addr, data};
    return i2c_master_transmit(mpu_handle, buff, sizeof(buff), I2C_TIMEOUT_MS);
}

static esp_err_t mpu6050_wake_up(void);

esp_err_t init_mpu6050(uint16_t address, gpio_num_t sda, gpio_num_t scl)
{
    i2c_master_bus_config_t bus_congif = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };

    
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_congif, &i2c_handle));

    i2c_device_config_t device_config ={
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = SCL_SPEED_HZ,
        .scl_wait_us = SCL_WAIT_US,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle,&device_config,&mpu_handle));

    ESP_ERROR_CHECK(mpu6050_wake_up());

    ESP_ERROR_CHECK((mpu6050_register(CONFIG, MPU6050_DLPF_CFG_44HZ)));
    ESP_ERROR_CHECK(mpu6050_register(ACCEL_CONFIG, MPU6050_ACCEL_RANGE_2G));
    ESP_ERROR_CHECK(mpu6050_register(GYRO_CONFIG, MPU6050_GYRO_RANGE_250DPS));

    return ESP_OK;
}

static esp_err_t mpu6050_wake_up(void)
{
    ESP_ERROR_CHECK(mpu6050_register(PWR_MGMT_1, MPU6050_CONFIG_PW_MGMT_1));

    uint8_t data = 0;
    uint8_t reg = WHO_AM_I;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu_handle, &reg, 1, &data, 1,I2C_TIMEOUT_MS));
    ESP_LOGI(tag, "WHO_AM_I = 0x%02X", data);

    return ESP_OK;
}

void mpu6050_accel(int16_t*x, int16_t*y, int16_t*z){
    uint8_t reg = ACCEL_XOUT_H;
    uint8_t accel_data[6];
    i2c_master_transmit_receive(mpu_handle, &reg, 1, accel_data, sizeof(accel_data), I2C_TIMEOUT_MS);

    *x = ((accel_data[0] << 8) | accel_data[1]) - accelx_offset;
    *y = ((accel_data[2] << 8) | accel_data[3]) - accely_offset;
    *z = ((accel_data[4] << 8) | accel_data[5]);
}

void mpu6050_gyro(int16_t*x, int16_t*y, int16_t*z){
    uint8_t reg = GYRO_XOUT_H;
    uint8_t gyro_data[6];
    i2c_master_transmit_receive(mpu_handle, &reg, 1, gyro_data, sizeof(gyro_data), I2C_TIMEOUT_MS);

    *x = ((gyro_data[0] << 8) | gyro_data[1]) - gyro_x_offset;
    *y = ((gyro_data[2] << 8) | gyro_data[3]) - gyro_y_offset;
    *z = ((gyro_data[4] << 8) | gyro_data[5]);
}

void mpu6050_temp(float*temp){
    uint8_t reg = TEMP_OUT_H;
    uint8_t temp_data[2];
    int16_t temp_raw = 0;
    i2c_master_transmit_receive(mpu_handle, &reg, 1, temp_data, sizeof(temp_data), I2C_TIMEOUT_MS);
    temp_raw = ((temp_data[0] << 8) | temp_data[1]);
    *temp = temp_raw/340.0 + 36.53;
}