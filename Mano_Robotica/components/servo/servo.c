#include <stdio.h>
#include "servo.h"

#define SERVO_NUM 3
#define SERVO_MIN_DUTY 27
#define SERVO_MAX_DUTY 128
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

esp_err_t servo_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    return ESP_OK;
}

esp_err_t servo_attach(const servo_t *servo)
{
    ledc_channel_config_t channel_config = {
        .gpio_num = servo->gpio,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = servo->channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    return ESP_OK;
}

esp_err_t servo_write(const servo_t *servo, uint8_t degrees)
{
    degrees = (degrees > SERVO_MAX_ANGLE) ? SERVO_MAX_ANGLE :degrees;
    degrees = (degrees < SERVO_MIN_ANGLE) ? SERVO_MIN_ANGLE : degrees;
    
    int duty = SERVO_MIN_DUTY + ((degrees * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)) / SERVO_MAX_ANGLE);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, servo->channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, servo->channel);
    
    return ESP_OK;
}
