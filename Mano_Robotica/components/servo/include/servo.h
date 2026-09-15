#include "esp_err.h"
#include "driver/ledc.h"

typedef struct
{
    gpio_num_t gpio;
    ledc_channel_t channel;
} servo_t;

typedef enum
{
    SERVO_PULGAR = 0,
    SERVO_INDICE,
    SERVO_MEDIO,
    SERVO_ANULAR,
    SERVO_MENIQUE,
    SERVO_MPU,
}servo_id_t;

esp_err_t servo_init(void);
esp_err_t servo_attach(const servo_t *servo);
esp_err_t servo_write(const servo_t *servo, uint8_t degrees);