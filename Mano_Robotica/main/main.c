#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "math.h"
#include "servo.h"

#define ESP_CHANNEL 1
#define DEDO_NUM 5
#define SERVO_NUM 7
#define LOW 0
#define HIGH 1

const static char *tag = "main";
static bool flag = 0;

typedef enum
{
    PULGAR = 0,
    INDICE,
    MEDIO,
    ANULAR,
    MENIQUE,
} dedo_t;

typedef struct
{
    uint8_t led;
    int dedo[DEDO_NUM];
    int16_t ax, ay, az;
} packet_t;

packet_t pack;

const servo_t servo_config[SERVO_NUM] = {
    {.gpio = GPIO_NUM_12, .channel = LEDC_CHANNEL_0},
    {.gpio = GPIO_NUM_14, .channel = LEDC_CHANNEL_1},
    {.gpio = GPIO_NUM_27, .channel = LEDC_CHANNEL_2},
    {.gpio = GPIO_NUM_26, .channel = LEDC_CHANNEL_3},
    {.gpio = GPIO_NUM_25, .channel = LEDC_CHANNEL_4},
    {.gpio = GPIO_NUM_33, .channel = LEDC_CHANNEL_5},
};
// GPIO sin usaran y se mantendran en un nivel bajo
static const gpio_num_t unused_gpio[] = {GPIO_NUM_32,
                                         GPIO_NUM_13,
                                         GPIO_NUM_23,
                                         GPIO_NUM_22,
                                         GPIO_NUM_21,
                                         GPIO_NUM_19,
                                         GPIO_NUM_18,
                                         GPIO_NUM_17,
                                         GPIO_NUM_16,
                                         GPIO_NUM_4,};

#define UNUSED_GPIO_COUNT (sizeof(unused_gpio) / sizeof(unused_gpio[0]))

uint8_t ang_y(int16_t ax, int16_t ay, int16_t az);

void GPIO_disable(void);
void wifi_init(void);
void espnow_init(void);
void reg_peer(void);

void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    memcpy(&pack, data, sizeof(pack));
    flag = 1;

}

void send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
}

void app_main(void)
{
    wifi_init();
    espnow_init();
    reg_peer();
    servo_init();

    for (size_t i = 0; i < SERVO_NUM; i++)
    {
        servo_attach(&servo_config[i]);
    }
    
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    uint8_t grado_pulgar = 0, grado_indice = 0, grado_medio = 0, grado_anular = 0, grado_menique = 0, grado_mpu = 0;

    while (1)
    {
        if (flag == 1)
        {
            gpio_set_level(GPIO_NUM_2, pack.led);

            for (servo_id_t servo = SERVO_PULGAR; servo <= SERVO_MPU; servo++)
            {
                switch (servo)
                {
                case SERVO_PULGAR:
                    grado_pulgar = 180 - ((pack.dedo[servo] * 90) / 4095);
                    servo_write(&servo_config[servo], grado_pulgar);
                    break;

                case SERVO_INDICE:
                    grado_indice = 180 - ((pack.dedo[servo] * 180) / 4095);
                    servo_write(&servo_config[servo], grado_indice);
                    break;

                case SERVO_MEDIO:
                    grado_medio = 180 - ((pack.dedo[servo] * 180) / 4095);
                    servo_write(&servo_config[servo], grado_medio);
                    break;

                case SERVO_ANULAR:
                    grado_anular = (pack.dedo[servo] * 180) / 4095;
                    servo_write(&servo_config[servo], grado_anular);
                    break;

                case SERVO_MENIQUE:
                    grado_menique = (pack.dedo[servo] * 180) / 4095;
                    servo_write(&servo_config[servo], grado_menique);
                    break;

                case SERVO_MPU:
                    grado_mpu = ang_y(pack.ax, pack.ay, pack.az);
                    servo_write(&servo_config[servo], ang_y(pack.ax, pack.ay, pack.az));
                    break;
                
                default:
                    break;
                }
            }
            flag = 0;
        }
    }
}

void GPIO_disable(void)
{
    for (size_t i = 0; i < UNUSED_GPIO_COUNT; i++)
    {
        gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << unused_gpio[i]),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));
    ESP_ERROR_CHECK(gpio_set_level(unused_gpio[i], LOW));
    }
}

void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESP_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);
}

void reg_peer(void)
{
    esp_now_peer_info_t info_peer = {0};
    info_peer.channel = ESP_CHANNEL;
    info_peer.ifidx = ESP_IF_WIFI_STA;
    info_peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&info_peer));
}

uint8_t ang_y(int16_t ax, int16_t ay, int16_t az)
{
    float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / M_PI);
    uint8_t ang_y = (uint8_t)(accel_ang_y + 90);
    return ang_y;
}
