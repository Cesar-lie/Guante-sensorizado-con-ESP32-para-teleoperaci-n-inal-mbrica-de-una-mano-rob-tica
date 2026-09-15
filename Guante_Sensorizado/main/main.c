#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "math.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "mpu6050.h"
#include "string.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "driver/gpio.h"

#define ESP_CHANNEL 1

#define ADC_NUM 6
#define DEDO_NUM 5
#define LOW 0
#define HIGH 1

// Configuracion MPU6050
#define GPIO_SDA GPIO_NUM_21
#define GPIO_SCL GPIO_NUM_22

// MASCARA DE BITS
static const gpio_num_t unused_gpio[] = {GPIO_NUM_12,
                                         GPIO_NUM_13, 
                                         GPIO_NUM_14, 
                                         GPIO_NUM_16, 
                                         GPIO_NUM_17, 
                                         GPIO_NUM_18,
                                         GPIO_NUM_19,
                                         GPIO_NUM_23,
                                         GPIO_NUM_25,
                                         GPIO_NUM_26,
                                         GPIO_NUM_27,
                                         GPIO_NUM_33};

#define UNUSED_GPIO_COUNT (sizeof(unused_gpio) / sizeof(unused_gpio[0]))
static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0x78, 0x42, 0x1C, 0x68, 0x3E, 0xE0}; // MAC de la mano robotica
const static char *tag = "main";
adc_oneshot_unit_handle_t adc_handle;

// Paquete de datos que se envia por ESP-NOW
typedef struct
{
    uint8_t led;
    int dedo[DEDO_NUM];
    int16_t ax, ay, az;
} packet_t;

packet_t pack;

typedef enum
{
    PULGAR = 0,
    INDICE,
    MEDIO,
    ANULAR,
    MENIQUE,
} dedo_t;

adc_channel_t adc_channels[ADC_NUM] = {
    ADC_CHANNEL_4,
    ADC_CHANNEL_7,
    ADC_CHANNEL_6,
    ADC_CHANNEL_3,
    ADC_CHANNEL_0,
};

void GPIO_disable(void);
void init_adc(void);
void wifi_init(void);
void espnow_init(void);
void reg_peer(void);

void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
}

void send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    /*if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(tag, "Enviado: %d", status);
    }
    else
    {
        ESP_LOGI(tag, "Error en el envio: %d", status);
    }*/
}

void app_main(void)
{
    wifi_init();
    espnow_init();
    reg_peer();
    GPIO_disable();
    init_adc();
    init_mpu6050(MPU6050_ADDRESS_AD0_LOW, GPIO_SDA, GPIO_SCL);
    

    vTaskDelay(pdMS_TO_TICKS(50));

    // int16_t ax, ay, az;
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    pack.led = 1;

    while (1)
    {
        // grados_mpu_y = ang_y(ax, ay, az);
        for (size_t i = PULGAR; i <= MENIQUE; i++)
        {
            adc_oneshot_read(adc_handle, adc_channels[i], &pack.dedo[i]);
        }
        mpu6050_accel(&pack.ax, &pack.ay, &pack.az);
        pack.led = !pack.led;
        esp_now_send(peer_mac, (uint8_t *)&pack, sizeof(pack));
        gpio_set_level(GPIO_NUM_2, !pack.led);
        esp_now_send(peer_mac, (uint8_t *)&pack, sizeof(pack));

        // ESP_LOGI(tag, "grados del mpu: %u", grados_mpu_y);

        vTaskDelay(pdMS_TO_TICKS(100));
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

void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    for (size_t i = 0; i < ADC_NUM; i++)
    {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, adc_channels[i], &config));
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
    memcpy(info_peer.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    info_peer.channel = ESP_CHANNEL;
    info_peer.ifidx = ESP_IF_WIFI_STA;
    info_peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&info_peer));
}