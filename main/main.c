#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "nvs_flash.h"

enum{
ESPNOW_RX,
ESPNOW_TX,
};

#define CONFIG_ESPNOW_PMK "pmk1234567890123"
#define CONFIG_ESPNOW_LMK "lmk1234567890123"

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static QueueHandle_t espnow_queue = NULL;

typedef struct {
    uint8_t State;
    uint8_t * Len;
    uint8_t * Buffer;
    uint8_t addr[ESP_NOW_ETH_ALEN];
} EspnowType;

EspnowType _Espnow;

static const char *TAG = "MyEspnow";
#define BLINK1_GPIO GPIO_NUM_12
#define BLINK2_GPIO GPIO_NUM_13
#define BUTTER_GPIO GPIO_NUM_9

TaskHandle_t BlinkTaskHandle;

static void BlinkTask(void *pvParameter)
{
    uint8_t ledstate = 0;
    uint8_t SendCount = 0;
    gpio_reset_pin(BLINK1_GPIO);
    gpio_reset_pin(BUTTER_GPIO);
    gpio_set_direction(BLINK1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUTTER_GPIO, GPIO_MODE_INPUT);

    while(1)
    {
        ledstate = !ledstate;
        gpio_set_level(BLINK1_GPIO,ledstate);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        if(gpio_get_level(BUTTER_GPIO) == 0)
        {
            ESP_LOGI(TAG, "BUTTER_TIG");
            _Espnow.Buffer[0] = SendCount++;
            if (esp_now_send(s_example_broadcast_mac, _Espnow.Buffer, 10) != ESP_OK) {
                ESP_LOGE(TAG, "Send error");
            }
        }
    }
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if(espnow_queue != NULL)
    {
        _Espnow.State = ESPNOW_TX;
        xQueueSend(espnow_queue, &_Espnow, 100);
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if(espnow_queue != NULL)
    {
        _Espnow.State = ESPNOW_RX;
        _Espnow.Len = len;
        memcpy(_Espnow.addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
        memcpy(_Espnow.Buffer,data,len);
        xQueueSend(espnow_queue, &_Espnow, 100);
    }
}

void app_main(void)
{
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    uint8_t ledstate = 0;
    gpio_reset_pin(BUTTER_GPIO);
    gpio_set_direction(BLINK2_GPIO, GPIO_MODE_OUTPUT);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());

    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
    ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 1;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    _Espnow.Buffer = malloc(256);
    char * recv_data = malloc(1024);

    for (size_t i = 0; i < 256; i++)
    {
        _Espnow.Buffer[i] = i;
    }

    espnow_queue = xQueueCreate(16,10);
    EspnowType * EspnowTig;

    //vTaskDelay(5000 / portTICK_PERIOD_MS);

    xTaskCreate(BlinkTask,"BlinkTask",2048,NULL,4,&BlinkTaskHandle);//状态指示灯任务初始化

    ESP_LOGE(TAG, "StartEspnow");

    while(xQueueReceive(espnow_queue, &EspnowTig, portMAX_DELAY) == pdTRUE)
    {
        switch (_Espnow.State)
        {
            case ESPNOW_TX:
                ESP_LOGI(TAG, "SendFinish");

            break;
            case ESPNOW_RX:
                ledstate = !ledstate;
                gpio_set_level(BLINK2_GPIO,ledstate);
                for (size_t i = 0; i < ESP_NOW_ETH_ALEN; i++)
                {
                    sprintf(recv_data + i*5,"0x%02X ",_Espnow.addr[i]);
                }
                ESP_LOGI(TAG, "ResevAddr:%s",recv_data);
                for (size_t i = 0; i < _Espnow.Len; i++)
                {
                    sprintf(recv_data + i*5,"0x%02X ",_Espnow.Buffer[i]);
                }
                ESP_LOGI(TAG, "ResevData:%s\r\n",recv_data);
            break;
            default:
                ESP_LOGE(TAG, "OtherState");
            break;
        }
    }
}