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
#include "esp_random.h"
#include "nvs_flash.h"

#define CONFIG_ESPNOW_PMK "pmk1234567890123"
#define CONFIG_ESPNOW_LMK "lmk1234567890123"
#define ESPNOW_HEAD_LEN 7
#define ESPNOW_TEST_DATA_LEN 20

static const char *TAG = "";
/*
#define BLINK1_GPIO GPIO_NUM_12
#define BLINK2_GPIO GPIO_NUM_13
#define BUTTER_GPIO GPIO_NUM_9
*/

#define BLINK1_GPIO GPIO_NUM_25
#define BLINK2_GPIO GPIO_NUM_22
#define BUTTER_GPIO GPIO_NUM_26

uint8_t ispairin = false;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static QueueHandle_t espnow_queue = NULL;

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

enum {
    ESPNOW_DATA_TYPE_READY,
    ESPNOW_DATA_TYPE_UNREADY,
    ESPNOW_DATA_TYPE_MAX,
};

enum espnow_state{
    STATE_TX_TG,
    STATE_TX_CD,
    STATE_RX_CD,
    STATE_MAX,
};

typedef struct {
    uint8_t type;                         //数据类型
    uint16_t seq_num;                     //数据编号
    uint16_t payloadlen;                  //负载数据长度
    uint16_t crc;                         //CRC16校验
    uint8_t payload[0];                   //负载数据
} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    uint8_t state;                          //Indicate that if has received broadcast ESPNOW data or not.、
    uint8_t send_state;                          //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t bufferlen;                     //MAC address of destination device.
    uint8_t src_mac[ESP_NOW_ETH_ALEN];       //MAC address of destination device.
    uint8_t des_mac[ESP_NOW_ETH_ALEN];       //MAC address of destination device.
    uint8_t *buffer;                        //Buffer pointing to ESPNOW data.
} espnow_param_t;


void espnow_data_prepare(espnow_param_t * param,uint8_t *data ,uint16_t len)
{
    espnow_data_t *buff = (espnow_data_t *)param->buffer;

    param->bufferlen = len + ESPNOW_HEAD_LEN;
    buff->payloadlen = len;
    buff->seq_num++;
    memcpy(buff->payload, data, len);
    buff->crc = 0;
    buff->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)param->buffer, param->bufferlen);
}

void espnow_data_set_type(espnow_param_t * param,uint8_t type)
{
    espnow_data_t *buff = (espnow_data_t *)param->buffer;
    buff->type = type;
}

uint8_t espnow_data_get_type(espnow_param_t * param)
{
    espnow_data_t *buff = (espnow_data_t *)param->buffer;
    return buff->type;
}

bool espnow_data_parse(espnow_param_t * param)
{
    espnow_data_t *buff = (espnow_data_t *)param->buffer;

    uint16_t crc = buff->crc;
    buff->crc = 0;
    buff->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)param->buffer, param->bufferlen);

    if (crc != buff->crc)
    {
        ESP_LOGE(TAG, "RecvCrc:%04X CalCrc:%04X",crc,buff->crc);
        return false;
    }
    return true;
}

TaskHandle_t BlinkTaskHandle;
TaskHandle_t EspNowSendTaskHandle;

static void BlinkTask(void *pvParameter)
{
    uint8_t ledstate = 0;
    gpio_reset_pin(BLINK1_GPIO);
    gpio_set_direction(BLINK1_GPIO, GPIO_MODE_OUTPUT);

    while(1)
    {
        ledstate = !ledstate;
        gpio_set_level(BLINK1_GPIO,ledstate);
        if(ispairin == true)
            vTaskDelay(100 / portTICK_PERIOD_MS);
        else
            vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


static void EspNowSendTask(void *pvParameter)
{
    espnow_param_t * espnow_send_param = pvParameter;
    uint8_t * send_data;
    int16_t send_state;
    int8_t espnow_send_type;
    espnow_send_param->buffer = malloc(256);
    memcpy(espnow_send_param->des_mac,s_example_broadcast_mac,ESP_NOW_ETH_ALEN);

    send_data = malloc(ESPNOW_TEST_DATA_LEN);

    gpio_reset_pin(BUTTER_GPIO);
    gpio_set_direction(BUTTER_GPIO, GPIO_MODE_INPUT);

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    espnow_data_set_type(espnow_send_param,ESPNOW_DATA_TYPE_UNREADY);     //启动时配置为未就绪模式

    esp_fill_random(send_data,ESPNOW_TEST_DATA_LEN);
    espnow_data_prepare(espnow_send_param,send_data,ESPNOW_TEST_DATA_LEN);

    while(1)
    {
        espnow_send_type = espnow_data_get_type(espnow_send_param);

        if(gpio_get_level(BUTTER_GPIO) == 0)
        {
            esp_fill_random(send_data,ESPNOW_TEST_DATA_LEN);
            espnow_data_prepare(espnow_send_param,send_data,ESPNOW_TEST_DATA_LEN);
            send_state = esp_now_send(espnow_send_param->des_mac,espnow_send_param->buffer,espnow_send_param->bufferlen);

            if (send_state != ESP_OK) {
                ESP_LOGE(TAG, "EspNowSendError:0x%04X",send_state);
            }
            ESP_LOGI(TAG, "EspNowSend:Unicast");
        }
        else if(IS_BROADCAST_ADDR(espnow_send_param->des_mac))
        {
            esp_fill_random(send_data,ESPNOW_TEST_DATA_LEN);
            espnow_data_prepare(espnow_send_param,send_data,ESPNOW_TEST_DATA_LEN);
            send_state = esp_now_send(espnow_send_param->des_mac,espnow_send_param->buffer,espnow_send_param->bufferlen);
            if (send_state != ESP_OK) {
                ESP_LOGE(TAG, "EspNowSendError:0x%04X",send_state);
            }
            ESP_LOGI(TAG, "EspNowSend:Broadcast");
            vTaskDelayUntil(&xLastWakeTime,90 / portTICK_PERIOD_MS);
        }

        vTaskDelayUntil(&xLastWakeTime,10 / portTICK_PERIOD_MS);
    }
}

static void EspNowRecvTask(void *pvParameter)
{

    while (1)
    {

    }
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if(espnow_queue != NULL)
    {
        espnow_param_t send_param;
        send_param.state = STATE_TX_CD;
        send_param.send_state = status;
        memcpy(send_param.des_mac, mac_addr, ESP_NOW_ETH_ALEN);
        if(xQueueSend(espnow_queue, &send_param, 5) != pdTRUE)
        {
            ESP_LOGE(TAG, "send_cb_queue_full");
        }
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if(espnow_queue != NULL)
    {
        espnow_param_t recv_param;
        recv_param.bufferlen = len;
        recv_param.state = STATE_RX_CD;
        recv_param.buffer = malloc(len);

        memcpy(recv_param.src_mac, recv_info->src_addr, ESP_NOW_ETH_ALEN);
        memcpy(recv_param.des_mac, recv_info->des_addr, ESP_NOW_ETH_ALEN);
        memcpy(recv_param.buffer, data, len);

        if(xQueueSend(espnow_queue, &recv_param, 5) != pdTRUE)
        {
            free(recv_param.buffer);
            ESP_LOGE(TAG, "recv_cb_queue_full");
        }
    }
}

void app_main(void)
{
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    espnow_param_t espnow_cd_param;
    espnow_param_t espnow_send_param;
    TickType_t ticks = 0;
    TickType_t ticksbask = 0;

    char * DebugStrBuff = malloc(256);//格式化数据输出缓存

    gpio_reset_pin(BLINK2_GPIO);
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
    esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA,WIFI_PHY_RATE_LORA_250K);
    //esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA,WIFI_PHY_RATE_MCS7_SGI);
    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 1;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    espnow_queue = xQueueCreate(16,sizeof(espnow_param_t));

    xTaskCreate(BlinkTask,"BlinkTask",2048,NULL,4,&BlinkTaskHandle);                                //状态指示灯任务初始化
    xTaskCreate(EspNowSendTask,"EspNowSendTask",2048,&espnow_send_param,4,&EspNowSendTaskHandle);      //数据发送任务初始化

    while(xQueueReceive(espnow_queue,&espnow_cd_param, portMAX_DELAY) == pdTRUE)
    {
        switch (espnow_cd_param.state)
        {
            case STATE_TX_TG://目前用轮询IO的方式来发送，先不用这个状态
                ESP_LOGI(TAG, "STATE_TX_TG");

            break;
            case STATE_TX_CD:
                ESP_LOGI(TAG, "STATE_TX_CD");

                if(espnow_cd_param.send_state == ESP_NOW_SEND_FAIL)
                {
                    ESP_LOGE(TAG, "ESP_NOW_SEND_FAIL");
                }

                /*
                if(IS_BROADCAST_ADDR(_espnow_param.mac) != 0)//非广播地址
                {
                    memcpy(espnow_send_param.mac, _espnow_param.mac, ESP_NOW_ETH_ALEN);//发送更新为当前mac
                }
                */

                /*输出接收mac地址
                for (size_t i = 0; i < ESP_NOW_ETH_ALEN; i++)
                {
                    sprintf(DebugStrBuff + i*5,"0x%02X ",espnow_cd_param.des_mac[i]);
                }
                ESP_LOGI(TAG, "SendMac: %s",DebugStrBuff);
                */
            break;
            case STATE_RX_CD:
                gpio_set_level(BLINK2_GPIO,0);
                ESP_LOGI(TAG, "STATE_RX_CD");

                /*输出接收src_mac地址
                for (size_t i = 0; i < ESP_NOW_ETH_ALEN; i++)
                {
                    sprintf(DebugStrBuff + i*5,"0x%02X ",espnow_cd_param.src_mac[i]);
                }
                ESP_LOGI(TAG, "RecvSrcMac: %s",DebugStrBuff);
                */
                /*输出接收des_mac地址
                for (size_t i = 0; i < ESP_NOW_ETH_ALEN; i++)
                {
                    sprintf(DebugStrBuff + i*5,"0x%02X ",espnow_cd_param.des_mac[i]);
                }
                ESP_LOGI(TAG, "RecvDesMac: %s",DebugStrBuff);
                */
                /*输出接收数据
                for (size_t i = 0; i < _espnow_param.bufferlen; i++)
                {
                    sprintf(DebugStrBuff + i*5,"0x%02X ",_espnow_param.buffer[i]);
                }
                ESP_LOGI(TAG, "RecvData: %s",DebugStrBuff);
                */

                /*输出接收数据CRC结果*/
                if(espnow_data_parse(&espnow_cd_param) == true)
                {
                    ticks = xTaskGetTickCount();
                    ESP_LOGI(TAG, "RecvCrcOk");
                    ESP_LOGI(TAG, "%ld",ticks - ticksbask);
                    ticksbask = ticks;
                }

                if (ispairin == false)
                {
                    if (esp_now_is_peer_exist(espnow_cd_param.src_mac) == false) 
                    {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = 1;
                        peer->ifidx = ESP_IF_WIFI_STA;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, espnow_cd_param.src_mac, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);

                        espnow_data_set_type(&espnow_send_param,ESPNOW_DATA_TYPE_READY);//发送更新为广播READ模式,通知对方已经准备好对方mac

                        ESP_LOGI(TAG, "AddNewMac");
                        vTaskDelay(200);//等200ms，确保自己能发出一个READY信号
                    }
                    
                    if(espnow_data_get_type(&espnow_cd_param) == ESPNOW_DATA_TYPE_READY)//自己已经注册地址，且对方就绪后切换为单播模式，发送更新为当前mac
                    {
                        ispairin = true;
                        espnow_data_set_type(&espnow_send_param,ESPNOW_DATA_TYPE_READY);//发送更新为广播READ模式,通知对方已经准备好对方mac
                        memcpy(espnow_send_param.des_mac, espnow_cd_param.src_mac, ESP_NOW_ETH_ALEN);
                        ESP_LOGI(TAG, "UnicastMod");
                    }
                }
                else
                {
                    if(espnow_data_get_type(&espnow_cd_param) == ESPNOW_DATA_TYPE_UNREADY)//当收到广播信号时从新执行配对流程
                    {
                        ispairin = false;
                        memcpy(espnow_send_param.des_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
                        xQueueReset(espnow_queue);
                        vTaskDelay(400);//等200ms，确保自己能发出一个READY信号
                    }
                }
                
                free(espnow_cd_param.buffer);
                gpio_set_level(BLINK2_GPIO,1);
            break;
            default:
                ESP_LOGE(TAG, "OtherState");
            break;
        }
    }
}