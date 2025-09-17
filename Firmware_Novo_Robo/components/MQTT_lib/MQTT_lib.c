#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "MQTT_lib.h"

// #define MQTT_RECONNECT_TIMEOUT_MS 0

static const char *TAG = "Biblioteca MQTT";

extern float left_speed_setpoint;
extern float right_speed_setpoint;
extern int operation_mode;

// Handle para o client MQTT
static esp_mqtt_client_handle_t client;

// Grupo de eventos e variáveis para checar conexão mqtt
static EventGroupHandle_t status_mqtt_event_group;
#define MQTT_CONNECTED BIT0

static void mqtt_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id){
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        xEventGroupSetBits(status_mqtt_event_group, MQTT_CONNECTED);
        mqtt_publish("SLAM/robot2/status", "Online", 1, 0);
        mqtt_subscribe("SLAM/vel/#", 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED");
        xEventGroupClearBits(status_mqtt_event_group, MQTT_CONNECTED);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED from msg_id = %d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED from msg_id = %d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED");
        break;
    case MQTT_EVENT_DATA:
    {
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");

        // Topico
        size_t tlen = event->topic_len;
        if (tlen == 0) break;
        char *topic = malloc(tlen + 1);
        if (!topic) {
            ESP_LOGE(TAG, "malloc topic failed");
            break;
        }
        memcpy(topic, event->topic, tlen);
        topic[tlen] = '\0';

        // Mensagem
        size_t mlen = event->data_len;
        char *message = NULL;
        if (mlen > 0) {
            message = malloc(mlen + 1);
            if (!message) {
                ESP_LOGE(TAG, "malloc message failed");
                free(topic);
                break;
            }
            memcpy(message, event->data, mlen);
            message[mlen] = '\0';
        } else {
            // Mensagem vazia
            message = malloc(1);
            if (message) message[0] = '\0';
        }

        // Exemplo de parsing seguro
        float value = 0.0f;
        if (message && strlen(message) > 0) {
            value = strtof(message, NULL);
        }

        if (strcmp(topic, "SLAM/vel/left") == 0)
        {
            ESP_LOGI("LEFT", "%.3f", value);
            left_speed_setpoint = value;
        }
        else if (strcmp(topic, "SLAM/vel/right") == 0)
        {
            ESP_LOGI("RIGHT", "%.3f", value);
            right_speed_setpoint = value;
        } else {
            ESP_LOGI(TAG, "TOPIC RECEIVED: %s MSG: %s", topic, message ? message : "(null)");
        }

        free(topic);
        free(message);
        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_ESP_TLS) {
            ESP_LOGE(TAG, "TLS error: %d", event->error_handle->esp_tls_last_esp_err);
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGE(TAG, "Connection refused, code: %d", event->error_handle->connect_return_code);
        } else {
            ESP_LOGE(TAG, "Unknown error type: %d", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG, "EVENTO DESCONHECIDO id:%d", event->event_id);
        break;
    }    
}

void mqtt_start(void){

    status_mqtt_event_group = xEventGroupCreate();
    //Configuração da estrutura do cliente MQTT
    esp_mqtt_client_config_t esp_mqtt_client_cfg = {
        .network.disable_auto_reconnect = false,                    // Habilitar reconexão 
        // .network.reconnect_timeout_ms = 0,
        .broker.address.uri = "mqtt://10.7.220.187",
        // .broker.address.uri = "mqtt://mqtt.eclipseprojects.io",
        .broker.address.port = 1883,
        .session.keepalive = 120,                                    // Padrão é 120 segundos
        .session.last_will = {
            .topic = "SLAM/robot2/status",
            .msg = "Offline",
            .msg_len = strlen("Offline"),
            .retain = 0
        }
    };

    client = esp_mqtt_client_init(&esp_mqtt_client_cfg);

    //Event handler para conexão MQTT
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void mqtt_subscribe(char *topic, int qos){

    int msg_id = esp_mqtt_client_subscribe(client, topic, qos);
    ESP_LOGI(TAG, "Subscriptin ID = %d", msg_id);
}

void mqtt_unsubscribe(char *topic){

    int msg_id = esp_mqtt_client_unsubscribe(client, topic);
    ESP_LOGI(TAG, "Unsubsciption ID = %d", msg_id);
}

void mqtt_publish(char *topic, char *payload, int qos, int retain){

    int msg_id = esp_mqtt_client_publish(client, topic, payload, strlen(payload), qos, retain);
    // ESP_LOGI(TAG, "ID da publicação = %d", msg_id);
}

int mqtt_connected(void){
    EventBits_t bits = xEventGroupGetBits(status_mqtt_event_group);

    if(bits & MQTT_CONNECTED){
        return 1;
    }else {
        return 0;
    }
}

// void mqtt_reconnect(void)
// {
//     esp_mqtt_client_stop(client);
//     esp_mqtt_client_reconnect(client);
// }