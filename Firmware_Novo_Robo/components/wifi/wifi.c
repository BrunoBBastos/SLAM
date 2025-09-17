// Bibliotecas
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif_ip_addr.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi.h"

// CONFIG
#define WIFI_MANAGER_TASK_STACK_SIZE 4096
#define WIFI_MANAGER_TASK_PRIORITY 5
#define WIFI_MANAGER_TIMEOUT_MS 10000
#define WIFI_MANAGER_CONNECT_WAIT_MS 5000
#define WIFI_MANAGER_MAX_RETRY CONFIG_ESP_MAXIMUM_RETRY

// Credenciais do Wi-Fi definidas no menuconfig
typedef struct {
    char ssid[32];
    char password[64];
} wifi_cred_t;

wifi_cred_t wifi_cred_list[] = {
    {"LARS-301-2.4GHz", "LARS@ROBOTICA"},
    {"LARS-301-5GHz", "LARS@ROBOTICA"},
    // {"SSIDqualquer", "password"},
};

#define WIFI_CRED_LIST_SIZE (sizeof(wifi_cred_list)/sizeof(wifi_cred_list[0]))


#define EXAMPLE_ESP_MAXIMUM_RETRY   CONFIG_ESP_MAXIMUM_RETRY

// Definição dos bits para o grupo de eventos (BIT0 e BIT1 já são macros, não precisamos realizar deslocamento de bits)
// #define WIFI_CONNECTED_BIT  BIT0
// #define WIFI_FAIL_BIT       BIT1

// TAG para info
static const char *TAG = "Wi-Fi";

//Sincronizar as tarefas conforme os eventos WiFi ocorrem
// static EventGroupHandle_t s_wifi_event_group;

static EventGroupHandle_t s_wifi_evgrp = NULL;
#define BIT_WIFI_STARTED      (1 << 0)
#define BIT_WIFI_SCAN_DONE    (1 << 1)
#define BIT_WIFI_CONNECTED    (1 << 2)
#define BIT_WIFI_FAIL         (1 << 3)
#define BIT_WIFI_DISCONNECTED (1 << 4)

// Estado e variáveis
static int s_retry_num = 0; // Quantidade de tentativas para conexão
static bool s_reconnect = false; // Para tentativas de reconexão
static bool s_wifi_inited = false;
static esp_netif_t *s_esp_netif = NULL; // *s_esp_netif = NULL;

// Motivo da desconexão
char *get_wifi_disconnection_string(wifi_err_reason_t wifi_err_reason)
{
    switch (wifi_err_reason)
    {
    case WIFI_REASON_UNSPECIFIED: return "WIFI_REASON_UNSPECIFIED";
    case WIFI_REASON_AUTH_EXPIRE: return "WIFI_REASON_AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE: return "WIFI_REASON_AUTH_LEAVE";
    case WIFI_REASON_ASSOC_EXPIRE: return "WIFI_REASON_ASSOC_EXPIRE";
    case WIFI_REASON_ASSOC_TOOMANY: return "WIFI_REASON_ASSOC_TOOMANY";
    case WIFI_REASON_NOT_AUTHED: return "WIFI_REASON_NOT_AUTHED";
    case WIFI_REASON_NOT_ASSOCED: return "WIFI_REASON_NOT_ASSOCED";
    case WIFI_REASON_ASSOC_LEAVE: return "WIFI_REASON_ASSOC_LEAVE";
    case WIFI_REASON_ASSOC_NOT_AUTHED: return "WIFI_REASON_ASSOC_NOT_AUTHED";
    case WIFI_REASON_DISASSOC_PWRCAP_BAD: return "WIFI_REASON_DISASSOC_PWRCAP_BAD";
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD: return "WIFI_REASON_DISASSOC_SUPCHAN_BAD";
    case WIFI_REASON_BSS_TRANSITION_DISASSOC: return "WIFI_REASON_BSS_TRANSITION_DISASSOC";
    case WIFI_REASON_IE_INVALID: return "WIFI_REASON_IE_INVALID";
    case WIFI_REASON_MIC_FAILURE: return "WIFI_REASON_MIC_FAILURE";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT: return "WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT";
    case WIFI_REASON_IE_IN_4WAY_DIFFERS: return "WIFI_REASON_IE_IN_4WAY_DIFFERS";
    case WIFI_REASON_GROUP_CIPHER_INVALID: return "WIFI_REASON_GROUP_CIPHER_INVALID";
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID: return "WIFI_REASON_PAIRWISE_CIPHER_INVALID";
    case WIFI_REASON_AKMP_INVALID: return "WIFI_REASON_AKMP_INVALID";
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION: return "WIFI_REASON_UNSUPP_RSN_IE_VERSION";
    case WIFI_REASON_INVALID_RSN_IE_CAP: return "WIFI_REASON_INVALID_RSN_IE_CAP";
    case WIFI_REASON_802_1X_AUTH_FAILED: return "WIFI_REASON_802_1X_AUTH_FAILED";
    case WIFI_REASON_CIPHER_SUITE_REJECTED: return "WIFI_REASON_CIPHER_SUITE_REJECTED";
    case WIFI_REASON_TDLS_PEER_UNREACHABLE: return "WIFI_REASON_TDLS_PEER_UNREACHABLE";
    case WIFI_REASON_TDLS_UNSPECIFIED: return "WIFI_REASON_TDLS_UNSPECIFIED";
    case WIFI_REASON_SSP_REQUESTED_DISASSOC: return "WIFI_REASON_SSP_REQUESTED_DISASSOC";
    case WIFI_REASON_NO_SSP_ROAMING_AGREEMENT: return "WIFI_REASON_NO_SSP_ROAMING_AGREEMENT";
    case WIFI_REASON_BAD_CIPHER_OR_AKM: return "WIFI_REASON_BAD_CIPHER_OR_AKM";
    case WIFI_REASON_NOT_AUTHORIZED_THIS_LOCATION: return "WIFI_REASON_NOT_AUTHORIZED_THIS_LOCATION";
    case WIFI_REASON_SERVICE_CHANGE_PERCLUDES_TS: return "WIFI_REASON_SERVICE_CHANGE_PERCLUDES_TS";
    case WIFI_REASON_UNSPECIFIED_QOS: return "WIFI_REASON_UNSPECIFIED_QOS";
    case WIFI_REASON_NOT_ENOUGH_BANDWIDTH: return "WIFI_REASON_NOT_ENOUGH_BANDWIDTH";
    case WIFI_REASON_MISSING_ACKS: return "WIFI_REASON_MISSING_ACKS";
    case WIFI_REASON_EXCEEDED_TXOP: return "WIFI_REASON_EXCEEDED_TXOP";
    case WIFI_REASON_STA_LEAVING: return "WIFI_REASON_STA_LEAVING";
    case WIFI_REASON_END_BA: return "WIFI_REASON_END_BA";
    case WIFI_REASON_UNKNOWN_BA: return "WIFI_REASON_UNKNOWN_BA";
    case WIFI_REASON_TIMEOUT: return "WIFI_REASON_TIMEOUT";
    case WIFI_REASON_PEER_INITIATED: return "WIFI_REASON_PEER_INITIATED";
    case WIFI_REASON_AP_INITIATED: return "WIFI_REASON_AP_INITIATED";
    case WIFI_REASON_INVALID_FT_ACTION_FRAME_COUNT: return "WIFI_REASON_INVALID_FT_ACTION_FRAME_COUNT";
    case WIFI_REASON_INVALID_PMKID: return "WIFI_REASON_INVALID_PMKID";
    case WIFI_REASON_INVALID_MDE: return "WIFI_REASON_INVALID_MDE";
    case WIFI_REASON_INVALID_FTE: return "WIFI_REASON_INVALID_FTE";
    case WIFI_REASON_TRANSMISSION_LINK_ESTABLISH_FAILED: return "WIFI_REASON_TRANSMISSION_LINK_ESTABLISH_FAILED";
    case WIFI_REASON_ALTERATIVE_CHANNEL_OCCUPIED: return "WIFI_REASON_ALTERATIVE_CHANNEL_OCCUPIED";
    case WIFI_REASON_BEACON_TIMEOUT: return "WIFI_REASON_BEACON_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND: return "WIFI_REASON_NO_AP_FOUND";
    case WIFI_REASON_AUTH_FAIL: return "WIFI_REASON_AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL: return "WIFI_REASON_ASSOC_FAIL";
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "WIFI_REASON_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_CONNECTION_FAIL: return "WIFI_REASON_CONNECTION_FAIL";
    case WIFI_REASON_AP_TSF_RESET: return "WIFI_REASON_AP_TSF_RESET";
    case WIFI_REASON_ROAMING: return "WIFI_REASON_ROAMING";
    case WIFI_REASON_ASSOC_COMEBACK_TIME_TOO_LONG: return "WIFI_REASON_ASSOC_COMEBACK_TIME_TOO_LONG";
    case WIFI_REASON_SA_QUERY_TIMEOUT: return "WIFI_REASON_SA_QUERY_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY: return "WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY";
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD: return "WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD";
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD: return "WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD";
    }
    return "UNKNOWN";
}

static void wifi_event_handler(void * arg, esp_event_base_t event_base, int32_t event_id, void *event_data){

    if(event_base == WIFI_EVENT){
        switch(event_id){
            case WIFI_EVENT_STA_START:
                xEventGroupSetBits(s_wifi_evgrp, BIT_WIFI_STARTED);
                ESP_LOGI(TAG, "EVENT: STA_START");
                break;
            case WIFI_EVENT_SCAN_DONE:
                xEventGroupSetBits(s_wifi_evgrp, BIT_WIFI_SCAN_DONE);
                ESP_LOGI(TAG, "EVENT: SCAN_DONE");
                break;
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *) event_data;
                ESP_LOGW(TAG, "EVENT: STA_DISCONNECTED, reason=%d (%s)", ev->reason, get_wifi_disconnection_string(ev->reason));
                xEventGroupSetBits(s_wifi_evgrp, BIT_WIFI_DISCONNECTED);
                break;
            }
            default:
                // ESP_LOGI(TAG, "EVENT: %d", event_id);
                break;
        }
    } else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "EVENT: GOT_IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_evgrp, BIT_WIFI_CONNECTED);
    }
}

static bool select_best_ap(wifi_ap_record_t *ap_records, uint16_t ap_num, wifi_config_t *out_config){
    int strongest_idx = -1;
    int strongest_rssi = -127;
    for(int i = 0; i < ap_num; i++){
        for(int j = 0; j < WIFI_CRED_LIST_SIZE; j++){
            if(strncmp((char *)ap_records[i].ssid, wifi_cred_list[j].ssid, sizeof(ap_records[i].ssid)) == 0){
                if(ap_records[i].rssi > strongest_rssi){
                    strongest_rssi = ap_records[i].rssi;
                    strongest_idx = j;
                }
            }
        }
    }
    if(strongest_idx < 0){
        return false;
    }

    memset(out_config, 0, sizeof(wifi_config_t));
    strncpy((char *)out_config->sta.ssid, wifi_cred_list[strongest_idx].ssid, sizeof(out_config->sta.ssid)-1);
    strncpy((char *)out_config->sta.password, wifi_cred_list[strongest_idx].password, sizeof(out_config->sta.password)-1);
    out_config->sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    out_config->sta.pmf_cfg.capable = true;
    out_config->sta.pmf_cfg.required = false;
    ESP_LOGI(TAG, "Escolhido SSID %s (RSSI %d)", wifi_cred_list[strongest_idx].ssid, strongest_rssi);
    return true;
}

static void wifi_manager_task(void *pvParameters){
    esp_err_t err;
    wifi_config_t wifi_config;

    for(;;){
        // Espera o driver iniciar
        EventBits_t bits = xEventGroupWaitBits(s_wifi_evgrp, BIT_WIFI_STARTED, pdFALSE, pdFALSE, portMAX_DELAY);
        if(!(bits & BIT_WIFI_STARTED)) continue;

        /*
        LOOP:
        scan
        seleciona ap
        conecta
        espera evento
        */
        while(1){
            // iniciar scan não-bloqueante (driver que gera WIFI_EVENT_SCAN_DONE)
            wifi_scan_config_t scan_cfg = {0};
            err = esp_wifi_scan_start(&scan_cfg, false);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_wifi_scan_start failed: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            // aguardar SCAN_DONE com timeout
            bits = xEventGroupWaitBits(s_wifi_evgrp, BIT_WIFI_SCAN_DONE, pdTRUE, pdFALSE, pdMS_TO_TICKS(WIFI_MANAGER_TIMEOUT_MS));
            if (!(bits & BIT_WIFI_SCAN_DONE)) {
                ESP_LOGW(TAG, "Scan timeout/failed");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            // obter lista de APs (alocar na heap)
            uint16_t ap_num = 0;
            err = esp_wifi_scan_get_ap_num(&ap_num);
            if (err != ESP_OK || ap_num == 0) {
                ESP_LOGW(TAG, "Nenhum AP encontrado ou erro: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            wifi_ap_record_t *ap_records = calloc(ap_num, sizeof(wifi_ap_record_t));
            if (!ap_records) {
                ESP_LOGE(TAG, "calloc falhou para ap_records");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            err = esp_wifi_scan_get_ap_records(&ap_num, ap_records);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_wifi_scan_get_ap_records falhou: %s", esp_err_to_name(err));
                free(ap_records);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            // selecionar melhor AP conhecido
            bool found = select_best_ap(ap_records, ap_num, &wifi_config);
            free(ap_records);

            if (!found) {
                ESP_LOGW(TAG, "Nenhuma rede conhecida detectada no scan. Retry em 2s.");
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            // aplicar config e conectar
            ESP_LOGI(TAG, "Setando config e conectando...");
            err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_wifi_set_config falhou: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_wifi_connect falhou: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            // aguardar resultado (IP ou DISCONNECT)
            bits = xEventGroupWaitBits(s_wifi_evgrp, BIT_WIFI_CONNECTED | BIT_WIFI_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(WIFI_MANAGER_CONNECT_WAIT_MS));
            if (bits & BIT_WIFI_CONNECTED) {
                ESP_LOGI(TAG, "Conectado com sucesso!");
                // permanência: task pode encerrar ciclo de tentativa ou ficar verificando desconexões
                // Espera desconexão para tentar reconectar
                xEventGroupWaitBits(s_wifi_evgrp, BIT_WIFI_DISCONNECTED, pdTRUE, pdFALSE, portMAX_DELAY);
                ESP_LOGW(TAG, "Detectada desconexão, tentando reconectar...");
                // loop volta e fará novo scan/seleção
            } else if (bits & BIT_WIFI_DISCONNECTED) {
                ESP_LOGW(TAG, "Não conseguiu conectar (DISCONNECTED bit). Tentando novamente...");
                s_retry_num++;
                if (s_retry_num >= WIFI_MANAGER_MAX_RETRY) {
                    ESP_LOGE(TAG, "Ultrapassou tentativas (%d). Pausando 5s.", s_retry_num);
                    s_retry_num = 0;
                    vTaskDelay(pdMS_TO_TICKS(5000));
                }
            } else {
                ESP_LOGW(TAG, "Timeout esperando conexão. Retry.");
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } // WHILE
    } // FOR
}

// API pública e init
void wifi_init(void){

    if(s_wifi_inited) return;
    s_wifi_inited = true;

    // init subsystems
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // set storage (opcional)
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // criar event group
    if (!s_wifi_evgrp) {
        s_wifi_evgrp = xEventGroupCreate();
    }

    // registrar handler (usando instância para poder desregistrar se quiser)
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
}

static void ap_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){

    // Verificar se o evento gerado é Wi-Fi e qual o motivo
    if(event_id == WIFI_EVENT_AP_STACONNECTED){
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "MAC do Station: "MACSTR" CONECTADO, AID = %d", MAC2STR(event->mac), event->aid);
    } else if(event_id == WIFI_EVENT_AP_STADISCONNECTED){
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "MAC do Station: "MACSTR" DESCONECTADO, AID = %d", MAC2STR(event->mac), event->aid);
    }
}

static bool wifi_scan_and_select_ap(wifi_config_t *wifi_config)
{
    wifi_scan_config_t scan_config = {0};
    uint16_t ap_num = 0;

    ESP_LOGI(TAG, "Iniciando scan de redes...");
    esp_err_t err = esp_wifi_scan_start(&scan_config, true); // true bloqueia até o scan completar
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Scan deu ruim: %s", esp_err_to_name(err));
        return false;
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));
    if(ap_num == 0){
        ESP_LOGW(TAG, "Nenhum AP encontrado.");
        return false;
    }

    wifi_ap_record_t *ap_records = calloc(ap_num, sizeof(wifi_ap_record_t));
    if(!ap_records){
        ESP_LOGE(TAG, "calloc falhou para ap_records");
        return false;
    }
    err = esp_wifi_scan_get_ap_records(&ap_num, ap_records);

    int strongest_idx = -1;
    int strongest_rssi = -127;
    for(int i = 0; i < ap_num; i++){
        for(int j = 0; j < WIFI_CRED_LIST_SIZE; j++){
            if(strcmp((char *)ap_records[i].ssid, wifi_cred_list[j].ssid) == 0){
                if(ap_records[i].rssi > strongest_rssi){
                    strongest_rssi = ap_records[i].rssi;
                    strongest_idx = j;
                }
            }
        }
    }

    free(ap_records);

    if(strongest_idx < 0){
        ESP_LOGE(TAG, "Nenhuma rede conhecida encontrada");
        return false;
    }

    ESP_LOGI(TAG, "SSID escolhido: %s (RSSI %d)",
            wifi_cred_list[strongest_idx].ssid,
            strongest_rssi);

    memset(wifi_config, 0, sizeof(wifi_config_t));
    strncpy((char *)wifi_config->sta.ssid, wifi_cred_list[strongest_idx].ssid,
        sizeof(wifi_config->sta.ssid) -1);
    strncpy((char *)wifi_config->sta.password, wifi_cred_list[strongest_idx].password,
        sizeof(wifi_config->sta.password) -1);
    wifi_config->sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config->sta.pmf_cfg.capable = true;
    wifi_config->sta.pmf_cfg.required = false;

    return true;
}

void wifi_init_sta(void){

    wifi_init();

    s_reconnect = true;
    s_retry_num = 0;
    
    // Cria LwIP para config station
    if(s_esp_netif == NULL){
        s_esp_netif = esp_netif_create_default_wifi_sta(); 
        if(!s_esp_netif){
            ESP_LOGE(TAG, "esp_netif_create_default_wifi_sta falhou");
            return;
        }
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    BaseType_t res = xTaskCreatePinnedToCore(
        wifi_manager_task,
        "wifi_manager_task",
        WIFI_MANAGER_TASK_STACK_SIZE,
        NULL,
        WIFI_MANAGER_TASK_PRIORITY,
        NULL,
        tskNO_AFFINITY);
    if(res != pdPASS){
        ESP_LOGE(TAG, "Falha ao criar a task do Wi-Fi Manager");
    } else {
        ESP_LOGI(TAG, "Task do Wi-Fi Manager criada com sucesso");
    }
}

void wifi_init_ap(const char *ssid, const char *pass){
    
    // Função de configuração
    wifi_init();
    ESP_LOGI(TAG, "INICIALIZANDO O WI-FI!");
    s_esp_netif = esp_netif_create_default_wifi_ap();     // LwIP com configurações básicas para o modo AP.

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, pass, sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.max_connection = 5;
    wifi_config.ap.beacon_interval = 100;   // Deve ser múltiplo de 100. Faixa de 100 a 600000. Unit: TU(time unit, 1 TU = 1024 us).
    wifi_config.ap.channel = 1;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    // Wi-Fi start Phase
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "ESP32 COMO AP PRONTO! CONECTAR AO SSID: %s, PASSWORD: %s E CANAL: %d", 
            wifi_config.ap.ssid, wifi_config.ap.password, wifi_config.ap.channel);
}

void wifi_disconnect(){
    s_reconnect = false;
    esp_wifi_disconnect();
    ESP_LOGI(TAG, "Solicitado desconexão do wifi");
}