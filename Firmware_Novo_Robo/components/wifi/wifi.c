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
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

// TAG para info
static const char *TAG = "Wi-Fi";

//Sincronizar as tarefas conforme os eventos WiFi ocorrem
static EventGroupHandle_t s_wifi_event_group;

static EventGroupHandle_t s_wifi_evgrp = NULL;
#define BIT_WIFI_STARTED      (1 << 0)
#define BIT_WIFI_SCAN_DONE    (1 << 1)
#define BIT_WIFI_CONNECTED    (1 << 2)
#define BIT_WIFI_FAIL         (1 << 3)
#define BIT_WIFI_DISCONNECTED (1 << 4)

// Estado e variáveis
static int s_retry_num = 0; // Quantidade de tentativas para conexão
static bool reconnect = false; // Para tentativas de reconexão
static esp_netif_t *esp_netif; // *s_esp_netif = NULL;

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

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "WIFI_EVENT_STA_START -> iniciar SCAN assíncrono");
            wifi_scan_config_t scan_config = {0};
            // non-blocking: retorna rapidamente, scan_done será gerado
            esp_err_t err = esp_wifi_scan_start(&scan_config, false);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_wifi_scan_start falhou: %s", esp_err_to_name(err));
            }
        } else if (event_id == WIFI_EVENT_SCAN_DONE) {
            uint16_t ap_num = 0;
            esp_err_t err = esp_wifi_scan_get_ap_num(&ap_num);
            if (err != ESP_OK || ap_num == 0) {
                ESP_LOGW(TAG, "nenhum AP encontrado ou erro: %s", esp_err_to_name(err));
                return;
            }
            // aloca no heap, não na pilha
            wifi_ap_record_t *ap_records = malloc(sizeof(wifi_ap_record_t) * ap_num);
            if (!ap_records) {
                ESP_LOGE(TAG, "malloc AP records falhou");
                return;
            }
            err = esp_wifi_scan_get_ap_records(&ap_num, ap_records);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_wifi_scan_get_ap_records falhou: %s", esp_err_to_name(err));
                free(ap_records);
                return;
            }

            // Processar ap_records (escolher melhor known SSID)
            int strongest_idx = -1;
            int strongest_rssi = -127;
            for (int i = 0; i < ap_num; ++i) {
                for (int j = 0; j < WIFI_CRED_LIST_SIZE; ++j) {
                    if (strcmp((char*)ap_records[i].ssid, wifi_cred_list[j].ssid) == 0) {
                        if (ap_records[i].rssi > strongest_rssi) {
                            strongest_rssi = ap_records[i].rssi;
                            strongest_idx = j;
                        }
                    }
                }
            }

            if (strongest_idx >= 0) {
                wifi_config_t wifi_config = {0};
                strncpy((char*)wifi_config.sta.ssid, wifi_cred_list[strongest_idx].ssid, sizeof(wifi_config.sta.ssid)-1);
                strncpy((char*)wifi_config.sta.password, wifi_cred_list[strongest_idx].password, sizeof(wifi_config.sta.password)-1);
                wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

                // Fazer conexão: NÃO chame esp_wifi_start() aqui (já startado); chame esp_wifi_set_config + esp_wifi_connect
                ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
                ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
                esp_err_t errc = esp_wifi_connect();
                if (errc != ESP_OK) {
                    ESP_LOGE(TAG, "esp_wifi_connect falhou: %s", esp_err_to_name(errc));
                }
            } else {
                ESP_LOGW(TAG, "Nenhuma rede conhecida detectada no scan");
            }

            free(ap_records);
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            // Estrutura para o evento da desconexão no modo station
            wifi_event_sta_disconnected_t *wifi_event_sta_disconnected = event_data;

            // Visualizando o motivo da desconexão
            ESP_LOGW(TAG, "Motivo da Desconexão %d: %s", wifi_event_sta_disconnected->reason, get_wifi_disconnection_string(wifi_event_sta_disconnected->reason));
            
            if(reconnect){
                /*  
                    Tentar novamente conexão se o motivo for:
                    if(wifi_event_sta_disconnected->reason == WIFI_REASON_NO_AP_FOUND ||
                    wifi_event_sta_disconnected->reason == WIFI_REASON_ASSOC_LEAVE ||
                    wifi_event_sta_disconnected->reason == WIFI_REASON_AUTH_EXPIRE){
                */

                // Tentando reconectar
                if(s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY){
                    esp_wifi_connect();
                    s_retry_num++;
                    ESP_LOGI(TAG, "Tentando se conectar %d", s_retry_num);
                }else{
                    // Caso não consiga conexão indica ao grupo de eventos
                    ESP_LOGI(TAG, "Falha na conexão");
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // Verificando se o evento é referente ao IP
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Pegamos um IP " IPSTR, IP2STR(&event->ip_info.ip));

        s_retry_num = 0;

        //Depois de ter conseguido IP a conexão foi realizada com sucesso. Indica ao grupo de eventos
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
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
    esp_err_t err = esp_wifi_scan_start(&scan_config, false); // true bloqueia até o scan completar
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Scan deu ruim: %s", esp_err_to_name(err));
        return false;
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));
    if(ap_num == 0){
        ESP_LOGW(TAG, "Nenhum AP encontrado.");
        return false;
    }

    wifi_ap_record_t ap_records[ap_num];
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

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

// Função auxiliar: conecta com a config escolhida
static void wifi_connect_sta(wifi_config_t *wifi_config){
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, wifi_config));
    // ESP_ERROR_CHECK(esp_wifi_start());

    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect falhou: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Wifi configurado, aguardando eventos...");

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);
    
    if(bits & WIFI_CONNECTED_BIT){
        ESP_LOGI(TAG, "Conectado ao AP SSID: %s password: %s",
            wifi_config->sta.ssid, wifi_config->sta.password);
    }
    else if(bits & WIFI_FAIL_BIT){
        ESP_LOGE(TAG, "Falha ao se conectar na %s",
            wifi_config->sta.ssid);
    }
    else {
        ESP_LOGE(TAG, "Evento inesperado");
    }
}

void wifi_init(){

    // Init Phase - Wi-Fi Driver and LwIP
    ESP_ERROR_CHECK(esp_netif_init());                       // LwIP
    ESP_ERROR_CHECK(esp_event_loop_create_default());        // Event loop for event task   
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();     // Driver Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Manipulador de eventos WiFi
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Realizando o storage na RAM. Deixa a conexão um pouco mais rápida.
    // É uma outra forma de configurar, ao invés de modo persistente na NVS.
    // Obs: Perdendo a conexão é preciso realizar a configurações novamente (executa essa mesma rotina).
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
}

void wifi_init_sta(void){

    wifi_init();
    reconnect = true;
        
    s_wifi_event_group = xEventGroupCreate(); 
    esp_netif = esp_netif_create_default_wifi_sta();    // Cria LwIP para config station

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_config_t wifi_config;
    if(!wifi_scan_and_select_ap(&wifi_config)){
        ESP_LOGE(TAG, "Scan não encontrou porra nenhuma");
        return;
    }

    wifi_connect_sta(&wifi_config);
}

void wifi_init_ap(const char *ssid, const char *pass){
    
    // Função de configuração
    wifi_init();
    ESP_LOGI(TAG, "INICIALIZANDO O WI-FI!");
    esp_netif = esp_netif_create_default_wifi_ap();     // LwIP com configurações básicas para o modo AP.

    // Wi-Fi Configuration Phase
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &ap_event_handler, NULL, NULL));
    
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, pass, sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.max_connection = 5;
    wifi_config.ap.beacon_interval = 100;   // Deve ser múltiplo de 100. Faixa de 100 a 600000. Unit: TU(time unit, 1 TU = 1024 us).
    wifi_config.ap.channel = 1;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    // Wi-Fi start Phase
    esp_wifi_start();

    ESP_LOGI(TAG, "ESP32 COMO AP PRONTO! CONECTAR AO SSID: %s, PASSWORD: %s E CANAL: %d", 
            wifi_config.ap.ssid, wifi_config.ap.password, wifi_config.ap.channel);
}

void wifi_disconnect(){
    reconnect = false;
    esp_wifi_stop();
    esp_netif_destroy(esp_netif);
}