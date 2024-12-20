/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "esp_err.h"
#include "esp_netif_types.h"
#include "esp_wifi_types_generic.h"
#include "nvs_wifi_connect_private.h"
#include "nvs_wifi_connect.h"

static const char *TAG = "WIFI";
static esp_netif_t *s_wifi_sta_netif = NULL;

static esp_event_handler_instance_t instance_any_id = NULL;
static esp_event_handler_instance_t instance_got_ip = NULL;
static esp_event_handler_instance_t instance_got_ipv6 = NULL;
static esp_event_handler_instance_t instance_on_wifi_connect = NULL;

static void event_handler_sta_connect(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "WiFi disconnected, try to reconnect...");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "WiFi connect failed! ret:%x", err);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_GOT_IP6)
    {
        ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
        ESP_LOGI(TAG, "got ip6:" IPV6STR, IPV62STR(event->ip6_info.ip));
    }
}

static void event_handler_wifi_netif_connect(void *arg, esp_event_base_t event_base,
    int32_t event_id, void *event_data) {
    esp_netif_create_ip6_linklocal(arg);
}

static void wifi_event_handler_ap(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void init_softap(char *ap_ssid, char *ap_pass)
{
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler_ap,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = 0,
            .max_connection = AP_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    strcpy((char *)wifi_config.ap.ssid, ap_ssid);
    strcpy((char *)wifi_config.ap.password, ap_pass);

    if (strlen((char *)wifi_config.ap.password) < 8)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        wifi_config.ap.password[0] = 0;
    }

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s",
             ap_ssid, ap_pass);
}

static void nvs_wifi_shutdown(void)
{
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_GOT_IP6, instance_got_ipv6));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, instance_on_wifi_connect));
    esp_wifi_disconnect();
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return;
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(s_wifi_sta_netif));
    esp_netif_destroy(s_wifi_sta_netif);
    s_wifi_sta_netif = NULL;
}

static esp_err_t init_sta(char *sta_ssid, char *sta_pass)
{
    esp_err_t err = ESP_OK;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  

    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
    // esp_netif_config.if_desc = NETIF_WIFI_DESC_NAME;
    esp_netif_config.route_prio = 128;
    s_wifi_sta_netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    esp_wifi_set_default_wifi_sta_handlers();


    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler_sta_connect,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler_sta_connect,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_GOT_IP6,
                                                        &event_handler_sta_connect,
                                                        NULL,
                                                        &instance_got_ipv6));


    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        WIFI_EVENT_STA_CONNECTED,
                                                        &event_handler_wifi_netif_connect,
                                                        s_wifi_sta_netif,
                                                        &instance_on_wifi_connect));
            
    wifi_config_t wifi_config = {
        .sta = {
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            // .threshold.rssi = -127,
            // .threshold.authmode = WIFI_AUTH_WPA2_PSK
        },
    };
    // set minimum security to open
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

    strcpy((char *)wifi_config.sta.ssid, sta_ssid);
    strcpy((char *)wifi_config.sta.password, sta_pass);

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&nvs_wifi_shutdown));

    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed! ret:%x", ret);
        return ret;
    }
    return err;
}

esp_err_t nvs_wifi_connect(struct network_init_cfg_t* apConfig)
{
    if (apConfig->wifiMode == WIFI_MODE_STA) {
        ESP_LOGI(TAG, "Connecting to WiFi with SSID: %s", apConfig->staWifiSsid);
        if (init_sta(apConfig->staWifiSsid, apConfig->staWifiPassword) == ESP_OK) {
            return ESP_OK;
        }
    }
    if (apConfig->wifiMode == WIFI_MODE_AP) {
        ESP_LOGI(TAG, "Creating SoftAP with SSID: %s", apConfig->apWifiSsid);
        init_softap(apConfig->apWifiSsid, apConfig->apWifiPassword);
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t nvs_wifi_connect_init_sta(char *sta_ssid, char *sta_pass)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    return init_sta(sta_ssid, sta_pass);
}
void nvs_wifi_connect_init_softap(char *sta_ssid, char *sta_pass)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    init_softap(sta_ssid, sta_pass);
}
