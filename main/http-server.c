/* 
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include "esp_err.h"
#include "esp_netif.h"

#include <esp_http_server.h>

#include "nvs_wifi_connect.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

/* A simple example that demonstrates using websocket echo server
 */
static const char *TAG = "example_ws_echo_server";

bool display_handle_websocket_packet(httpd_req_t *req, httpd_ws_frame_t* ws_pkt);
 
static esp_err_t websocket_server(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len) {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
    }
    // print the type of frame
    switch (ws_pkt.type) {
        case HTTPD_WS_TYPE_TEXT:
            ESP_LOGI(TAG, "Received TEXT frame");
            break;
        case HTTPD_WS_TYPE_BINARY:
            ESP_LOGI(TAG, "Received BINARY frame");
            break;
        case HTTPD_WS_TYPE_CLOSE:
            ESP_LOGI(TAG, "Received CLOSE frame");
            break;
        case HTTPD_WS_TYPE_PING:
            ESP_LOGI(TAG, "Received PING frame");
            break;
        case HTTPD_WS_TYPE_PONG:
            ESP_LOGI(TAG, "Received PONG frame");
            break;
        default:
            ESP_LOGI(TAG, "Received UNKNOWN frame");
            ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);
            break;
    }
    if (!display_handle_websocket_packet(req, &ws_pkt)) {
        ESP_LOGI(TAG, "Send default response");
        ret = httpd_ws_send_frame(req, &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
        }
    }
    free(buf);
    return ret;
}


static const httpd_uri_t example_ws = {
    .uri        = "/ws",
    .method     = HTTP_GET,
    .handler    = websocket_server,
    .user_ctx   = NULL,
    .is_websocket = true
};

static esp_err_t get_handler(httpd_req_t *req)
{
    extern const unsigned char index_html_start[] asm("_binary_index_html_start");
    extern const unsigned char index_html_end[] asm("_binary_index_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);

    httpd_resp_send_chunk(req, (const char *)index_html_start, index_html_size);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}
static const httpd_uri_t example_gh = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_handler,
    .user_ctx = NULL
};
// css file handler
//filename /style.css
static esp_err_t get_css_handler(httpd_req_t *req)
{
    extern const unsigned char style_css_start[] asm("_binary_style_css_start");
    extern const unsigned char style_css_end[] asm("_binary_style_css_end");
    const size_t style_css_size = (style_css_end - style_css_start);
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send_chunk(req, (const char *)style_css_start, style_css_size);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}
static const httpd_uri_t example_css = {
    .uri = "/style.css",
    .method = HTTP_GET,
    .handler = get_css_handler,
    .user_ctx = NULL
};

esp_err_t example_register_uri_handler(httpd_handle_t server)
{
    esp_err_t ret = ESP_OK;
    ret = httpd_register_uri_handler(server, &example_gh);
    if (ret)
        goto _ret;
    ret = httpd_register_uri_handler(server, &example_css);
    if (ret)
        goto _ret;
    ret = httpd_register_uri_handler(server, &example_ws);
    if (ret)
        goto _ret;
_ret:
    return ret;
}
