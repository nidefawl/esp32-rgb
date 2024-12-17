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

/*
 * This handler echos back the received ws data
 * and triggers an async send if certain message received
 */
static esp_err_t websocket_server_broken(httpd_req_t *req)
{
    char addr_str[128];
    struct sockaddr_storage source_addr;// Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);

    bzero(&source_addr, sizeof(source_addr));
    if (getpeername(httpd_req_to_sockfd(req), (struct sockaddr*) &source_addr, &socklen) < 0) {
      ESP_LOGE(TAG, "Error getting client IP");
      return ESP_FAIL;
    }
    // Get the sender's ip address as string
    if (source_addr.ss_family == PF_INET) {
      inet_ntoa_r(((struct sockaddr_in*) &source_addr)->sin_addr, addr_str,
                  sizeof(addr_str) - 1);
    } else if (source_addr.ss_family == PF_INET6) {
      inet6_ntoa_r(((struct sockaddr_in6*) &source_addr)->sin6_addr, addr_str,
                   sizeof(addr_str) - 1);
    }

    ESP_LOGI(TAG, "WS Request URI: %s, Method %d", req->uri, req->method);
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
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
            break;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    //ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
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
        // make sure trailing 0 is added 
        ws_pkt.payload[ws_pkt.len] = 0;
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        //ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    }
    ESP_LOGI(TAG, "Received %u bytes from %s", ws_pkt.len, addr_str);

    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }

    display_handle_websocket_packet(req, &ws_pkt);

    // ret = httpd_ws_send_frame(req, &ws_pkt);
    // if (httpd_ws_send_frame(req, &ws_pkt) != ESP_OK) {
    //     ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
    // }
    free(buf);
    return ret;
}

/*
 * Structure holding server handle
 * and internal socket fd in order
 * to use out of request send
 */
 struct async_resp_arg {
  httpd_handle_t hd;
  int fd;
};
/*
 * async send function, which we put into the httpd work queue
 */
static void ws_async_send(void *arg)
{
    static const char * data = "Async data";
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)data;
    ws_pkt.len = strlen(data);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
}

static esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    if (resp_arg == NULL) {
        return ESP_ERR_NO_MEM;
    }
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    esp_err_t ret = httpd_queue_work(handle, ws_async_send, resp_arg);
    if (ret != ESP_OK) {
        free(resp_arg);
    }
    return ret;
}

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
