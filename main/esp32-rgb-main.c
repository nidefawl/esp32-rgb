#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "led_strip_spi.h"
#include "led_strip_types.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include "network.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "rgb-network-types.h"
#include <lwip/netdb.h>
#include "driver/ledc.h"

#define CONFIG_USE_IPV4 1
#define CONFIG_USE_IPV6 1

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)
#define LED_STRIP_HAS_GRBW 0
#define LED_STRIP_NUM_STRIPS 10
#define LED_STRIP_NUM_LEDS_PER_STRIP 30

#define LED_UDP_LISTEN_PORT 54321

static const char* TAG                              = "RGB_MAIN";
led_strip_handle_t led_strips[LED_STRIP_NUM_STRIPS] = {};

enum DebugFlags : uint8_t {
  DebugNone = 0,
  DebugShowBufferPosition = 1 << 0,
};

struct DisplayState {
  // display config
  uint8_t maxBrightness;
  uint16_t frameRate;
  uint32_t stripesEnable;
  enum DebugFlags debugFlags;
  uint32_t heartbeatIntervalFrames; // send heartbeat every n frames
  // ring buffer positions
  volatile uint64_t readIndex;
  volatile uint64_t writeIndex;
  // runtime stats
  int64_t numCallbacks;
  int64_t numFrames;
  int64_t numBufferUnderrun;
  int64_t numBufferOverrun;
  int64_t numLastCallbacks;
  int64_t numLastFrames;
};


enum : uint16_t {
  RingBufferLength = 64,
};

struct DisplayState displayState;
volatile uint32_t ledsBuffer[RingBufferLength][LED_STRIP_NUM_STRIPS][LED_STRIP_NUM_LEDS_PER_STRIP];
esp_timer_handle_t periodic_timer;
SemaphoreHandle_t semaphoreDisplay = NULL;
StaticSemaphore_t s_semaphoreDisplay;
SemaphoreHandle_t semaphoreNetwork = NULL;
SemaphoreHandle_t semaphoreSocketStart = NULL;
int g_socket_udp = -1;
struct sockaddr_storage g_udp_master_address = {};

void reset_display_state() {
  memset((void*)ledsBuffer, 0, sizeof(ledsBuffer));
  memset(&displayState, 0, sizeof(displayState));
  displayState.frameRate     = 30;
  displayState.maxBrightness = 127;
  displayState.stripesEnable = ~0;
  displayState.debugFlags    = DebugNone;
  displayState.heartbeatIntervalFrames = 0;
  displayState.readIndex  = 0;
  displayState.writeIndex = RingBufferLength / 2;
}

void reset_display_buffer_position() {
  displayState.readIndex  = 0;
  displayState.writeIndex = RingBufferLength / 2;
  displayState.numFrames = 0;
  displayState.numCallbacks = 0;
  displayState.numBufferUnderrun = 0;
  displayState.numLastFrames = 0;
  displayState.numLastCallbacks = 0;
}

uint8_t scale_and_clamp(uint8_t color, uint8_t maxBrightness) {
  uint32_t scaled = color;
  scaled *= maxBrightness;
  scaled /= 255;
  if (scaled > 255)
    return 255;
  return scaled;
}


void scale_and_clamp_u32(uint32_t color, uint8_t maxBrightness, uint8_t* out) {
  uint8_t w = (color >> 24) & 0xFF;
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;
  out[0]    = scale_and_clamp(r, maxBrightness);
  out[1]    = scale_and_clamp(g, maxBrightness);
  out[2]    = scale_and_clamp(b, maxBrightness);
  out[3]    = scale_and_clamp(w, maxBrightness);
}

uint32_t clamp_u32(uint32_t i, uint32_t i_min, uint32_t i_max) {
  return i < i_min ? i_min : (i > i_max ? i_max : i);
}

led_strip_handle_t configure_led(int n) {
  int gpio = -1;
  switch (n) {
    case 0:
      gpio = 13;
      break;
    case 1:
      gpio = 12;
      break;
    case 2:
      gpio = 27;
      break;
    case 3:
      gpio = 26;
      break;
    case 4:
      gpio = 25;
      break;
    case 5:
      gpio = 33;
      break;
    case 6:
      gpio = 4;
      break;
    case 7:
      gpio = 16;
      break;
    case 8:
      gpio = 17;
      break;
    case 9:
      gpio = 5;
      break;
    default:
      break;
  }

  // LED strip general initialization, according to your led board design
  led_strip_config_t strip_config = {
    .strip_gpio_num = gpio,                 // The GPIO that connected to the LED strip's data line
    .max_leds       = LED_STRIP_NUM_LEDS_PER_STRIP,// The number of LEDs in the strip,
#if LED_STRIP_HAS_GRBW
    .led_pixel_format = LED_PIXEL_FORMAT_GRBW,// Pixel format of your LED strip
    .led_model        = LED_MODEL_SK6812,     // LED strip model
#else
    .led_pixel_format = LED_PIXEL_FORMAT_GRB,      // Pixel format of your LED strip
    .led_model        = LED_MODEL_WS2812,          // LED strip model
#endif
    .flags.invert_out = false,// whether to invert the output signal
  };

  // LED Strip object handle
  led_strip_handle_t led_strip;
  if (n < 8) {
    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
      .rmt_channel = 0,
#else
      .clk_src       = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
      .resolution_hz = LED_STRIP_RMT_RES_HZ,// RMT counter clock frequency
      .flags.with_dma =
      false,// DMA feature is available on ESP target like ESP32-S3
#endif
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
  } else if (n < 10) {
    // LED strip backend configuration: SPI
    led_strip_spi_config_t spi_config = {
      .clk_src        = SPI_CLK_SRC_DEFAULT,
      .spi_bus        = (n == 8) ? SPI2_HOST : SPI3_HOST,
      .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with SPI backend");
  } else {
    ESP_LOGE(TAG, "Invalid LED strip number");
    return NULL;
  }
  return led_strip;
}

void send_packet(int sock, struct sockaddr_storage* dest_addr, void* message, int message_len) {
  char tx_buffer[256];
  if (message_len > sizeof(tx_buffer)) {
    ESP_LOGE(TAG, "Message too long to send");
    return;
  }
  memcpy(tx_buffer, message, message_len);
  int err = sendto(sock, tx_buffer, message_len, 0,
                   (struct sockaddr*) dest_addr, sizeof(struct sockaddr_storage));
  if (err < 0) {
    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
  }
}

static void led_strips_update(void* arg)
{
  const uint32_t totalLEDs = LED_STRIP_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP;
  bool bHoldsSemaphore = xSemaphoreTake(semaphoreDisplay, 1);
  // check if we can read the next frame
  bool bCanRead = bHoldsSemaphore && displayState.readIndex < displayState.writeIndex;
  uint8_t color[4] = {};
  const uint64_t readIdx = displayState.readIndex % RingBufferLength;
  for (uint32_t i = 0; i < totalLEDs && bCanRead; i++) {
    const uint32_t stripIndex = i / LED_STRIP_NUM_LEDS_PER_STRIP;
    const uint32_t ledIndex   = i % LED_STRIP_NUM_LEDS_PER_STRIP;
    const uint32_t ledState = ledsBuffer[readIdx][stripIndex][ledIndex];
    uint8_t maxBrightness = displayState.maxBrightness;
    if (!(displayState.stripesEnable & (1 << stripIndex))) {
      continue;
    }
    scale_and_clamp_u32(ledState, maxBrightness, color);
#if LED_STRIP_HAS_GRBW
    ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(led_strips[stripIndex], ledIndex,
                                              color[0], color[1], color[2],
                                              color[3]));
#else
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strips[stripIndex], ledIndex,
                                        color[0], color[1], color[2]));
#endif
  }

  if (bCanRead) {
    displayState.readIndex = (displayState.readIndex + 1);
  }

  if (bHoldsSemaphore) {
    if (displayState.debugFlags & DebugShowBufferPosition) {
      // show read and write index in 2 the first 2 rows
      for (uint16_t stripIndex = 0; stripIndex < LED_STRIP_NUM_STRIPS; stripIndex++) {
        if (displayState.readIndex == stripIndex) {
          ESP_ERROR_CHECK(led_strip_set_pixel(led_strips[stripIndex], 0, 255, 0, 0));
        }
        if (displayState.writeIndex == stripIndex) {
          ESP_ERROR_CHECK(led_strip_set_pixel(led_strips[stripIndex], 1, 255, 0, 0));
        }
      }
    }
    xSemaphoreGive(semaphoreDisplay);
  }

  /* Refresh all strips */
  for (int stripIndex = 0; stripIndex < LED_STRIP_NUM_STRIPS; stripIndex++) {
    if (!(displayState.stripesEnable & (1 << stripIndex))) {
      continue;
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strips[stripIndex]));
  }

  if (bCanRead) {
    /* send heartbeat */
    if (displayState.heartbeatIntervalFrames > 0 
          && displayState.numFrames % displayState.heartbeatIntervalFrames == 0
          && g_udp_master_address.s2_len) {
      struct packet_heartbeat_t heartbeat = {
        .header = {
            .packetType = PKT_TYPE_HEARTBEAT,
            .len        = sizeof(struct heartbeat_message_t),
        },
        .message = {
            .frameId = displayState.numFrames,
            .fps     = displayState.frameRate,
            .heartbeatIntervalFrames = displayState.heartbeatIntervalFrames,
        },
      };
      if (g_socket_udp != -1) {
        send_packet(g_socket_udp, &g_udp_master_address, &heartbeat, sizeof(heartbeat));
      }
    }
    displayState.numFrames++;
  } else {
    if (displayState.numFrames > 0)
      displayState.numBufferUnderrun++;
  }
  displayState.numCallbacks++;
}

void led_display_task() {
  ESP_LOGI(TAG, "Starting LED display task");
  for (int i = 0; i < LED_STRIP_NUM_STRIPS; i++) {
    led_strips[i] = configure_led(i);
    if (led_strips[i] == NULL) {
      ESP_LOGE(TAG, "Failed to configure LED strip %d", i);
      return;
    }
    ESP_ERROR_CHECK(led_strip_clear(led_strips[i]));
  }

  const esp_timer_create_args_t periodic_timer_args = {
    .callback = &led_strips_update,
    .arg = NULL,
    /* name is optional, but may help identify the timer when debugging */
    .name = "periodic",
    // use ISR dispatch method to call the timer callback
    .dispatch_method = ESP_TIMER_TASK,
    .skip_unhandled_events = true,
  };
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000 / displayState.frameRate));
  int64_t timeLastFPS_us = esp_timer_get_time();
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    int64_t numFrames = displayState.numFrames - displayState.numLastFrames;
    int64_t numCallbacks = displayState.numCallbacks - displayState.numLastCallbacks;
    int64_t timeSince_us = esp_timer_get_time() - timeLastFPS_us;
    // print FPS stats every 5 seconds
    if (timeSince_us > 5 * 1e6) {
      float fps_actual = numFrames / (((float) timeSince_us) / 1e6);
      float fps_callbacks = numCallbacks / (((float) timeSince_us) / 1e6);
      ESP_LOGI(TAG, "Frames %lld - Dropped %lld - FPS: %.2f - CALLBACK: %.2f", 
                displayState.numFrames, displayState.numBufferUnderrun,
                fps_actual, fps_callbacks);
      timeLastFPS_us = esp_timer_get_time();
      displayState.numLastFrames = displayState.numFrames;
      displayState.numLastCallbacks = displayState.numCallbacks;
    }
  }
}

void handle_packet(char* rx_buffer, int len, int sock,
                   struct sockaddr_storage* source_addr) {
  struct packet_hdr_t* hdr = (struct packet_hdr_t*) rx_buffer;
  if (hdr->packetType == PKT_TYPE_HEARTBEAT) {
    // struct packet_heartbeat_t *pkt = (struct packet_heartbeat_t *) rx_buffer;
    // ESP_LOGI(TAG, "Received heartbeat packet");
    struct packet_heartbeat_t response = {
      .header = {
          .packetType = PKT_TYPE_HEARTBEAT,
          .len        = sizeof(struct heartbeat_message_t),
      },
      .message = {
          .frameId = displayState.numFrames,
          .fps     = displayState.frameRate,
          .heartbeatIntervalFrames = displayState.heartbeatIntervalFrames,
      },
    };
    send_packet(sock, source_addr, &response, sizeof(response));
  } else if (hdr->packetType == PKT_TYPE_CONFIG) {
    struct packet_config_t* pkt = (struct packet_config_t*) rx_buffer;
    if (pkt->message.cfgId >= CFG_NUM_CONFIGS) {
      ESP_LOGI(TAG, "Invalid config ID %lu", pkt->message.cfgId);
      return;
    }
    enum RGBConfigType cfgId = pkt->message.cfgId;
    switch (cfgId) {
      case CFG_ID_MAX_BRIGHTNESS:
        displayState.maxBrightness = clamp_u32(pkt->message.value, 0, 255);
        ESP_LOGI(TAG, "Set max brightness to %u", displayState.maxBrightness);
        break;
      case CFG_ID_FRAME_RATE:
        if (periodic_timer) {
          ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
        }
        displayState.frameRate = clamp_u32(pkt->message.value, 1, 10000);
        if (xSemaphoreTake(semaphoreDisplay, 20)) {
          reset_display_buffer_position();
          xSemaphoreGive(semaphoreDisplay);
        }
        ESP_LOGI(TAG, "Set frame rate to %u", displayState.frameRate);
        if (periodic_timer) {
          ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000 / displayState.frameRate));
        }
        break;
      case CFG_ID_STRIPES_ENABLE:
        displayState.stripesEnable = pkt->message.value;
        ESP_LOGI(TAG, "Set stripes enable to %lu", displayState.stripesEnable);
        break;
      case CFG_ID_ENABLE_DEBUG:
        displayState.debugFlags = pkt->message.value;
        ESP_LOGI(TAG, "Set debug flags to %u", displayState.debugFlags);
        break;
      case CFG_ID_HEARTBEAT_INTERVAL_FRAMES:
        displayState.heartbeatIntervalFrames = pkt->message.value;
        ESP_LOGI(TAG, "Set heartbeat interval to %lu", displayState.heartbeatIntervalFrames);
        memcpy(&g_udp_master_address, source_addr, sizeof(struct sockaddr_storage));
        break;
      default:
        break;
    }
  } else if (hdr->packetType == PKT_TYPE_LED_FRAME) {
    struct packet_led_frame_t* pkt = (struct packet_led_frame_t*) rx_buffer;
    if (pkt->message.frameOffset + pkt->message.frameSize >
        LED_STRIP_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP) {
      ESP_LOGE(TAG, "Invalid frame offset %lu and size %lu",
               pkt->message.frameOffset, pkt->message.frameSize);
      return;
    }
    // also check received data length
    int expectedSize = sizeof(struct packet_hdr_t) +
                       sizeof(struct led_frame_message_t) +
                       pkt->message.frameSize * 4;
    if (len != expectedSize) {
      ESP_LOGE(TAG, "Invalid frame data length %d. Expected %d", len, expectedSize);
      return;
    }

    if (!xSemaphoreTake(semaphoreDisplay, (TickType_t) 10)) {
      return;
    }
    uint32_t* pData = (uint32_t*) pkt->message.data;
    uint64_t writeIdx = displayState.writeIndex % RingBufferLength;
    for (uint32_t frameIdx = 0; frameIdx < pkt->message.frameSize; frameIdx++) {
      const uint32_t ledIndex      = pkt->message.frameOffset + frameIdx;
      const uint32_t stripIndex    = ledIndex / LED_STRIP_NUM_LEDS_PER_STRIP;
      const uint32_t ledStripIndex = ledIndex % LED_STRIP_NUM_LEDS_PER_STRIP;
      ledsBuffer[writeIdx][stripIndex][ledStripIndex] = pData[frameIdx];
    }
    displayState.writeIndex = (displayState.writeIndex + 1);
    xSemaphoreGive(semaphoreDisplay);
  }
}


char udp_rx_buffer[4096];
int s_socket = -1;
static void udp_server_task(void* pvParameters) {
  char addr_str[128];
#if CONFIG_USE_IPV6
  int addr_family = AF_INET6;
#else
  int addr_family = AF_INET;
#endif
  int ip_protocol = 0;
  struct sockaddr_in6 dest_addr;
  while (1) {
    xSemaphoreTake(semaphoreSocketStart, portMAX_DELAY);
    bzero(&dest_addr, sizeof(dest_addr));
    if (addr_family == AF_INET) {
      struct sockaddr_in* dest_addr_ip4 = (struct sockaddr_in*) &dest_addr;
      dest_addr_ip4->sin_addr.s_addr    = htonl(INADDR_ANY);
      dest_addr_ip4->sin_family         = AF_INET;
      dest_addr_ip4->sin_port           = htons(LED_UDP_LISTEN_PORT);
      ip_protocol                       = IPPROTO_IP;
    } else if (addr_family == AF_INET6) {
      dest_addr.sin6_family = AF_INET6;
      dest_addr.sin6_port   = htons(LED_UDP_LISTEN_PORT);
      ip_protocol           = IPPROTO_IPV6;
    }
    if (s_socket != -1) {
      ESP_LOGE(TAG, "Shutting down socket and restarting...");
      shutdown(s_socket, 0);
      close(s_socket);
      s_socket = -1;
    }

    s_socket = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (s_socket < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }

#if defined(CONFIG_USE_IPV4) && defined(CONFIG_USE_IPV6)
    if (addr_family == AF_INET6) {
      // Note that by default IPV6 binds to both protocols, it is must be
      // disabled if both protocols used at the same time (used in CI)
      int opt = 1;
      setsockopt(s_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
      // setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
    }
#endif

    // Set infinite timeout
    struct timeval timeout;
    bzero(&timeout, sizeof(timeout));
    timeout.tv_sec  = 0;
    timeout.tv_usec = 0;
    setsockopt(s_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    int err = bind(s_socket, (struct sockaddr*) &dest_addr, sizeof(dest_addr));
    if (err < 0) {
      ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    g_socket_udp = s_socket;

    struct sockaddr_storage source_addr;// Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);

    while (s_socket >= 0) {
      bzero(&source_addr, sizeof(source_addr));
      ssize_t len = recvfrom(s_socket, udp_rx_buffer, sizeof(udp_rx_buffer), 0,
                         (struct sockaddr*) &source_addr, &socklen);
      if (len < 0) {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        break;
      }
      // Get the sender's ip address as string
      if (source_addr.ss_family == PF_INET) {
        inet_ntoa_r(((struct sockaddr_in*) &source_addr)->sin_addr, addr_str,
                    sizeof(addr_str) - 1);
      } else if (source_addr.ss_family == PF_INET6) {
        inet6_ntoa_r(((struct sockaddr_in6*) &source_addr)->sin6_addr, addr_str,
                     sizeof(addr_str) - 1);
      }

      /* ESP_LOGI(TAG, "Received %d bytes from %s", len, addr_str);
      // print first 8 bytes as hex
      for (int i = 0; i < 8 && i < len; i++) {
        ESP_LOGI(TAG, "rx_buffer[%d] = 0x%02x", i, rx_buffer[i]);
      } */

      handle_packet(udp_rx_buffer, len, s_socket, &source_addr);
    }
  }
}

void app_main(void) {
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  semaphoreDisplay = xSemaphoreCreateBinaryStatic(&s_semaphoreDisplay);
  semaphoreNetwork = xSemaphoreCreateCounting(2, 0);
  semaphoreSocketStart = xSemaphoreCreateBinary();
  if (semaphoreNetwork == NULL || semaphoreDisplay == NULL || semaphoreSocketStart == NULL) {
    ESP_LOGE(TAG, "Failed to create semaphore");
    return;
  }
  xSemaphoreGive(semaphoreDisplay);
  reset_display_state();
  xTaskCreatePinnedToCore(led_display_task, "led_display", 4096, NULL, 6, NULL, 1);
  ESP_ERROR_CHECK(network_connect());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  xTaskCreate(udp_server_task, "udp_server", 4096, (void*) 0, 5, NULL);
  while (1) {
    xSemaphoreTake(semaphoreNetwork, portMAX_DELAY);
    // wait for a second notification (IP6), but only for 5 seconds
    xSemaphoreTake(semaphoreNetwork, pdMS_TO_TICKS(5000));
    // log network restart
    ESP_LOGI(TAG, "Network restart...");
    if (s_socket != -1) {
      shutdown(s_socket, 0);
      close(s_socket);
      s_socket = -1;
    }
    if (xSemaphoreTake(semaphoreDisplay, portMAX_DELAY)) {
      reset_display_buffer_position();
      xSemaphoreGive(semaphoreDisplay);
    }
    xSemaphoreGive(semaphoreSocketStart);
    ESP_LOGI(TAG, "Network restarted");
  }
}
