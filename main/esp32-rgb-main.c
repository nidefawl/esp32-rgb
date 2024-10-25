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
#include "protocol_examples_common.h"
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

#define CONFIG_USE_IPV4
#define CONFIG_USE_BUFFER_SYNC

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)
#define LED_STRIP_HAS_GRBW 0
#define LED_STRIP_NUM_STRIPS 10
#define LED_STRIP_LED_NUMBERS 30

#define LED_UDP_LISTEN_PORT 54321

static const char* TAG                              = "RGB_MAIN";
led_strip_handle_t led_strips[LED_STRIP_NUM_STRIPS] = {};
uint64_t frameId                                    = 0;

struct LEDState {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t w;
};

struct DisplayState {
  uint8_t maxBrightness;
  uint16_t frameRate;
  uint32_t stripesEnable;
  uint8_t readIndex;
  uint8_t writeIndex;
};
enum : uint8_t {
  RingBufferLength = 8,
};
volatile struct DisplayState displayState;
volatile struct LEDState ledsBuffer[RingBufferLength][LED_STRIP_NUM_STRIPS][LED_STRIP_LED_NUMBERS];
SemaphoreHandle_t xSemaphore = NULL;
StaticSemaphore_t xSemaphoreBuffer;

void reset_display_state() {
  memset(ledsBuffer, 0, sizeof(ledsBuffer));
  displayState.frameRate     = 30;
  displayState.maxBrightness = 127;
  displayState.stripesEnable = ~0;
  displayState.readIndex     = 0;
  displayState.writeIndex    = 0;
}

uint8_t scale_and_clamp(uint8_t color, uint8_t maxBrightness) {
  uint32_t scaled = color;
  scaled *= maxBrightness;
  scaled /= 255;
  if (scaled > 255)
    return 255;
  return scaled;
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
    .strip_gpio_num = gpio,           // The GPIO that connected to the LED strip's data line
    .max_leds = LED_STRIP_LED_NUMBERS,// The number of LEDs in the strip,
#if HAS_GRBW
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
      .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
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

void app_led_main_loop() {
  ESP_LOGI(TAG, "Starting LED main loop");
  for (int i = 0; i < LED_STRIP_NUM_STRIPS; i++) {
    led_strips[i] = configure_led(i);
    if (led_strips[i] == NULL) {
      ESP_LOGE(TAG, "Failed to configure LED strip %d", i);
      return;
    }
    ESP_ERROR_CHECK(led_strip_clear(led_strips[i]));
  }
  const uint32_t totalLEDs = LED_STRIP_NUM_STRIPS * LED_STRIP_LED_NUMBERS;
  int64_t timeLastFPS_us = esp_timer_get_time();
  int64_t numFrames = 0;
  while (1) {
    // check if we can read the next frame
    bool bHoldsSemaphore = true;
#ifdef CONFIG_USE_BUFFER_SYNC
    bHoldsSemaphore = xSemaphoreTake( xSemaphore, 1 );
#endif
    const uint8_t readIndex = displayState.readIndex;
    bool bCanRead = bHoldsSemaphore && readIndex != displayState.writeIndex;
    numFrames++;
    for (int i = 0; i < totalLEDs && bCanRead; i++) {
      const int stripIndex = i / LED_STRIP_LED_NUMBERS;
      const int ledIndex   = i % LED_STRIP_LED_NUMBERS;
      volatile const struct LEDState* ledState = &ledsBuffer[readIndex][stripIndex][ledIndex];
      uint8_t maxBrightness = displayState.maxBrightness;
      if (!(displayState.stripesEnable & (1 << stripIndex))) {
        continue;
      }
      uint8_t color[4] = {
        scale_and_clamp(ledState->r, maxBrightness),
        scale_and_clamp(ledState->g, maxBrightness),
        scale_and_clamp(ledState->b, maxBrightness),
        scale_and_clamp(ledState->w, maxBrightness),
      };
#if HAS_GRBW
      ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(led_strips[stripIndex], ledIndex,
                                              color[0], color[1], color[2],
                                              color[3]));
#else
      ESP_ERROR_CHECK(led_strip_set_pixel(led_strips[stripIndex], ledIndex,
                                          color[0], color[1], color[2]));
#endif
    }
#ifdef CONFIG_USE_BUFFER_SYNC
    if (bHoldsSemaphore) {
      xSemaphoreGive( xSemaphore );
    }
#endif

    /* Refresh all strips */
    for (int stripIndex = 0; stripIndex < LED_STRIP_NUM_STRIPS; stripIndex++) {
      if (!(displayState.stripesEnable & (1 << stripIndex))) {
        continue;
      }
      ESP_ERROR_CHECK(led_strip_refresh(led_strips[stripIndex]));
    }
    const uint32_t frameDelay_ms = 1000 / displayState.frameRate;

    int delay = pdMS_TO_TICKS(frameDelay_ms * 3 / 4);
    if (delay > 0) {
      vTaskDelay(delay);
    }
    if (bCanRead) {
      displayState.readIndex = (readIndex + 1) % RingBufferLength;
    }

    // print FPS stats every 10 seconds
    int64_t timeSince_us = numFrames < 10 ? 0 : esp_timer_get_time() - timeLastFPS_us;
    if (timeSince_us > 10 * 1e6) {
      float fps = numFrames / (((float)timeSince_us) / 1e6);
      ESP_LOGI(TAG, "FPS: %.2f", fps);
      timeLastFPS_us = esp_timer_get_time();
      numFrames   = 0;
    }

    // increment frame ID
    frameId++;
  }
}

void send_packet(int sock, struct sockaddr_storage* dest_addr, void* message, int message_len) {
  char tx_buffer[256];
  memcpy(tx_buffer + sizeof(struct packet_hdr_t), message, message_len);
  int err = sendto(sock, tx_buffer, sizeof(struct packet_hdr_t) + message_len, 0,
             (struct sockaddr*) dest_addr, sizeof(struct sockaddr_storage));
  if (err < 0) {
    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
  }
}

void handle_packet(char* rx_buffer, int len, int sock,
                   struct sockaddr_storage* source_addr) {
  struct packet_hdr_t* hdr = (struct packet_hdr_t*) rx_buffer;
  if (hdr->packetType == PKT_TYPE_HEARTBEAT) {
    // struct packet_heartbeat_t *pkt = (struct packet_heartbeat_t *) rx_buffer;
    // ESP_LOGI(TAG, "Received heartbeat packet");
    struct packet_heartbeat_t response = {
      .header = { .packetType = PKT_TYPE_HEARTBEAT, .len = sizeof(struct heartbeat_message_t), },
      .message = { .frameId = frameId, },
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
        displayState.frameRate = clamp_u32(pkt->message.value, 1, 10000);
        ESP_LOGI(TAG, "Set frame rate to %u", displayState.frameRate);
        break;
      case CFG_ID_STRIPES_ENABLE:
        displayState.stripesEnable = pkt->message.value;
        ESP_LOGI(TAG, "Set stripes enable to %lu", displayState.stripesEnable);
        break;
      default:
        break;
    }
  } else if (hdr->packetType == PKT_TYPE_LED_FRAME) {
    struct packet_led_frame_t* pkt = (struct packet_led_frame_t*) rx_buffer;
    if (pkt->message.frameOffset + pkt->message.frameSize >
        LED_STRIP_NUM_STRIPS * LED_STRIP_LED_NUMBERS) {
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

#ifdef CONFIG_USE_BUFFER_SYNC
    if(!xSemaphoreTake( xSemaphore, ( TickType_t ) 10 )) {
      return;
    }
    // vTaskSuspendAll();
#endif
    uint8_t* pData = pkt->message.data;
    int writeIndex = displayState.writeIndex;
    for (int i = 0; i < pkt->message.frameSize; i++) {
      const int ledIndex        = pkt->message.frameOffset + i;
      const int stripIndex      = ledIndex / LED_STRIP_LED_NUMBERS;
      const int ledStripIndex   = ledIndex % LED_STRIP_LED_NUMBERS;
      volatile struct LEDState* ledState = &ledsBuffer[writeIndex][stripIndex][ledStripIndex];
      ledState->r = pData[4 * i];
      ledState->g = pData[4 * i + 1];
      ledState->b = pData[4 * i + 2];
      ledState->w = pData[4 * i + 3];
    }
    displayState.writeIndex = (writeIndex + 1) % RingBufferLength;
#ifdef CONFIG_USE_BUFFER_SYNC
    xSemaphoreGive( xSemaphore );
    // xTaskResumeAll();
#endif
  }
}


char udp_rx_buffer[4096];
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
    if (addr_family == AF_INET) {
      struct sockaddr_in* dest_addr_ip4 = (struct sockaddr_in*) &dest_addr;
      dest_addr_ip4->sin_addr.s_addr    = htonl(INADDR_ANY);
      dest_addr_ip4->sin_family         = AF_INET;
      dest_addr_ip4->sin_port           = htons(LED_UDP_LISTEN_PORT);
      ip_protocol                       = IPPROTO_IP;
    } else if (addr_family == AF_INET6) {
      bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
      dest_addr.sin6_family = AF_INET6;
      dest_addr.sin6_port   = htons(LED_UDP_LISTEN_PORT);
      ip_protocol           = IPPROTO_IPV6;
    }

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }

#if defined(CONFIG_USE_IPV4) && defined(CONFIG_USE_IPV6)
    if (addr_family == AF_INET6) {
      // Note that by default IPV6 binds to both protocols, it is must be
      // disabled if both protocols used at the same time (used in CI)
      int opt = 1;
      setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
      setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
    }
#endif

    // Set infinite timeout
    struct timeval timeout;
    timeout.tv_sec  = 0;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    int err = bind(sock, (struct sockaddr*) &dest_addr, sizeof(dest_addr));
    if (err < 0) {
      ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }

    struct sockaddr_storage source_addr;// Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);

    while (1) {
      int len = recvfrom(sock, udp_rx_buffer, sizeof(udp_rx_buffer), 0,
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

      handle_packet(udp_rx_buffer, len, sock, &source_addr);
    }

    if (sock != -1) {
      ESP_LOGE(TAG, "Shutting down socket and restarting...");
      shutdown(sock, 0);
      close(sock);
    }
  }
  vTaskDelete(NULL);
}

void app_main(void) {
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  xSemaphore = xSemaphoreCreateBinaryStatic( &xSemaphoreBuffer );
  if (xSemaphore == NULL) {
    ESP_LOGE(TAG, "Failed to create semaphore");
    return;
  }
	xSemaphoreGive( xSemaphore );
  reset_display_state();
  xTaskCreatePinnedToCore(app_led_main_loop, "led_main_loop", 4096, NULL, 6, NULL, 1);
  ESP_ERROR_CHECK(example_connect());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  xTaskCreate(udp_server_task, "udp_server", 4096, (void*) 0, 5, NULL);
}
