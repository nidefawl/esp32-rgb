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
#include "nvs.h"
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
#define CONFIG_MAX_STRIPS 10
#define CONFIG_MAX_LEDS 30
#define CONFIG_USE_ASYNC_RMT 1

#define LED_UDP_LISTEN_PORT 54321

#define STORAGE_NAMESPACE "esp32-rgb"
static const char* TAG = "RGB_MAIN";

enum DebugFlags : uint8_t {
  DebugNone = 0,
};

struct DisplayState {
  bool bHeartbeatEnabled;
  bool bRuntimeStatsEnabled;
  // ring buffer positions
  volatile uint64_t readIndex;
  volatile uint64_t writeIndex;
  // runtime stats
  int64_t numCallbacks;
  int64_t numFrames;
  int64_t numBufferUnderrun;
  int64_t numLastCallbacks;
  int64_t numLastFrames;
  int64_t numHeartbeats;
  float fps_actual;
  float fps_callbacks;
};

enum : uint16_t {
  RingBufferLength = 64,
};

led_strip_handle_t led_strips[CONFIG_MAX_STRIPS] = {};
struct DisplayState displayState;
display_config_t displayConfig;
bool bDisplayConfigChanged = false;
volatile uint32_t ledsBuffer[RingBufferLength][CONFIG_MAX_STRIPS][CONFIG_MAX_LEDS];
esp_timer_handle_t periodic_timer;
SemaphoreHandle_t semaphoreDisplay = NULL;
StaticSemaphore_t s_semaphoreDisplay;
SemaphoreHandle_t semaphoreNetwork = NULL;
SemaphoreHandle_t semaphoreNetworkMasterAddress = NULL;
SemaphoreHandle_t semaphoreSocketStart = NULL;
int g_socket_udp = -1;
struct sockaddr_storage g_udp_master_address = {};

void display_state_reset() {
  memset((void*)ledsBuffer, 0, sizeof(ledsBuffer));
  memset(&displayState, 0, sizeof(displayState));
  displayState.readIndex  = 0;
  displayState.writeIndex = 0;
}

void display_buffer_position_reset() {
  displayState.readIndex  = 0;
  displayState.writeIndex = 0;
  displayState.numFrames = 0;
  displayState.numCallbacks = 0;
  displayState.numBufferUnderrun = 0;
  displayState.numLastFrames = 0;
  displayState.numLastCallbacks = 0;
  displayState.numHeartbeats = 0;
}
#define DISPLAY_CONFIG_VERSION 1
bool display_config_validate(display_config_t* config) {
  if (config->version < 1) {
    ESP_LOGE(TAG, "Invalid version");
    return false;
  }
  if (config->magic != DISPLAY_CONFIG_MAGIC) {
    ESP_LOGE(TAG, "Invalid magic");
    return false;
  }
  if (config->dimensionsWidth < 1 || config->dimensionsHeight < 1
      || config->dimensionsWidth > CONFIG_MAX_STRIPS || config->dimensionsHeight > CONFIG_MAX_LEDS) {
    ESP_LOGE(TAG, "Invalid display dimensions");
    return false;
  }
  if (config->frameRate < 1) {
    ESP_LOGE(TAG, "Invalid frame rate");
    return false;
  }
  return true;
}

void display_config_reset(display_config_t* config) {
  config->version = DISPLAY_CONFIG_VERSION;
  config->magic = DISPLAY_CONFIG_MAGIC;
  config->rmtClockResolution_hz = 10000000; // 10MHz
  config->dimensionsWidth = 10;
  config->dimensionsHeight = 30;
  config->maxBrightness = 127;
  config->frameRate = 60;
  config->stripsEnable = 0xFFFF;
  config->debugFlags = DebugNone;
  config->heartbeatIntervalFrames = 8;
}
void display_strips_init();
void log_display_config(display_config_t* config) {
  ESP_LOGI(TAG, "Version: %lu", config->version);
  ESP_LOGI(TAG, "Magic: %lu", config->magic);
  ESP_LOGI(TAG, "ClockResolution: %lu -> %lu", displayConfig.rmtClockResolution_hz, config->rmtClockResolution_hz);
  ESP_LOGI(TAG, "Width: %lu -> %lu", displayConfig.dimensionsWidth, config->dimensionsWidth);
  ESP_LOGI(TAG, "Height: %lu -> %lu", displayConfig.dimensionsHeight, config->dimensionsHeight);
  ESP_LOGI(TAG, "FrameRate: %u -> %u", displayConfig.frameRate, config->frameRate);
  ESP_LOGI(TAG, "Heartbeat: %lu -> %lu", displayConfig.heartbeatIntervalFrames, config->heartbeatIntervalFrames);
  ESP_LOGI(TAG, "Brightness: %u -> %u", displayConfig.maxBrightness, config->maxBrightness);
  ESP_LOGI(TAG, "Strips: %lu -> %lu", displayConfig.stripsEnable, config->stripsEnable);
  ESP_LOGI(TAG, "Debug: %lu -> %lu", displayConfig.debugFlags, config->debugFlags);
  ESP_LOGI(TAG, "RGBW: %s", config->isRGBW ? "true" : "false");
}
void display_config_set(display_config_t* config, bool bForceReset) {
  bool bChanged = memcmp(&displayConfig, config, sizeof(display_config_t)) != 0;
  // bool bDimensChanged = displayConfig.dimensionsWidth != config->dimensionsWidth
  //                       || displayConfig.dimensionsHeight != config->dimensionsHeight;
  // bool bRateChanged = displayConfig.frameRate != config->frameRate;
  log_display_config(config);
  if (bChanged || bForceReset) {
    ESP_LOGI(TAG, "Stopped periodic timer0");
    if (periodic_timer) {
      esp_timer_stop(periodic_timer);
    }
    ESP_LOGI(TAG, "Stopped periodic timer1");
    vTaskDelay(5);
    ESP_LOGI(TAG, "Stopped periodic timer2");
    displayConfig = *config;
    display_strips_init();
    display_buffer_position_reset();
    if (periodic_timer) {
      ESP_LOGI(TAG, "Start periodic timer");
      ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000 / config->frameRate));
    }
    bDisplayConfigChanged = true;
  }
}

bool read_display_config(display_config_t* config) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;

    // Read the size of memory space required for display_config_t
    size_t display_config_len = 0;
    err = nvs_get_blob(nvs_handle, "display_config", NULL, &display_config_len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    if (display_config_len >= sizeof(display_config_t)) {
      // Read previously saved display_config_t if available
      uint32_t* read_buf = malloc(sizeof(display_config_t));
      err = nvs_get_blob(nvs_handle, "display_config", read_buf, &display_config_len);
      if (err == ESP_OK) {
          *config = *(display_config_t*) read_buf;
      }
      free(read_buf);
      return err == ESP_OK;
    }
    return false;
}

bool write_display_config(display_config_t* config) {
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Write display_config_t
    err = nvs_set_blob(my_handle, "display_config", config, sizeof(display_config_t));
    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return true;
}

void display_load_config_from_flash() {
  display_config_t config;
  if (!read_display_config(&config) || !display_config_validate(&config)) {
    display_config_reset(&config);
    ESP_LOGI(TAG, "Invalid config in flash, resetting");
    write_display_config(&config);
  } else {
    ESP_LOGI(TAG, "Loaded config from flash");
    log_display_config(&config);
  }
  displayConfig = config;
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

led_strip_handle_t display_led_strip_configure(uint32_t stripIndex) {
  display_config_t* config = &displayConfig;
  if (stripIndex >= config->dimensionsWidth) {
    ESP_LOGE(TAG, "Invalid strip index");
    return NULL;
  }
  int32_t gpio = -1;
  switch (stripIndex) {
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
    .strip_gpio_num = gpio,                     // The GPIO that connected to the LED strip's data line
    .max_leds       = config->dimensionsHeight, // Number of LEDs in your strip
    .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
    .led_model        = LED_MODEL_WS2812,     // LED strip model
    .flags.invert_out = false,// whether to invert the output signal
  };
  
  if (config->isRGBW) {
    strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRBW;
    strip_config.led_model        = LED_MODEL_SK6812;
  }

  // LED Strip object handle
  led_strip_handle_t led_strip;
  if (stripIndex < 8) {
    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
      .rmt_channel = 0,
#else
      .clk_src       = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
      .resolution_hz = config->rmtClockResolution_hz,// RMT counter clock frequency
      .flags.with_dma =
      false,// DMA feature is available on ESP target like ESP32-S3
#endif
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend (GPIO %ld) (PixelFormat %s)", gpio,
             config->isRGBW ? "GRBW" : "GRB");
  } else if (stripIndex < 10) {
    // LED strip backend configuration: SPI
    led_strip_spi_config_t spi_config = {
      .clk_src        = SPI_CLK_SRC_DEFAULT,
      .spi_bus        = (stripIndex == 8) ? SPI2_HOST : SPI3_HOST,
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

static inline bool display_strip_is_enabled(uint32_t configStripsEnabled, int stripIndex) {
  return configStripsEnabled & (1 << stripIndex);
}

static inline uint32_t display_get_buffer_fillstate() {
  int16_t writePos = displayState.writeIndex % RingBufferLength;
  int16_t readPos = displayState.readIndex % RingBufferLength;
  return writePos >= readPos ? writePos - readPos : RingBufferLength + writePos - readPos;
}

static void display_strips_update(void* arg)
{
  // check if we can read the next frame
  const bool bCanRead = displayState.readIndex < displayState.writeIndex;
  // log positions and frame
  /* if (displayConfig.debugFlags&2) {
    ESP_LOGI(TAG, "timer: Read %llu - Write %llu - CanRead %d", displayState.readIndex, displayState.writeIndex, bCanRead);
  } */
  uint8_t color[4] = {};
  const uint32_t dimW = displayConfig.dimensionsWidth;
  const uint32_t dimH = displayConfig.dimensionsHeight;
  const uint32_t configStripsEnabled = displayConfig.stripsEnable;
  const uint64_t readIdx = displayState.readIndex % RingBufferLength;
  for (uint32_t stripIndex = 0; bCanRead && stripIndex < dimW; ++stripIndex) {
    if (!display_strip_is_enabled(configStripsEnabled, stripIndex)) {
      continue;
    }
    for (uint32_t ledIndex = 0; ledIndex < dimH; ++ledIndex) {
      const uint32_t ledState = ledsBuffer[readIdx][stripIndex][ledIndex];
      uint8_t maxBrightness = displayConfig.maxBrightness;
      scale_and_clamp_u32(ledState, maxBrightness, color);
      if (displayConfig.isRGBW) {
        ESP_ERROR_CHECK(led_strip_set_pixel_rgbw(led_strips[stripIndex], ledIndex,
                                                color[0], color[1], color[2],
                                                color[3]));
      } else {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strips[stripIndex], ledIndex,
                                            color[0], color[1], color[2]));
      }
    }
  }

  if (bCanRead) {
    displayState.readIndex = (displayState.readIndex + 1);
  }

#ifdef CONFIG_USE_ASYNC_RMT
  /* Refresh all strips */
  for (int stripIndex = 0; stripIndex < dimW; stripIndex++) {
    if (!display_strip_is_enabled(configStripsEnabled, stripIndex)) {
      continue;
    }
    ESP_ERROR_CHECK(led_strip_refresh_begin(led_strips[stripIndex]));
  }

  for (int stripIndex = 0; stripIndex < dimW; stripIndex++) {
    if (!display_strip_is_enabled(configStripsEnabled, stripIndex)) {
      continue;
    }
    ESP_ERROR_CHECK(led_strip_refresh_end(led_strips[stripIndex]));
  }
#else
  for (int stripIndex = 0; stripIndex < dimW; stripIndex++) {
    if (!display_strip_is_enabled(configStripsEnabled, stripIndex)) {
      continue;
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strips[stripIndex]));
  }
#endif

  /* send heartbeat */
  if (displayState.bHeartbeatEnabled && displayConfig.heartbeatIntervalFrames > 0 
      && displayState.numCallbacks % displayConfig.heartbeatIntervalFrames == 0) {
    struct packet_heartbeat_t heartbeat = {
      .header = {
          .packetType = PKT_TYPE_HEARTBEAT,
          .len        = sizeof(struct heartbeat_message_t),
      },
      .message = {
          .frameId = displayState.numCallbacks,
          .bufferFillLevel = display_get_buffer_fillstate(),
      },
    };
    int socket = g_socket_udp;
    if (socket != -1) {
      if (xSemaphoreTake(semaphoreNetworkMasterAddress, portMAX_DELAY) == pdTRUE) {
        if (g_udp_master_address.s2_len) {
            send_packet(socket, &g_udp_master_address, &heartbeat, sizeof(heartbeat));
            int heartbeatsPerSecond = displayConfig.frameRate / displayConfig.heartbeatIntervalFrames;
            if (displayState.bRuntimeStatsEnabled && displayState.numCallbacks % heartbeatsPerSecond == 0) {
              // send stats every second
              struct packet_runtime_stats_t stats = {
                .header = {
                  .packetType = PKT_TYPE_RUNTIME_STATS,
                  .len = sizeof(struct runtime_stats_t),
                },
                .message = {
                  .readIndex = displayState.readIndex,
                  .writeIndex = displayState.writeIndex,
                  .numCallbacks = displayState.numCallbacks,
                  .numFrames = displayState.numFrames,
                  .numBufferUnderrun = displayState.numBufferUnderrun,
                  .numHeartbeats = displayState.numHeartbeats,
                  .fps_actual = displayState.fps_actual,
                  .fps_callbacks = displayState.fps_callbacks,
                },
              };
              send_packet(socket, &g_udp_master_address, &stats, sizeof(stats));
            }
            displayState.numHeartbeats++;
        }
        xSemaphoreGive(semaphoreNetworkMasterAddress);
      }
    }
  }

  if (bCanRead) {
    displayState.numFrames++;
  } else {
    if (displayState.numFrames > 0)
      displayState.numBufferUnderrun++;
  }
  displayState.numCallbacks++;
}

void display_strips_init() {
  ESP_LOGI(TAG, "Initializing Dimensions: %lux%lu", displayConfig.dimensionsWidth, displayConfig.dimensionsHeight);
  for (int i = 0; i < CONFIG_MAX_STRIPS; i++) {
    if (led_strips[i] != NULL) {
      ESP_ERROR_CHECK(led_strip_del(led_strips[i]));
      led_strips[i] = NULL;
    }
  }

  for (int i = 0; i < displayConfig.dimensionsWidth; i++) {
    led_strips[i] = display_led_strip_configure(i);
    if (led_strips[i] == NULL) {
      ESP_LOGE(TAG, "Failed to configure LED strip %d", i);
      return;
    }
  }
  for (int i = 0; i < displayConfig.dimensionsWidth; i++) {
    ESP_ERROR_CHECK(led_strip_clear(led_strips[i]));
  }
}

void led_display_task() {
  const esp_timer_create_args_t periodic_timer_args = {
    .callback = &display_strips_update,
    .arg = NULL,
    /* name is optional, but may help identify the timer when debugging */
    .name = "periodic",
    // use ISR dispatch method to call the timer callback
    .dispatch_method = ESP_TIMER_TASK,
    .skip_unhandled_events = true,
  };
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  display_strips_init();
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000 / displayConfig.frameRate));
  int64_t timeLastFPS_us = esp_timer_get_time();
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    int64_t numFrames = displayState.numFrames - displayState.numLastFrames;
    int64_t numCallbacks = displayState.numCallbacks - displayState.numLastCallbacks;
    int64_t timeSince_us = esp_timer_get_time() - timeLastFPS_us;
    // print FPS stats every 5 seconds
    if (timeSince_us > 5 * 1e6) {
      displayState.fps_actual = numFrames / (((float) timeSince_us) / 1e6);
      displayState.fps_callbacks = numCallbacks / (((float) timeSince_us) / 1e6);
      uint32_t readWriteDiff = display_get_buffer_fillstate();
      ESP_LOGI(TAG, "Frames %lld - Dropped %lld - FPS: %.2f - CALLBACK: %.2f - Buffer: %lu (rd %llu/wr %llu) - Heartbeats: %lld",
                displayState.numFrames, displayState.numBufferUnderrun,
                displayState.fps_actual, displayState.fps_callbacks, readWriteDiff, displayState.readIndex, displayState.writeIndex, displayState.numHeartbeats);
      timeLastFPS_us = esp_timer_get_time();
      displayState.numLastFrames = displayState.numFrames;
      displayState.numLastCallbacks = displayState.numCallbacks;
    }
    if (bDisplayConfigChanged) {
      bDisplayConfigChanged = false;
      write_display_config(&displayConfig);
      ESP_LOGI(TAG, "wrote display config to flash");
    }
  }
}

void handle_packet(char* rx_buffer, int len, int sock,
                   struct sockaddr_storage* source_addr, char addr_str[128]) {
  struct packet_hdr_t* hdr = (struct packet_hdr_t*) rx_buffer;
  if (hdr->packetType == PKT_TYPE_HEARTBEAT) {
    struct packet_heartbeat_t heartbeat = {
      .header = {
          .packetType = PKT_TYPE_HEARTBEAT,
          .len        = sizeof(struct heartbeat_message_t),
      },
      .message = {
          .frameId = displayState.numCallbacks,
          .bufferFillLevel = display_get_buffer_fillstate(),
      },
    };
    send_packet(sock, source_addr, &heartbeat, sizeof(heartbeat));
  } else if (hdr->packetType == PKT_TYPE_REQUEST_HEARTBEAT) {
    struct packet_request_heartbeat_t* pkt = (struct packet_request_heartbeat_t*) rx_buffer;
    if (xSemaphoreTake(semaphoreNetworkMasterAddress, portMAX_DELAY) == pdTRUE) {
      displayState.bHeartbeatEnabled = pkt->message.flags & REQUEST_HEARTBEAT_ENABLE;
      displayState.bRuntimeStatsEnabled = pkt->message.flags & REQUEST_RUNTIME_STATS_ENABLE;
      if (!displayState.bHeartbeatEnabled) {
        memset(&g_udp_master_address, 0, sizeof(struct sockaddr_storage));
      } else {
        memcpy(&g_udp_master_address, source_addr, sizeof(struct sockaddr_storage));
      }
      xSemaphoreGive(semaphoreNetworkMasterAddress);
      if (!displayState.bHeartbeatEnabled) {
        ESP_LOGI(TAG, "Disabled heartbeat and stats");
      } else {
        ESP_LOGI(TAG, "Enabled heartbeat. Destination host: %s", addr_str);
      }
    }
  } else if (hdr->packetType == PKT_TYPE_READ_CONFIG) {
    struct packet_config_all_t readConfig = {
      .header = {
          .packetType = PKT_TYPE_CONFIG_ALL,
          .len        = sizeof(struct config_message_t),
      },
      .message = displayConfig,
    };
    send_packet(sock, source_addr, &readConfig, sizeof(readConfig));
  } else if (hdr->packetType == PKT_TYPE_CONFIG_ALL) {
    struct packet_config_all_t* pkt = (struct packet_config_all_t*) rx_buffer;
    if (display_config_validate(&pkt->message)) {
      display_config_set(&pkt->message, true);
      ESP_LOGI(TAG, "Set config all");
    } else {
      ESP_LOGE(TAG, "Invalid config all");
    }
  } else if (hdr->packetType == PKT_TYPE_CONFIG_SINGLE) {
    struct packet_config_single_t* pkt = (struct packet_config_single_t*) rx_buffer;
    if (pkt->message.cfgId >= CFG_NUM_CONFIGS) {
      ESP_LOGI(TAG, "Invalid config ID %lu", pkt->message.cfgId);
      return;
    }
    enum RGBConfigType cfgId = pkt->message.cfgId;
    display_config_t config = displayConfig;
    switch (cfgId) {
      case CFG_ID_DIMENSIONS_WIDTH:
        config.dimensionsWidth = clamp_u32(pkt->message.value, 1, CONFIG_MAX_STRIPS);
        break;
      case CFG_ID_DIMENSIONS_HEIGHT:
        config.dimensionsHeight = clamp_u32(pkt->message.value, 1, CONFIG_MAX_LEDS);
        break;
      case CFG_ID_MAX_BRIGHTNESS:
        config.maxBrightness = clamp_u32(pkt->message.value, 0, 255);
        break;
      case CFG_ID_FRAME_RATE:
        config.frameRate = clamp_u32(pkt->message.value, 1, 10000);
        break;
      case CFG_ID_STRIPS_ENABLE:
        config.stripsEnable = pkt->message.value;
        break;
      case CFG_ID_ENABLE_DEBUG:
        config.debugFlags = pkt->message.value;
        break;
      case CFG_ID_HEARTBEAT_INTERVAL_FRAMES:
        config.heartbeatIntervalFrames = pkt->message.value;
        break;
      default:
        break;
    }
    if (display_config_validate(&config)) {
      display_config_set(&config, false);
    }
  } else if (hdr->packetType == PKT_TYPE_LED_FRAME) {
      struct packet_led_frame_t* pkt = (struct packet_led_frame_t*) rx_buffer;
      if (pkt->message.frameOffset + pkt->message.frameSize >
          displayConfig.dimensionsWidth * displayConfig.dimensionsHeight) {
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
      uint32_t* pData = (uint32_t*) pkt->message.data;
      uint64_t writeIdx = displayState.writeIndex % RingBufferLength;
      for (uint32_t frameIdx = 0; frameIdx < pkt->message.frameSize; frameIdx++) {
        const uint32_t ledIndex      = pkt->message.frameOffset + frameIdx;
        const uint32_t stripIndex    = ledIndex / displayConfig.dimensionsHeight;
        const uint32_t ledStripIndex = ledIndex % displayConfig.dimensionsHeight;
        ledsBuffer[writeIdx][stripIndex][ledStripIndex] = pData[frameIdx];
      }
      displayState.writeIndex = (displayState.writeIndex + 1);
      /* if (displayConfig.debugFlags&2) {
        ESP_LOGI(TAG, "Network: Read %llu - Write %llu", displayState.readIndex, displayState.writeIndex);
      } */
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

      handle_packet(udp_rx_buffer, len, s_socket, &source_addr, addr_str);
    }
  }
}

void app_main(void) {
  display_config_reset(&displayConfig);
  esp_err_t err = nvs_flash_init();
  ESP_ERROR_CHECK( err );
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  semaphoreDisplay = xSemaphoreCreateBinaryStatic(&s_semaphoreDisplay);
  semaphoreNetwork = xSemaphoreCreateCounting(2, 0);
  semaphoreSocketStart = xSemaphoreCreateBinary();
  semaphoreNetworkMasterAddress = xSemaphoreCreateBinary();
  if (semaphoreNetwork == NULL 
      || semaphoreDisplay == NULL 
      || semaphoreSocketStart == NULL
      || semaphoreNetworkMasterAddress == NULL
  ) {
    ESP_LOGE(TAG, "Failed to create semaphore");
    return;
  }
  xSemaphoreGive(semaphoreDisplay);
  xSemaphoreGive(semaphoreNetworkMasterAddress);
  display_load_config_from_flash();
  display_state_reset();
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
      display_buffer_position_reset();
      xSemaphoreGive(semaphoreDisplay);
    }
    xSemaphoreGive(semaphoreSocketStart);
    ESP_LOGI(TAG, "Network restarted");
  }
}
