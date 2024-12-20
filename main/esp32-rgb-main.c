#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "led_strip_spi.h"
#include "led_strip_types.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "portmacro.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <esp_http_server.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "rgb-network-types.h"
#include <lwip/netdb.h>
#include "driver/ledc.h"
#include "nvs_wifi_connect.h"
#include "mdns.h"
#include "lwip/apps/netbiosns.h"

#define CONFIG_USE_IPV4 1
#define CONFIG_USE_IPV6 1
#define CONFIG_MAX_LEDS 30

#define CONFIG_USE_ASYNC_RMT 1
#ifdef CONFIG_IDF_TARGET_ESP32S3
#define MAX_RMT_CHANNELS 4
#else
#define MAX_RMT_CHANNELS 8
#endif

#define LED_UDP_LISTEN_PORT 54321
#define DISPLAY_CONFIG_VERSION 2
#define DISPLAY_NETWORK_HOSTNAME_DEFAULT "esp32-rgb"
#define DISPLAY_NETWORK_WIFI_AP_SSID_DEFAULT "ESP32-RGB"
#define DISPLAY_NETWORK_WIFI_AP_PASS_DEFAULT "rgb1234"
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

static led_strip_handle_t led_strips[CONFIG_MAX_STRIPS] = {};
static struct DisplayState displayState;
static display_config_t displayConfig;
static display_led_strip_config_t displayLedStripConfig[CONFIG_MAX_STRIPS];
static display_network_config_t displayNetworkConfig;
static bool bDisplayConfigChanged = false;
static volatile uint32_t ledsBuffer[RingBufferLength][CONFIG_MAX_STRIPS][CONFIG_MAX_LEDS];
static esp_timer_handle_t periodic_timer;
static SemaphoreHandle_t semaphoreDisplay = NULL;
static StaticSemaphore_t s_semaphoreDisplay;
static SemaphoreHandle_t semaphoreNetwork = NULL;
static SemaphoreHandle_t semaphoreNetworkMasterAddress = NULL;
static SemaphoreHandle_t semaphoreSocketStart = NULL;
static int g_socket_udp = -1;
struct sockaddr_storage g_udp_master_address = {};
static int64_t g_timeLastFrame_us = 0;

static void display_state_reset() {
  memset((void*)ledsBuffer, 0, sizeof(ledsBuffer));
  memset(&displayState, 0, sizeof(displayState));
  displayState.readIndex  = 0;
  displayState.writeIndex = 0;
}

static void display_buffer_position_reset() {
  displayState.readIndex  = 0;
  displayState.writeIndex = 0;
  displayState.numFrames = 0;
  displayState.numCallbacks = 0;
  displayState.numBufferUnderrun = 0;
  displayState.numLastFrames = 0;
  displayState.numLastCallbacks = 0;
  displayState.numHeartbeats = 0;
}

static bool display_config_validate(display_config_t* config) {
  if (config->version < DISPLAY_CONFIG_VERSION) {
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

static bool display_strip_config_validate(display_led_strip_config_all_t* config) {
  if (config->version < DISPLAY_CONFIG_VERSION) {
    ESP_LOGE(TAG, "Invalid version");
    return false;
  }
  if (config->magic != DISPLAY_CONFIG_MAGIC) {
    ESP_LOGE(TAG, "Invalid magic");
    return false;
  }
  for (int i = 0; i < CONFIG_MAX_STRIPS; i++) {
    if (config->configs[i].gpio < -1) {
      ESP_LOGE(TAG, "Invalid GPIO for strip %d", i);
      return false;
    }
    if (config->configs[i].ledType > LED_TYPE_SK6812) {
      ESP_LOGE(TAG, "Invalid LED type for strip %d", i);
      return false;
    }
    if (config->configs[i].dataTypeId > RGB_DATA_TYPE_RGBW) {
      ESP_LOGE(TAG, "Invalid data type for strip %d", i);
      return false;
    }
    if (config->configs[i].ledCount < 1) {
      ESP_LOGE(TAG, "Invalid LED count for strip %d", i);
      return false;
    }
  }
  return true;
}

static bool display_network_config_validate(display_network_config_t* config) {
  if (config->version < DISPLAY_CONFIG_VERSION) {
    ESP_LOGE(TAG, "Invalid version");
    return false;
  }
  if (config->magic != DISPLAY_CONFIG_MAGIC) {
    ESP_LOGE(TAG, "Invalid magic");
    return false;
  }
  // check for empty hostname
  if (strnlen(config->hostname, sizeof(config->hostname)) < 1) {
    ESP_LOGE(TAG, "Invalid hostname");
    return false;
  }
  if (config->wifiMode >= NUM_NETWORK_WIFI_MODES) {
    ESP_LOGE(TAG, "Invalid wifi mode");
    return false;
  }
  enum WifiMode wifiMode = (enum WifiMode)config->wifiMode;
  // check for empty SSID
  if (wifiMode == NETWORK_WIFI_MODE_AP && strnlen(config->apWifiSsid, sizeof(config->apWifiSsid)) < 1) {
    ESP_LOGE(TAG, "Invalid AP SSID");
    return false;
  }
  if (wifiMode == NETWORK_WIFI_MODE_STA && strnlen(config->staWifiSsid, sizeof(config->staWifiSsid)) < 1) {
    ESP_LOGE(TAG, "Invalid STA SSID");
    return false;
  }
  return true;
}

static void display_config_reset(display_config_t* config) {
  config->version = DISPLAY_CONFIG_VERSION;
  config->magic = DISPLAY_CONFIG_MAGIC;
  config->rmtClockResolution_hz = 10000000; // 10MHz
  config->dimensionsWidth = 1;
  config->dimensionsHeight = 15;
  config->maxBrightness = 127;
  config->frameRate = 60;
  config->stripsEnable = 0xFFFF;
  config->debugFlags = DebugNone;
  config->heartbeatIntervalFrames = 8;
}

static void display_strip_config_reset(display_led_strip_config_all_t* config) {
  config->version = DISPLAY_CONFIG_VERSION;
  config->magic = DISPLAY_CONFIG_MAGIC;
  memset(&config->configs, 0, sizeof(display_led_strip_config_t) * CONFIG_MAX_STRIPS);
  for (int i = 0; i < CONFIG_MAX_STRIPS; i++) {
    config->configs[i].gpio = -1;
    config->configs[i].ledCount = 1;
    config->configs[i].ledType = LED_TYPE_WS2812B;
    config->configs[i].dataTypeId = RGB_DATA_TYPE_RGB;
  }
}

static void display_network_config_reset(display_network_config_t* config) {
  config->version = DISPLAY_CONFIG_VERSION;
  config->magic = DISPLAY_CONFIG_MAGIC;
  config->port = 54321;
  config->wifiMode = NETWORK_WIFI_MODE_AP;
  memset(&config->hostname, 0, sizeof(config->hostname));
  memset(&config->apWifiSsid, 0, sizeof(config->apWifiSsid));
  memset(&config->apWifiPassword, 0, sizeof(config->apWifiPassword));
  memset(&config->staWifiSsid, 0, sizeof(config->staWifiSsid));
  memset(&config->staWifiPassword, 0, sizeof(config->staWifiPassword));
  strncpy(config->hostname, DISPLAY_NETWORK_HOSTNAME_DEFAULT, sizeof(config->hostname));
  strncpy(config->apWifiSsid, DISPLAY_NETWORK_WIFI_AP_SSID_DEFAULT, sizeof(config->apWifiSsid));
  strncpy(config->apWifiPassword, DISPLAY_NETWORK_WIFI_AP_PASS_DEFAULT, sizeof(config->apWifiPassword));
}

static void log_display_config(display_config_t* config) {
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
}

static void log_display_strip_config(display_led_strip_config_all_t* config) {
  ESP_LOGI(TAG, "Version: %lu", config->version);
  ESP_LOGI(TAG, "Magic: %lu", config->magic);
  for (int i = 0; i < CONFIG_MAX_STRIPS; i++) {
    ESP_LOGI(TAG, "Strip %d: GPIO %d, LED count %d, LED type %d, Data type %d", i, config->configs[i].gpio, config->configs[i].ledCount, config->configs[i].ledType, config->configs[i].dataTypeId);
  }
}

static void display_strips_init();
static void display_config_set(display_config_t* config, bool bForceReset) {
  bool bChanged = memcmp(&displayConfig, config, sizeof(display_config_t)) != 0;
  log_display_config(config);
  if (bChanged || bForceReset) {
    if (periodic_timer) {
      esp_timer_stop(periodic_timer);
    }
    vTaskDelay(5);
    xSemaphoreTake(semaphoreDisplay, portMAX_DELAY);
    displayConfig = *config;
    display_strips_init();
    display_buffer_position_reset();
    xSemaphoreGive(semaphoreDisplay);
    if (periodic_timer) {
      ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000 / config->frameRate));
    }
    bDisplayConfigChanged = true;
  }
}
static void display_strip_config_set(display_led_strip_config_all_t* config, bool bForceReset) {
  bool bChanged = memcmp(&displayLedStripConfig[0], &config->configs[0], sizeof(display_led_strip_config_t) * CONFIG_MAX_STRIPS) != 0;
  log_display_strip_config(config);
  if (bChanged || bForceReset) {
    if (periodic_timer) {
      esp_timer_stop(periodic_timer);
    }
    vTaskDelay(5);
    for (int i = 0; i < CONFIG_MAX_STRIPS; i++) {
      displayLedStripConfig[i] = config->configs[i];
    }
    display_strips_init();
    display_buffer_position_reset();
    if (periodic_timer) {
      ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000 / displayConfig.frameRate));
    }
    bDisplayConfigChanged = true;
  }
}


static esp_err_t write_nvs_struct(const char* key, void* data, size_t len) {
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Write data
    err = nvs_set_blob(my_handle, key, data, len);
    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return err;
}
static esp_err_t read_nvs_struct(const char* key, void* data, size_t len) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)  {
      return err;
    }

    // Read the size of memory space required for data
    size_t stored_data_len = 0;
    err = nvs_get_blob(nvs_handle, key, NULL, &stored_data_len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
      return err;
    if (stored_data_len >= len) {
      // Read previously saved display_config_t if available
      uint32_t* read_buf = malloc(stored_data_len);
      err = nvs_get_blob(nvs_handle, key, read_buf, &stored_data_len);
      if (err == ESP_OK) {
        memcpy(data, read_buf, len);
      }
      free(read_buf);
    }
    nvs_close(nvs_handle);
    return err;
}

void write_strip_config(display_led_strip_config_all_t* config) {
  if (display_strip_config_validate(config)) {
    xSemaphoreTake(semaphoreDisplay, portMAX_DELAY);
    display_strip_config_set(config, true);
    write_nvs_struct("strip_config", config, sizeof(display_led_strip_config_all_t));
    xSemaphoreGive(semaphoreDisplay);
    ESP_LOGI(TAG, "Set LED strip config all");
  } else {
    ESP_LOGE(TAG, "Invalid LED strip config all");
  }
}

static void _log_nvs_error(esp_err_t err){
  switch (err) {
    case ESP_OK:
      break;
    case ESP_FAIL:
      ESP_LOGE(TAG, "Error: Failed. NVS corrupt?");
      break;
    case ESP_ERR_NVS_NOT_INITIALIZED:
      ESP_LOGE(TAG, "Error: NVS not initialized");
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      ESP_LOGE(TAG, "Error: NVS not found");
      break;
    case ESP_ERR_NVS_TYPE_MISMATCH:
      ESP_LOGE(TAG, "Error: NVS type mismatch");
      break;
    case ESP_ERR_NVS_READ_ONLY:
      ESP_LOGE(TAG, "Error: NVS read only");
      break;
    case ESP_ERR_NVS_NOT_ENOUGH_SPACE:
      ESP_LOGE(TAG, "Error: NVS not enough space");
      break;
    case ESP_ERR_NVS_INVALID_NAME:
      ESP_LOGE(TAG, "Error: NVS invalid name");
      break;
    case ESP_ERR_NVS_INVALID_HANDLE:
      ESP_LOGE(TAG, "Error: NVS invalid handle");
      break;
    case ESP_ERR_NVS_REMOVE_FAILED:
      ESP_LOGE(TAG, "Error: NVS remove failed");
      break;
    case ESP_ERR_NVS_KEY_TOO_LONG:
      ESP_LOGE(TAG, "Error: NVS key too long");
      break;
    case ESP_ERR_NVS_PAGE_FULL:
      ESP_LOGE(TAG, "Error: NVS page full");
      break;
    case ESP_ERR_NVS_INVALID_STATE:
      ESP_LOGE(TAG, "Error: NVS invalid state");
      break;
    case ESP_ERR_NVS_INVALID_LENGTH:
      ESP_LOGE(TAG, "Error: NVS invalid length");
      break;
    case ESP_ERR_NVS_NO_FREE_PAGES:
      ESP_LOGE(TAG, "Error: NVS no free pages");
      break;
    case ESP_ERR_NVS_VALUE_TOO_LONG:
      ESP_LOGE(TAG, "Error: NVS value too long");
      break;
    case ESP_ERR_NVS_PART_NOT_FOUND:
      ESP_LOGE(TAG, "Error: NVS part not found");
      break;
    default:
      // unknown error, print as hex
      ESP_LOGE(TAG, "Error: 0x%x", err);
      break;
  }
}
static void display_load_config_from_flash() {
  xSemaphoreTake(semaphoreDisplay, portMAX_DELAY);
  esp_err_t err;
  display_config_t config;
  if ((err = read_nvs_struct("display_config", &config, sizeof(display_config_t))) != ESP_OK 
      || !display_config_validate(&config)) {
    display_config_reset(&config);
    ESP_LOGW(TAG, "Invalid config in flash, resetting");
    esp_err_t err = write_nvs_struct("display_config", &config, sizeof(display_config_t));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error writing config to flash");
      _log_nvs_error(err);
    }
  } else {
    ESP_LOGI(TAG, "Loaded config from flash");
    log_display_config(&config);
  }
  display_network_config_t networkConfig;
  if ((err = read_nvs_struct("network_config", &networkConfig, sizeof(display_network_config_t))) != ESP_OK
      || !display_network_config_validate(&networkConfig)) {
    ESP_LOGW(TAG, "Invalid network config in flash, resetting");
    display_network_config_reset(&networkConfig);
    esp_err_t err = write_nvs_struct("network_config", &networkConfig, sizeof(display_network_config_t));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error writing network config to flash");
      _log_nvs_error(err);
    }
  } else {
    ESP_LOGI(TAG, "Loaded network config from flash");
  }
  displayConfig = config;
  displayNetworkConfig = networkConfig;
  display_led_strip_config_all_t ledStripConfigAll;
  if ((err = read_nvs_struct("strip_config", &ledStripConfigAll, sizeof(display_led_strip_config_all_t))) != ESP_OK 
    || !display_strip_config_validate(&ledStripConfigAll)) {
    display_strip_config_reset(&ledStripConfigAll);
    ESP_LOGW(TAG, "Invalid LED strip config in flash, resetting");
    write_strip_config(&ledStripConfigAll);
  } else {
    display_strip_config_set(&ledStripConfigAll, false);
    ESP_LOGI(TAG, "Loaded LED strip config from flash");
  }
  bDisplayConfigChanged = false;
  xSemaphoreGive(semaphoreDisplay);
}

static uint8_t scale_and_clamp(uint8_t color, uint8_t maxBrightness) {
  uint32_t scaled = color;
  scaled *= maxBrightness;
  scaled /= 255;
  if (scaled > 255)
    return 255;
  return scaled;
}


static void scale_and_clamp_u32(uint32_t color, uint8_t maxBrightness, uint8_t* out) {
  uint8_t w = (color >> 24) & 0xFF;
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;
  out[0]    = scale_and_clamp(r, maxBrightness);
  out[1]    = scale_and_clamp(g, maxBrightness);
  out[2]    = scale_and_clamp(b, maxBrightness);
  out[3]    = scale_and_clamp(w, maxBrightness);
}

static uint32_t clamp_u32(uint32_t i, uint32_t i_min, uint32_t i_max) {
  return i < i_min ? i_min : (i > i_max ? i_max : i);
}

static led_strip_handle_t display_led_strip_configure(uint32_t stripIndex, display_led_strip_config_t* stripConfig) {
  display_config_t* config = &displayConfig;
  if (stripIndex >= config->dimensionsWidth) {
    ESP_LOGE(TAG, "Invalid strip index");
    return NULL;
  }

  // LED strip general initialization
  led_strip_config_t strip_config = {
    .strip_gpio_num = stripConfig->gpio,        // The GPIO that connected to the LED strip's data line
    .max_leds       = config->dimensionsHeight, // Number of LEDs in your strip
    .led_pixel_format = stripConfig->dataTypeId == RGB_DATA_TYPE_RGB ? LED_PIXEL_FORMAT_GRB : LED_PIXEL_FORMAT_GRBW, // Pixel format of the strip
    .led_model        = stripConfig->ledType == LED_TYPE_WS2812B ? LED_MODEL_WS2812 : LED_MODEL_SK6812, // LED model of the strip
    .flags.invert_out = false,// whether to invert the output signal
  };


  led_strip_handle_t led_strip;
  if (stripIndex < MAX_RMT_CHANNELS) {
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
    ESP_LOGI(TAG, "Created LED strip object with RMT backend (GPIO %d) (PixelFormat %s)", stripConfig->gpio,
             stripConfig->dataTypeId ? "RGBW" : "RGB");
  } else if (stripIndex < CONFIG_MAX_STRIPS) {
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

static void send_udp_packet(int sock, struct sockaddr_storage* dest_addr, void* message, int message_len) {
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
  xSemaphoreTake(semaphoreDisplay, portMAX_DELAY);

  // check if we can read the next frame
  const bool bCanRead = displayState.readIndex < displayState.writeIndex;
  uint8_t color[4] = {};
  const uint32_t dimW = displayConfig.dimensionsWidth;
  const uint32_t dimH = displayConfig.dimensionsHeight;
  const uint32_t configStripsEnabled = displayConfig.stripsEnable;
  const uint64_t readIdx = displayState.readIndex % RingBufferLength;
  for (uint32_t stripIndex = 0; bCanRead && stripIndex < dimW && stripIndex < CONFIG_MAX_STRIPS; ++stripIndex) {
    led_strip_handle_t ledStrip = led_strips[stripIndex];
    if (ledStrip == NULL || !display_strip_is_enabled(configStripsEnabled, stripIndex)) {
      continue;
    }
    const display_led_strip_config_t *stripConfig = &displayLedStripConfig[stripIndex];
    for (uint32_t ledIndex = 0; ledIndex < dimH && ledIndex < CONFIG_MAX_LEDS; ++ledIndex) {
      const uint32_t ledState = ledsBuffer[readIdx][stripIndex][ledIndex];
      uint8_t maxBrightness = displayConfig.maxBrightness;
      scale_and_clamp_u32(ledState, maxBrightness, color);
      if (stripConfig->dataTypeId == RGB_DATA_TYPE_RGBW) {
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
    if (led_strips[stripIndex] == NULL || !display_strip_is_enabled(configStripsEnabled, stripIndex)) {
      continue;
    }
    ESP_ERROR_CHECK(led_strip_refresh_begin(led_strips[stripIndex]));
  }

  for (int stripIndex = 0; stripIndex < dimW; stripIndex++) {
    if (led_strips[stripIndex] == NULL || !display_strip_is_enabled(configStripsEnabled, stripIndex)) {
      continue;
    }
    ESP_ERROR_CHECK(led_strip_refresh_end(led_strips[stripIndex]));
  }
#else
  for (int stripIndex = 0; stripIndex < dimW; stripIndex++) {
    if (led_strips[stripIndex] == NULL || !display_strip_is_enabled(configStripsEnabled, stripIndex)) {
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
            send_udp_packet(socket, &g_udp_master_address, &heartbeat, sizeof(heartbeat));
            int heartbeatsPerSecond = displayConfig.frameRate / displayConfig.heartbeatIntervalFrames;
            if (displayState.bRuntimeStatsEnabled && displayState.numCallbacks % heartbeatsPerSecond == 0) {
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
              send_udp_packet(socket, &g_udp_master_address, &stats, sizeof(stats));
            }
            displayState.numHeartbeats++;
        }
        xSemaphoreGive(semaphoreNetworkMasterAddress);
      }
    }
  }

  if (bCanRead) {
    displayState.numFrames++;
    g_timeLastFrame_us = esp_timer_get_time();
  } else {
    if (displayState.numFrames > 0) {
      displayState.numBufferUnderrun++;
      // if we haven't received a frame in 5 seconds, reset the master address so broadcasts are sent again
      if (esp_timer_get_time() - g_timeLastFrame_us > 5e6) {
        if (xSemaphoreTake(semaphoreNetworkMasterAddress, portMAX_DELAY) == pdTRUE) {
          g_timeLastFrame_us = esp_timer_get_time();
          bool bHadMaster = g_udp_master_address.s2_len > 0;
          memset(&g_udp_master_address, 0, sizeof(g_udp_master_address));
          xSemaphoreGive(semaphoreNetworkMasterAddress);
          if (bHadMaster) {
            ESP_LOGI(TAG, "Resetting master address");
          }
        }
      }
    }
  }
  displayState.numCallbacks++;
  xSemaphoreGive(semaphoreDisplay);
}

static void display_strips_init() {
  ESP_LOGI(TAG, "Initializing Dimensions: %lux%lu", displayConfig.dimensionsWidth, displayConfig.dimensionsHeight);
  for (int i = 0; i < CONFIG_MAX_STRIPS; i++) {
    if (led_strips[i] != NULL) {
      ESP_ERROR_CHECK(led_strip_del(led_strips[i]));
      led_strips[i] = NULL;
    }
  }

  for (int i = 0; i < displayConfig.dimensionsWidth && i < CONFIG_MAX_STRIPS; i++) {
    display_led_strip_config_t* stripConfig = &displayLedStripConfig[i];
    if (stripConfig->gpio < 0) {
      continue;
    }
    led_strips[i] = display_led_strip_configure(i, stripConfig);
    if (led_strips[i] == NULL) {
      ESP_LOGE(TAG, "Failed to configure LED strip %d", i);
      return;
    }
  }
  for (int i = 0; i < displayConfig.dimensionsWidth && i < CONFIG_MAX_STRIPS; i++) {
    if (led_strips[i] != NULL) {
      ESP_ERROR_CHECK(led_strip_clear(led_strips[i]));
    }
  }
}

static void led_display_task() {
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

    // Write config updates to nvs
    if (bDisplayConfigChanged) {
      xSemaphoreTake(semaphoreDisplay, portMAX_DELAY);
      bDisplayConfigChanged = false;
      write_nvs_struct("display_config", &displayConfig, sizeof(display_config_t));
      ESP_LOGI(TAG, "wrote display config to flash");
      xSemaphoreGive(semaphoreDisplay);
    }

    // Broadcast packet handling
    int socket = g_socket_udp;
    if (socket != -1) {
      if (xSemaphoreTake(semaphoreNetworkMasterAddress, portMAX_DELAY) == pdTRUE) {
        if (!g_udp_master_address.s2_len) {
          struct packet_broadcast_t pkt = {
            .header = {
              .packetType = PKT_TYPE_ANNOUNCE_BROADCAST,
              .len        = sizeof(int),
            },
            .message = 0,
          };
          struct sockaddr_storage udp_broadcast_address = {};
          struct sockaddr_in* udp_broadcast_address_in = (struct sockaddr_in*)&udp_broadcast_address;
          udp_broadcast_address_in->sin_family = AF_INET;
          udp_broadcast_address_in->sin_addr.s_addr = INADDR_BROADCAST;
          udp_broadcast_address_in->sin_port = htons(LED_UDP_LISTEN_PORT);
          send_udp_packet(socket, &udp_broadcast_address, &pkt, sizeof(pkt));
        }
        xSemaphoreGive(semaphoreNetworkMasterAddress);
      }
    }
  }
}

void handle_config_single_packet(struct packet_config_single_t* pkt) {
  if (pkt->message.cfgId >= CFG_NUM_CONFIGS) {
    ESP_LOGI(TAG, "Invalid config ID %lu", pkt->message.cfgId);
    return;
  }
  enum RGBConfigType cfgId = pkt->message.cfgId;
  xSemaphoreTake(semaphoreDisplay, portMAX_DELAY);
  display_config_t config = displayConfig;
  xSemaphoreGive(semaphoreDisplay);
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
}

void handle_config_all_packet(display_config_t* config) {
  if (display_config_validate(config)) {
    display_config_set(config, true);
    ESP_LOGI(TAG, "Set config all");
  } else {
    ESP_LOGE(TAG, "Invalid config all");
  }
}

struct packet_strip_config_all_t read_strip_config() {
  struct packet_strip_config_all_t readStripConfig = {
    .header = {
        .packetType = PKT_TYPE_READ_STRIP_CONFIG_ALL,
        .len        = sizeof(display_led_strip_config_all_t),
    },
    .message = {},
  };
  for (int i = 0; i < CONFIG_MAX_STRIPS; i++) {
    readStripConfig.message.configs[i] = displayLedStripConfig[i];
  }
  return readStripConfig;
}

void write_display_config(display_network_config_t* config) {
  if (display_network_config_validate(config)) {
    displayNetworkConfig = *config;
    write_nvs_struct("network_config", config, sizeof(display_network_config_t));
    ESP_LOGI(TAG, "Set network config");
  } else {
    ESP_LOGE(TAG, "Invalid network config");
  }
}
struct packet_network_config_t read_network_config() {
  struct packet_network_config_t config = {
    .header = {
      .packetType = PKT_TYPE_READ_NETWORK_CONFIG,
      .len        = sizeof(display_network_config_t),
    },
    .message = displayNetworkConfig,
  };
  return config;
};

struct packet_heartbeat_t handle_heartbeat() {
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
  return heartbeat;
}

void handle_request_heartbeat_packet(struct request_heartbeat_message_t* req, struct sockaddr_storage* source_addr, char addr_str[128]) {
  if (xSemaphoreTake(semaphoreNetworkMasterAddress, portMAX_DELAY) == pdTRUE) {
    displayState.bHeartbeatEnabled = req->flags & REQUEST_HEARTBEAT_ENABLE;
    displayState.bRuntimeStatsEnabled = req->flags & REQUEST_RUNTIME_STATS_ENABLE;
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
}

struct packet_config_all_t read_display_config() {
  xSemaphoreTake(semaphoreDisplay, portMAX_DELAY);
  struct packet_config_all_t config = {
    .header = {
        .packetType = PKT_TYPE_CONFIG_ALL,
        .len        = sizeof(display_config_t),
    },
    .message = displayConfig,
  };
  xSemaphoreGive(semaphoreDisplay);
  return config;
}

void display_handle_packet(uint8_t* rx_buffer, int len, int sock,
                   struct sockaddr_storage* source_addr, char addr_str[128]) {
  struct packet_hdr_t* hdr = (struct packet_hdr_t*) rx_buffer;
  if (hdr->packetType == PKT_TYPE_HEARTBEAT) {
    struct packet_heartbeat_t heartbeat = handle_heartbeat();
    send_udp_packet(sock, source_addr, &heartbeat, sizeof(heartbeat));
  } else if (hdr->packetType == PKT_TYPE_REQUEST_HEARTBEAT) {
    struct packet_request_heartbeat_t* pkt = (struct packet_request_heartbeat_t*) rx_buffer;
    handle_request_heartbeat_packet(&pkt->message, source_addr, addr_str);
  } else if (hdr->packetType == PKT_TYPE_READ_NETWORK_CONFIG) {
    struct packet_network_config_t readConfig = read_network_config();
    send_udp_packet(sock, source_addr, &readConfig, sizeof(readConfig));
  } else if (hdr->packetType == PKT_TYPE_WRITE_NETWORK_CONFIG) {
    struct packet_network_config_t* pkt = (struct packet_network_config_t*) rx_buffer;
    write_display_config(&pkt->message);
  } else if (hdr->packetType == PKT_TYPE_READ_CONFIG) {
    struct packet_config_all_t readConfig = read_display_config();
    send_udp_packet(sock, source_addr, &readConfig, sizeof(readConfig));
  } else if (hdr->packetType == PKT_TYPE_READ_STRIP_CONFIG_ALL) {
    struct packet_strip_config_all_t readStripConfig = read_strip_config();
    send_udp_packet(sock, source_addr, &readStripConfig, sizeof(readStripConfig));
  } else if (hdr->packetType == PKT_TYPE_WRITE_STRIP_CONFIG_ALL) {
    struct packet_strip_config_all_t* pkt = (struct packet_strip_config_all_t*) rx_buffer;
    write_strip_config(&pkt->message);
  } else if (hdr->packetType == PKT_TYPE_CONFIG_ALL) {
    struct packet_config_all_t* pkt = (struct packet_config_all_t*) rx_buffer;
    handle_config_all_packet(&pkt->message);
  } else if (hdr->packetType == PKT_TYPE_CONFIG_SINGLE) {
    struct packet_config_single_t* pkt = (struct packet_config_single_t*) rx_buffer;
    handle_config_single_packet(pkt);
  } else if (hdr->packetType == PKT_TYPE_LED_FRAME) {
      struct packet_led_frame_t* pkt = (struct packet_led_frame_t*) rx_buffer;
      if (pkt->message.frameOffset + pkt->message.frameSize >
          displayConfig.dimensionsWidth * displayConfig.dimensionsHeight) {
        ESP_LOGE(TAG, "Invalid frame offset %lu and size %lu",
                pkt->message.frameOffset, pkt->message.frameSize);
        return;
      }
      // check received data length
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
  }
}
void send_websocket_packet(httpd_req_t *req, httpd_ws_frame_t* ws_pkt, void* message, int len) {
  ws_pkt->payload = message;
  ws_pkt->len = len;
  ws_pkt->type = HTTPD_WS_TYPE_BINARY;
  int ret = httpd_ws_send_frame(req, ws_pkt);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
  }
}
bool display_handle_websocket_packet(httpd_req_t *req, httpd_ws_frame_t* ws_pkt) {
  struct packet_hdr_t* hdr = (struct packet_hdr_t*) ws_pkt->payload;
  if (hdr->packetType == PKT_TYPE_READ_NETWORK_CONFIG) {
    struct packet_network_config_t readConfig = read_network_config();
    send_websocket_packet(req, ws_pkt, &readConfig, sizeof(readConfig));
    return true;
  } else if (hdr->packetType == PKT_TYPE_WRITE_NETWORK_CONFIG) {
    struct packet_network_config_t* pkt = (struct packet_network_config_t*) ws_pkt->payload;
    write_display_config(&pkt->message);
    return false;
  } else if (hdr->packetType == PKT_TYPE_READ_CONFIG) {
    struct packet_config_all_t readConfig = read_display_config();
    send_websocket_packet(req, ws_pkt, &readConfig, sizeof(readConfig));
    return true;
  } else if (hdr->packetType == PKT_TYPE_READ_STRIP_CONFIG_ALL) {
    struct packet_strip_config_all_t readStripConfig = read_strip_config();
    send_websocket_packet(req, ws_pkt, &readStripConfig, sizeof(readStripConfig));
    return true;
  } else if (hdr->packetType == PKT_TYPE_WRITE_STRIP_CONFIG_ALL) {
    struct packet_strip_config_all_t* pkt = (struct packet_strip_config_all_t*) ws_pkt->payload;
    write_strip_config(&pkt->message);
    return false;
  } else if (hdr->packetType == PKT_TYPE_CONFIG_ALL) {
    struct packet_config_all_t* pkt = (struct packet_config_all_t*) ws_pkt->payload;
    handle_config_all_packet(&pkt->message);
    return false;
  } else if (hdr->packetType == PKT_TYPE_CONFIG_SINGLE) {
    struct packet_config_single_t* pkt = (struct packet_config_single_t*) ws_pkt->payload;
    handle_config_single_packet(pkt);
    return false;
  }
  return false;
}



static uint8_t udp_rx_buffer[4096];
static int s_socket = -1;
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
      g_socket_udp = -1;
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
    int broadcastValue = 1;
    err = setsockopt(s_socket, SOL_SOCKET, SO_BROADCAST, &broadcastValue, sizeof(broadcastValue));
    if (err < 0) {
      ESP_LOGE(TAG, "Socket unable to set broadcast: errno %d", errno);
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

      display_handle_packet(udp_rx_buffer, len, s_socket, &source_addr, addr_str);
    }
  }
}

static void initialise_mdns(void)
{
    mdns_init();
    mdns_hostname_set(displayNetworkConfig.hostname);
    mdns_instance_name_set("ESP32 RGB LED Controller");

    mdns_txt_item_t serviceTxtData[] = {
        {"board", "esp32"},
        {"path", "/"}};

    ESP_ERROR_CHECK(mdns_service_add(TAG, "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}
esp_err_t example_register_uri_handler(httpd_handle_t server);

static void connect_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    if (semaphoreNetwork) {
      xSemaphoreGive(semaphoreNetwork);
    }
}
static void connect_handler_ipv6(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    if (semaphoreNetwork) {
      xSemaphoreGive(semaphoreNetwork);
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
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, &connect_handler_ipv6, NULL));
  
  initialise_mdns();
  netbiosns_init();
  netbiosns_set_name(displayNetworkConfig.hostname);
  struct network_init_cfg_t cfg = {
    .wifiMode = displayNetworkConfig.wifiMode == NETWORK_WIFI_MODE_AP ? WIFI_MODE_AP : WIFI_MODE_STA,
  };
  strncpy(cfg.apWifiSsid, displayNetworkConfig.apWifiSsid, sizeof(cfg.apWifiSsid));
  strncpy(cfg.apWifiPassword, displayNetworkConfig.apWifiPassword, sizeof(cfg.apWifiPassword));
  strncpy(cfg.staWifiSsid, displayNetworkConfig.staWifiSsid, sizeof(cfg.staWifiSsid));
  strncpy(cfg.staWifiPassword, displayNetworkConfig.staWifiPassword, sizeof(cfg.staWifiPassword));
  nvs_wifi_connect(&cfg);
  nvs_wifi_connect_start_http_server(NVS_WIFI_CONNECT_MODE_RESTART_ESP32, example_register_uri_handler); // run server
  xTaskCreate(udp_server_task, "udp_server", 4096*2, (void*) 0, 5, NULL);
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
