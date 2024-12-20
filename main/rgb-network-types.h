#pragma once
#include <stdint.h>
#define DISPLAY_CONFIG_MAGIC 0x12345678
#define CONFIG_MAX_STRIPS 10
#define CONFIG_ERR_MSG_BUF_SIZE 100-4

enum RGBNetworkPacketType : uint8_t {
  PKT_TYPE_HEARTBEAT              = 0,
  PKT_TYPE_CONFIG_ALL             = 1,
  PKT_TYPE_CONFIG_SINGLE          = 2,
  PKT_TYPE_LED_FRAME              = 3,
  PKT_TYPE_RUNTIME_STATS          = 4,
  PKT_TYPE_REQUEST_HEARTBEAT      = 5,
  PKT_TYPE_READ_CONFIG            = 6,
  PKT_TYPE_READ_STRIP_CONFIG_ALL  = 7,
  PKT_TYPE_WRITE_STRIP_CONFIG_ALL = 8,
  PKT_TYPE_READ_NETWORK_CONFIG    = 9,
  PKT_TYPE_WRITE_NETWORK_CONFIG   = 10,
  PKT_TYPE_ANNOUNCE_BROADCAST     = 11,
  PKT_TYPE_CONFIG_RESPONSE        = 12,
  PKT_NUM_TYPES,
};

enum RGBConfigType : uint8_t {
  CFG_ID_DIMENSIONS_WIDTH  = 0,
  CFG_ID_DIMENSIONS_HEIGHT = 1,
  CFG_ID_MAX_BRIGHTNESS    = 2,
  CFG_ID_FRAME_RATE        = 3,
  CFG_ID_STRIPS_ENABLE     = 4, // bitmask of strips to enable
  CFG_ID_ENABLE_DEBUG      = 5, // set debug variable (Bit 0: show buffer positions in Display)
  CFG_ID_HEARTBEAT_INTERVAL_FRAMES = 6, // send heartbeat every n frames
  CFG_NUM_CONFIGS,
};

enum RGBDataType : uint8_t {
  RGB_DATA_TYPE_RGB = 0,
  RGB_DATA_TYPE_RGBW = 1,
};

enum LEDType : uint8_t {
  LED_TYPE_WS2812B = 0,
  LED_TYPE_SK6812 = 1,
};

enum WifiMode : uint8_t {
  NETWORK_WIFI_MODE_AP = 0,
  NETWORK_WIFI_MODE_STA = 1,
  NUM_NETWORK_WIFI_MODES,
};

#pragma pack(push, 2)
struct packet_hdr_t {
  uint16_t packetType;
  uint16_t len;
};

struct heartbeat_message_t {
  uint32_t frameId;
  uint32_t bufferFillLevel;
};
typedef struct {
  uint32_t version;
  uint32_t magic;
  uint32_t rmtClockResolution_hz;
  uint32_t dimensionsWidth;
  uint32_t dimensionsHeight;
  uint32_t heartbeatIntervalFrames;
  uint32_t stripsEnable;
  uint32_t debugFlags;
  uint16_t frameRate;
  uint8_t maxBrightness;
} display_config_t;

typedef struct {
  int16_t gpio;
  uint16_t ledCount;
  uint8_t ledType;    // 0: RGB (WS2812B), 1: RGBW (SK6812)
  uint8_t dataTypeId; // 0: RGB (3 bytes), 1: RGBW (4 bytes)
} display_led_strip_config_t;

typedef struct {
  uint32_t version;
  uint32_t magic;
  display_led_strip_config_t configs[CONFIG_MAX_STRIPS];
} display_led_strip_config_all_t;

typedef struct {
  uint32_t version;
  uint32_t magic;
  uint16_t port;
  uint8_t wifiMode;
  uint8_t reserved[5];
  char hostname[64];
  char apWifiSsid[32];
  char apWifiPassword[64];
  char staWifiSsid[32];
  char staWifiPassword[64];
} display_network_config_t;

struct runtime_stats_t
{
  // ring buffer positions
  uint64_t readIndex;
  uint64_t writeIndex;
  // runtime stats
  int64_t numCallbacks;
  int64_t numFrames;
  int64_t numBufferUnderrun;
  int64_t numHeartbeats;
  float fps_actual;
  float fps_callbacks;
};

struct config_message_t {
  uint32_t cfgId;
  uint32_t value;
};

struct led_frame_message_t {
  uint32_t frameSize;  // number of LEDs
  uint32_t frameOffset;// offset in LEDs
  uint8_t data[0];     // RGBW data
};

enum request_heartbeat_flags : uint8_t {
  REQUEST_HEARTBEAT_ENABLE = 1,
  REQUEST_RUNTIME_STATS_ENABLE = 2,
};

struct request_heartbeat_message_t {
  uint8_t flags;  // bitmask of flags
};

enum config_category_e : uint32_t {
  CONFIG_CATEGORY_DISPLAY = 0,
  CONFIG_CATEGORY_NETWORK = 1,
  CONFIG_CATEGORY_STRIPS = 2,
  CONFIG_CATEGORY_NUM,
};
struct config_response_message_t {
  uint32_t cfgEnum;
  int32_t status; // 0: success, 1: error
  char errorMsg[CONFIG_ERR_MSG_BUF_SIZE];
};

struct packet_heartbeat_t {
  struct packet_hdr_t header;
  struct heartbeat_message_t message;
};

struct packet_config_single_t {
  struct packet_hdr_t header;
  struct config_message_t message;
};

struct packet_config_all_t {
  struct packet_hdr_t header;
  display_config_t message;
};

struct packet_led_frame_t {
  struct packet_hdr_t header;
  struct led_frame_message_t message;
};

struct packet_request_heartbeat_t {
  struct packet_hdr_t header;
  struct request_heartbeat_message_t message;
};

struct packet_runtime_stats_t {
  struct packet_hdr_t header;
  struct runtime_stats_t message;
};
struct packet_strip_config_all_t {
  struct packet_hdr_t header;
  display_led_strip_config_all_t message;
};
struct packet_network_config_t {
  struct packet_hdr_t header;
  display_network_config_t message;
};

struct packet_broadcast_t {
  struct packet_hdr_t header;
  int message;
};
struct packet_config_response_t {
  struct packet_hdr_t header;
  struct config_response_message_t message;
};

#pragma pack(pop)
