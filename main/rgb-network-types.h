#pragma once
#include <stdint.h>

enum RGBNetworkPacketType : uint8_t {
  PKT_TYPE_HEARTBEAT     = 0,
  PKT_TYPE_CONFIG_ALL    = 1,
  PKT_TYPE_CONFIG_SINGLE = 2,
  PKT_TYPE_LED_FRAME     = 3,
  PKT_TYPE_RUNTIME_STATS = 4,
  PKT_TYPE_REQUEST_HEARTBEAT = 5,
  PKT_TYPE_READ_CONFIG   = 6,
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

#pragma pack(push, 2)
struct packet_hdr_t {
  uint16_t packetType;
  uint16_t len;
};

struct heartbeat_message_t {
  uint32_t frameId;
  uint32_t bufferFillLevel;
};
#define DISPLAY_CONFIG_MAGIC 0x12345678
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
  uint8_t isRGBW;
} display_config_t;

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

#pragma pack(pop)
