#pragma once
#include <stdint.h>

enum RGBNetworkPacketType : uint8_t {
  PKT_TYPE_HEARTBEAT = 0,
  PKT_TYPE_CONFIG    = 1,
  PKT_TYPE_LED_FRAME = 2,
};

// move to own header
enum RGBConfigType : uint8_t {
  CFG_ID_MAX_BRIGHTNESS = 1,
  CFG_ID_FRAME_RATE     = 2,
  CFG_ID_STRIPES_ENABLE = 3, // bitmask of stripes to enable
  CFG_ID_ENABLE_DEBUG   = 4, // set debug variable (Bit 0: show buffer positions in Display)
  CFG_ID_HEARTBEAT_INTERVAL_FRAMES = 5, // send heartbeat every n frames
  CFG_NUM_CONFIGS,
};

#pragma pack(push, 2)
struct packet_hdr_t {
  uint16_t packetType;
  uint16_t len;
};

struct heartbeat_message_t {
  uint64_t frameId;
  uint64_t fps;
  uint64_t heartbeatIntervalFrames;
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

struct packet_heartbeat_t {
  struct packet_hdr_t header;
  struct heartbeat_message_t message;
};

struct packet_config_t {
  struct packet_hdr_t header;
  struct config_message_t message;
};

struct packet_led_frame_t {
  struct packet_hdr_t header;
  struct led_frame_message_t message;
};
#pragma pack(pop)
