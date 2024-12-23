import http
from http.server import HTTPServer, SimpleHTTPRequestHandler
import json
import os
import socket
import socketserver
import sys
import threading
import time
import struct
import random
import math
import traceback
import websockets
import asyncio

from websockets.asyncio.server import serve

HTTP_ADDR = ""
HTTP_PORT = 8000
HTTP_DIRECTORY = "./web/"

WEBSOCKET_LISTEN_ADDR = ""
WEBSOCKET_LISTEN_PORT = 8001

# packet types
PKT_TYPE_HEARTBEAT = 0
PKT_TYPE_CONFIG_ALL = 1
PKT_TYPE_CONFIG_SINGLE = 2
PKT_TYPE_LED_FRAME = 3
PKT_TYPE_RUNTIME_STATS = 4
PKT_TYPE_REQUEST_HEARTBEAT = 5
PKT_TYPE_READ_CONFIG = 6
PKT_TYPE_READ_STRIP_CONFIG_ALL  = 7
PKT_TYPE_WRITE_STRIP_CONFIG_ALL = 8
PKT_TYPE_READ_NETWORK_CONFIG    = 9
PKT_TYPE_WRITE_NETWORK_CONFIG   = 10
PKT_TYPE_ANNOUNCE_BROADCAST     = 11
PKT_TYPE_JSON_CLIENT_LIST       = 100

# config types
CFG_ID_DIMENSIONS_WIDTH = 0
CFG_ID_DIMENSIONS_HEIGHT = 1
CFG_ID_MAX_BRIGHTNESS = 2
CFG_ID_FRAME_RATE = 3
CFG_ID_STRIPS_ENABLE = 4  # bitmask of strips to enable
CFG_ID_ENABLE_DEBUG = 5  # set debug variable (Bit 0: show buffer positions in Display)
CFG_ID_HEARTBEAT_INTERVAL_FRAMES = 6  # send heartbeat every n frames
CFG_ID_TEMPO = 100
CFG_ID_SCALE = 200

# network settings
LED_CONTROLLER_DEFAULT_IP = "192.168.188.60"
LED_CONTROLLER_UDP_PORT = 54321

# default settings
FPS_DEFAULT = 100
MAX_BRIGHTNESS_DEFAULT = 64
HEARTBEAT_INTERVAL_DEFAULT = 8

# packet defs relating to https://github.com/nidefawl/esp32-rgb/blob/main/main/rgb-network-types.h
# packet header, u16 packet type, u16 packet size
packet_hdr = struct.Struct("<H H")

# heartbeat message, u32 frameId, u32 buffer fill level
heartbeat_message = struct.Struct("<I I")

# config message, u32 config id, u32 config value
config_message = struct.Struct("<I I")

# led frame message: u32 frame size (in number of LEDS), u32 frame offset (in number of LEDS), then led data (LED data is 4 bytes per LED (WRGB))
# pointer to frame data is not included in struct, it is added later
led_frame_message = struct.Struct("<I I")

DISPLAY_CONFIG_VERSION = 2
DISPLAY_CONFIG_MAGIC = 0x12345678

# struct config_message_t {
#   uint32_t cfgId;
#   uint32_t value;
# };

# typedef struct {
#  uint32_t version;
#  uint32_t magic;
#  uint32_t rmtClockResolution_hz;
#  uint32_t dimensionsWidth;
#  uint32_t dimensionsHeight;
#  uint32_t heartbeatIntervalFrames;
#  uint32_t stripsEnable;
#  uint32_t debugFlags;
#  uint16_t frameRate;
#  uint8_t maxBrightness;
#  uint8_t isRGBW;
# } display_config_t;
config_message_all = struct.Struct("<I I I I I I I I H B B")

# struct runtime_stats_t
# {
#   // ring buffer positions
#   uint64_t readIndex;
#   uint64_t writeIndex;
#   // runtime stats
#   int64_t numCallbacks;
#   int64_t numFrames;
#   int64_t numBufferUnderrun;
#   int64_t numHeartbeats;
#   float fps_actual;
#   float fps_callbacks;
# };
runtime_stats_message = struct.Struct("<Q Q q q q q f f")

# enum request_heartbeat_flags : uint8_t {
#   REQUEST_HEARTBEAT_ENABLE = 1,
#   REQUEST_RUNTIME_STATS_ENABLE = 2,
# };

# struct request_heartbeat_message_t {
#   uint8_t flags;  // bitmask of flags
# };
request_heartbeat_message = struct.Struct("<B")

# global runtime variables
g_enable_runtime_stats = True
g_time_start = 0
g_num_frames_all_lamps = 0
g_abs_speed = 2.0
g_abs_scale = 2.0
ledrand = random.Random()

def get_program_names():
    # return ["rainbow_circle", "spiral_outward", "static_color", "2s_second_black_white", "white_noise"]
    # use python reflection (globals()) to find all functions starting with process_pixels_
    return [str(x) for x in globals() if x.startswith("process_pixels_")]
CUR_PROGRAM = "process_pixels_rainbow_circle"

palettes = []
palettes.append(([0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [1.0, 1.0, 1.0], [0.0, 0.33, 0.67]))
palettes.append(([0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [1.0, 1.0, 1.0], [0.0, 0.10, 0.20]))
palettes.append(([0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [1.0, 1.0, 1.0], [0.3, 0.20, 0.20]))
palettes.append(([0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [1.0, 1.0, 0.5], [0.8, 0.90, 0.30]))
palettes.append(([0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [1.0, 0.7, 0.4], [0.0, 0.15, 0.20]))
palettes.append(([0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [2.0, 1.0, 0.0], [0.5, 0.20, 0.25]))
palettes.append(([0.8, 0.5, 0.4], [0.2, 0.4, 0.2], [2.0, 1.0, 1.0], [0.0, 0.25, 0.25]))


# vec3 pal( in float t, in vec3 a, in vec3 b, in vec3 c, in vec3 d )
# {
#     return a + b*cos( 6.28318*(c*t+d) );
# }
# python implementation of the above GLSL function
def pal(t, a, b, c, d):
    return [a[i] + b[i] * math.cos(6.28318 * (c[i] * t + d[i])) for i in range(3)]

def switch_to_next_program():
    global CUR_PROGRAM
    programs = get_program_names()
    idx = programs.index(CUR_PROGRAM)
    idx = (idx + 1) % len(programs)
    CUR_PROGRAM = programs[idx]
    print("Switched to program", CUR_PROGRAM)
# python class representing a lamp config
class LampConfig:
    """Class representing a lamp configuration"""

    # default cstr
    def __init__(
        self,
        dimensions,
        is_rgbw,
        max_brightness,
        frame_rate,
        strips_enable,
        debug_bits,
        heartbeat_interval_frames,
        rmt_clock_hz,
    ):
        self.dimensions = dimensions
        self.is_rgbw = 1 * is_rgbw
        self.max_brightness = max_brightness
        self.frame_rate = frame_rate
        self.strips_enable = strips_enable
        self.debug_bits = debug_bits
        self.heartbeat_interval_frames = heartbeat_interval_frames
        self.rmt_clock_hz = rmt_clock_hz

    # cstr not taking any arguments
    @classmethod
    def default(cls):
        return cls([0, 0], 0, 0, 0, 0, 0, 0, 0)

    def __init__(
        self,
        dimensions,
        is_rgbw,
        max_brightness,
        frame_rate,
        strips_enable,
        debug_bits,
        heartbeat_interval_frames,
        rmt_clock_hz,
    ):
        self.dimensions = dimensions
        self.is_rgbw = 1 * is_rgbw
        self.max_brightness = max_brightness
        self.frame_rate = frame_rate
        self.strips_enable = strips_enable
        self.debug_bits = debug_bits
        self.heartbeat_interval_frames = heartbeat_interval_frames
        self.rmt_clock_hz = rmt_clock_hz

    def pack(self):
        return config_message_all.pack(
            DISPLAY_CONFIG_VERSION,
            DISPLAY_CONFIG_MAGIC,
            self.rmt_clock_hz,
            self.dimensions[0],
            self.dimensions[1],
            self.heartbeat_interval_frames,
            self.strips_enable,
            self.debug_bits,
            self.frame_rate,
            self.max_brightness,
            self.is_rgbw,
        )

    def __str__(self):
        return (
            "dimensions: "
            + str(self.dimensions)
            + ", is_rgbw: "
            + str(self.is_rgbw)
            + ", max_brightness: "
            + str(self.max_brightness)
            + ", frame_rate: "
            + str(self.frame_rate)
            + ", strips_enable: "
            + str(self.strips_enable)
            + ", debug_bits: "
            + str(self.debug_bits)
            + ", heartbeat_interval_frames: "
            + str(self.heartbeat_interval_frames)
            + ", rmt_clock_hz: "
            + str(self.rmt_clock_hz)
        )

    def __repr__(self):
        return self.__str__()
    

class LampConfigEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, LampConfig):
            return {
                "dimensions": obj.dimensions,
                "is_rgbw": obj.is_rgbw,
                "max_brightness": obj.max_brightness,
                "frame_rate": obj.frame_rate,
                "strips_enable": obj.strips_enable,
                "debug_bits": obj.debug_bits,
                "heartbeat_interval_frames": obj.heartbeat_interval_frames,
                "rmt_clock_hz": obj.rmt_clock_hz,
            }
        return json.JSONEncoder.default(self, obj)
    


# python class representing a lamp
class Lamp:
    """Class representing a lamp"""

    def __init__(self, idx, name, ip, port, ip_pton):
        self.idx = idx
        self.name = name
        self.ip = ip
        self.port = port
        self.config = LampConfig.default()
        self.frames_sent = 0
        self.frame_id_max = 0
        self.ip_pton = ip_pton
        self.program_state = {}
        self.stats = {}
        self.tmLastPacket = time.time()
        self.frame_counter = 0

    def __str__(self):
        return (
            "name: "
            + self.name
            + ", ip: "
            + self.ip
            + ", port: "
            + str(self.port)
            + ", config: "
            + str(self.config)
        )

    def __repr__(self):
        return self.__str__()

    def request_heartbeat(self, sock, enable_heartbeat, enable_runtime_stats):
        flags = 0
        if enable_heartbeat:
            flags |= 1
        if enable_runtime_stats:
            flags |= 2
        pkt_request_heartbeat = bytearray(
            packet_hdr.pack(PKT_TYPE_REQUEST_HEARTBEAT, request_heartbeat_message.size)
        )
        pkt_request_heartbeat += request_heartbeat_message.pack(flags)
        sock.sendto(pkt_request_heartbeat, (self.ip, self.port))
        return sock

    def send_config(self, sock):
        pkt_cfg_all = bytearray(
            packet_hdr.pack(PKT_TYPE_CONFIG_ALL, config_message_all.size)
        )
        pkt_cfg_all += self.config.pack()
        sock.sendto(pkt_cfg_all, (self.ip, self.port))
        return sock
    
    def update_config(self, sock, cfg_id, value):
        pkt_cfg_single = bytearray(packet_hdr.pack(PKT_TYPE_CONFIG_SINGLE, config_message.size))
        pkt_cfg_single += config_message.pack(cfg_id, value)
        sock.sendto(pkt_cfg_single, (self.ip, self.port))
        return sock

    def send_heartbeat(self, sock):
        pkt_heartbeat = bytearray(
            packet_hdr.pack(PKT_TYPE_HEARTBEAT, heartbeat_message.size)
        )
        pkt_heartbeat += heartbeat_message.pack(0, 0)
        sock.sendto(pkt_heartbeat, (self.ip, self.port))
        return sock

    def load_config(self, sock):
        pkt_read_config = bytearray(packet_hdr.pack(PKT_TYPE_READ_CONFIG, 0))
        sock.sendto(pkt_read_config, (self.ip, self.port))
        time_start = time.time()
        while True:
            # read pack packets, (possibly some we are not interested in)
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                if time.time() - time_start > 2.0:
                    print("Timeout reading config")
                    break
                continue
            except IOError as msg:
                print(msg)
                continue
            except KeyboardInterrupt:
                return False

            if (
                len(data) > config_message.size
                and socket.inet_pton(sock.family, addr[0]) == self.ip_pton
            ):
                pkt_type, _ = packet_hdr.unpack(data[:4])
                if pkt_type == PKT_TYPE_CONFIG_ALL:
                    (
                        version,
                        magic,
                        rmt_clock_hz,
                        dimensions_width,
                        dimensions_height,
                        heartbeat_interval_frames,
                        strips_enable,
                        debug_flags,
                        frame_rate,
                        max_brightness,
                        is_rgbw,
                    ) = config_message_all.unpack(data[4:])
                    self.config = LampConfig(
                        [dimensions_width, dimensions_height],
                        is_rgbw,
                        max_brightness,
                        frame_rate,
                        strips_enable,
                        debug_flags,
                        heartbeat_interval_frames,
                        rmt_clock_hz,
                    )
                    print(self.config)
                    return True

        return False

    def gen_frame(self, sock, frame_id):
        time_since = frame_id / self.config.frame_rate
        led_frame_data = None
        if "image" in self.program_state:
            if "image_load_failed" in self.program_state:
                pass
            else:
                # load image and transform to display resolution
                try:
                    from PIL import Image
                    # first check if path is a folder
                    is_folder = os.path.isdir(self.program_state["image"])
                    if is_folder:
                        print("Loading image from folder", self.program_state["image"])
                        try:
                          # get all files in folder
                          files = os.listdir(self.program_state["image"])
                          # filter for image files
                          files = [f for f in files if f.endswith(".png") or f.endswith(".jpg") or f.endswith(".jpeg")]
                          # sort files by name
                          files.sort()
                          # get current frame
                          frame_idx = frame_id % len(files)
                          self.program_state["image"] = os.path.join(self.program_state["image"], files[frame_idx])
                        except:
                          print("Error loading image from folder", self.program_state["image"])
                          print("Error:", sys.exc_info()[0])
                    img = Image.open(self.program_state["image"])
                    imgMode = img.mode
                    # mirror the image
                    img = img.transpose(Image.FLIP_LEFT_RIGHT)
                    if img.size[0] != self.config.dimensions[0] or img.size[1] != self.config.dimensions[1]:
                      img = img.resize(self.config.dimensions)
                    #print color of lower left pixel to console
                    print(img.getpixel((0, img.size[1] - 1)))
                    #print color of lower right pixel to console
                    print(img.getpixel((img.size[0] - 1, img.size[1] - 1)))
                    img = img.convert("RGBA")
                    img_data = img.tobytes()
                    self.program_state["image_load_failed"] = False
                                      
                      # r = (color >> 16) & 0xFF
                      # g = (color >> 8) & 0xFF
                      # b = color & 0xFF
                      # w = (color >> 24) & 0xFF
                      # for _ in range(LED_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP):
                      #     led_frame_data += struct.pack("BBBB", r, g, b, w)
                    frame_image_data = bytearray()
                    # for i in range(0, len(img_data), 4):
                    for idx_strip in range(self.config.dimensions[0]):
                        for idx_led in range(self.config.dimensions[1]):
                            idx_image = (idx_strip + idx_led * self.config.dimensions[0]) * 4
                            r = img_data[idx_image]
                            g = img_data[idx_image + 1]
                            b = img_data[idx_image + 2]
                            a = img_data[idx_image + 3]
                            # apply alpha blending
                            r = r * a // 255
                            g = g * a // 255
                            b = b * a // 255
                            frame_image_data += struct.pack("BBBB", b, g, r, a)
                    self.program_state["image_data"] = frame_image_data
                except:
                    print("Error loading image", self.program_state["image"])
                    print("Error:", sys.exc_info()[0])  
                    self.program_state["image_load_failed"] = True
            if "image_data" in self.program_state:
              led_frame_data = led_frame_message.pack(
                  len(self.program_state["image_data"]) // 4, 0
              )
              led_frame_data += self.program_state["image_data"]
        elif "static" in self.program_state:
          led_frame_data = process_pixels_static_color(
              time_since, frame_id, self.program_state, self.config.dimensions
          )
        else:
          # get function by reflection
          fn_program = globals()[CUR_PROGRAM]
          led_frame_data = fn_program(
              time_since, frame_id, self.program_state, self.config.dimensions
          )
        if not led_frame_data:
            return
        size_led_frame_data = len(led_frame_data)
        led_frame_packet = bytearray(
            packet_hdr.pack(PKT_TYPE_LED_FRAME, size_led_frame_data)
        )
        led_frame_packet += led_frame_data
        # send led frame packet
        sock.sendto(led_frame_packet, (self.ip, self.port))
        self.frames_sent += 1
        self.frame_id_max = frame_id

    def get_frames_sent(self):
        return self.frames_sent
  
    def get_frame_id_max(self):
        return self.frame_id_max

    def handle_packet(self, server, data, pkt_type):
        global g_num_frames_all_lamps
        sock = server["socket"]
        sync_to_first_lamp = True
        self.tmLastPacket = time.time()
        if pkt_type == PKT_TYPE_HEARTBEAT:
            frame_id, buf_fill = heartbeat_message.unpack(data[4:])
            num_frames_to_generate = self.config.heartbeat_interval_frames
            if buf_fill < 16:
                num_frames_to_generate += 4
                # print("4 frames added", buf_fill)
            if buf_fill < 48-5:
                num_frames_to_generate += 1
                # print("1 frame added", buf_fill)
            if buf_fill > 48+self.config.heartbeat_interval_frames-5:
                num_frames_to_generate -= 1
                # print("1 frame removed", buf_fill)

            frame_id = self.frame_counter
            if sync_to_first_lamp:
                # find lamp with highest frame id
                lamp_highest = max(server["lamps"], key=lambda x: x.frame_counter)
                if lamp_highest is not self and lamp_highest.frame_counter > frame_id:
                    abs_diff = lamp_highest.frame_counter - frame_id
                    if abs_diff > 100:
                      frame_id = lamp_highest.frame_counter
                      print("Syncing to ", lamp_highest.name, frame_id)

            for _ in range(num_frames_to_generate):
                self.gen_frame(sock, frame_id)
                frame_id += 1
            self.frame_counter = frame_id
            g_num_frames_all_lamps += num_frames_to_generate
        elif pkt_type == PKT_TYPE_RUNTIME_STATS:
            # put stuff in a dictionary
            lamp_stats = {}
            (
                lamp_stats["read_index"],
                lamp_stats["write_index"],
                lamp_stats["num_callbacks"],
                lamp_stats["num_frames"],
                lamp_stats["num_buffer_underrun"],
                lamp_stats["num_heartbeats"],
                lamp_stats["fps_actual"],
                lamp_stats["fps_callbacks"],
            ) = runtime_stats_message.unpack(data[4:])
            self.stats = lamp_stats
            # calculate buffer fill level
            # buf_fill = lamp_stats["write_index"] - lamp_stats["read_index"]
            # print(self.name, "Buffer fill level", buf_fill)
            # print(self.stats)
        elif pkt_type == PKT_TYPE_CONFIG_ALL:
            (
                version,
                magic,
                rmt_clock_hz,
                dimensions_width,
                dimensions_height,
                heartbeat_interval_frames,
                strips_enable,
                debug_flags,
                frame_rate,
                max_brightness,
                is_rgbw,
            ) = config_message_all.unpack(data[4:])
            print(
                "Received config all",
                "version",
                version,
                "magic",
                magic,
                "rmt_clock_hz",
                rmt_clock_hz,
                "dimensions_width",
                dimensions_width,
                "dimensions_height",
                dimensions_height,
                "heartbeat_interval_frames",
                heartbeat_interval_frames,
                "strips_enable",
                strips_enable,
                "debug_flags",
                debug_flags,
                "frame_rate",
                frame_rate,
                "max_brightness",
                max_brightness,
                "is_rgbw",
                is_rgbw,
            )
            self.config = LampConfig(
                [dimensions_width, dimensions_height],
                is_rgbw,
                max_brightness,
                frame_rate,
                strips_enable,
                debug_flags,
                heartbeat_interval_frames,
                rmt_clock_hz,
            )
            print(self.config)

    def get_time_last_packet(self):
        return self.tmLastPacket


def process_pixels_rainbow_circle(time_since, frame_id, state, dimensions):
    global ledrand, g_abs_scale, g_abs_speed
    LED_NUM_STRIPS = dimensions[0]
    LED_STRIP_NUM_LEDS_PER_STRIP = dimensions[1]
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP, 0
    )
    speed2 = 33 * g_abs_speed
    f = math.cos(time_since * speed2 * math.pi / 180) * 0.5 + 0.5
    f2 = math.cos(time_since * speed2 * 1.1 * math.pi / 180) * 0.5 + 0.5
    palette_phase = time_since * 0.1 * g_abs_speed
    idx_palette = int(palette_phase) % len(palettes)
    palette_phase = palette_phase % 1.0
    for strip in range(LED_NUM_STRIPS):
        for led in range(LED_STRIP_NUM_LEDS_PER_STRIP):
            y0to1 = 0.5 if LED_STRIP_NUM_LEDS_PER_STRIP == 1 else led / (LED_STRIP_NUM_LEDS_PER_STRIP - 1)
            x0to1 = 0.5 if LED_NUM_STRIPS == 1 else strip / (LED_NUM_STRIPS - 1)
            # use distance from center to determine a continuous gradient circle
            dst = math.sqrt((x0to1 - 0.5) ** 2 + (y0to1 - 0.5) ** 2)
            hue = dst * 360 * (0.5 + g_abs_scale * f + 5.5 * f * f2)
            speed = 64.0 *g_abs_speed

            hue -= time_since * speed
            hue = hue % 360

            palette = palettes[idx_palette]
            val = pal(hue / 360.0, palette[0], palette[1], palette[2], palette[3])
            # interpolate with next palette after 0.8
            if palette_phase > 0.9:
                phase = (palette_phase - 0.9) / 0.1
                palette2 = palettes[(idx_palette + 1) % len(palettes)]
                val2 = pal(
                    hue / 360.0, palette2[0], palette2[1], palette2[2], palette2[3]
                )
                val = [val[i] * (1.0 - phase) + val2[i] * phase for i in range(3)]

            # # calculate a proper white value from the RGB values
            luma = val[0] * 0.299 + val[1] * 0.587 + val[2] * 0.114
            col_normalized = val + [luma]  # add white channel

            # clamp values
            for i in range(4):
                # if col_normalized[i] > 1.0:
                #     print("Warning: color value out of range", col_normalized[i])
                col_normalized[i] = max(0.0, col_normalized[i])

            # apply gamma correction
            gamma = 2.2
            for i in range(4):
                col_normalized[i] = col_normalized[i] ** gamma

            # clamp values
            for i in range(4):
                col_normalized[i] = min(1.0, max(0.0, col_normalized[i]))

            # add dither noise before quantization to 8 bit
            noise_scale = (1.0 / 255) * 0.5
            for i in range(4):
                col_normalized[i] += ledrand.uniform(
                    -noise_scale * 0.5, noise_scale * 0.5
                )
                col_normalized[i] = min(1.0, max(0.0, col_normalized[i]))

            r, g, b, w = [int(col_normalized[i] * 255) for i in range(4)]
            led_frame_data += struct.pack("BBBB", int(b), int(g), int(r), int(w))
    return led_frame_data

def process_pixels_spiral_outward(time_since, frame_id, state, dimensions):
    LED_NUM_STRIPS = dimensions[0]
    LED_STRIP_NUM_LEDS_PER_STRIP = dimensions[1]
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP, 0
    )
    begin = [LED_NUM_STRIPS // 2, LED_STRIP_NUM_LEDS_PER_STRIP // 2]
    if LED_NUM_STRIPS % 2 == 0:
        begin[0] -= 1
    if LED_STRIP_NUM_LEDS_PER_STRIP % 2 == 0:
        begin[1] -= 1
    SPIRAL_DIRECTIONS = [[-1, 0], [0, -1], [1, 0], [0, 1]]
      # clear spiral if all leds are filled
    # to have all leds filled we need to make sure x_min, y_min, x_max, y_max are all filled
    all_filled = False
    if "spiral" in state:
        x_min = min([x[0] for x in state["spiral"]])
        x_max = max([x[0] for x in state["spiral"]])
        y_min = min([x[1] for x in state["spiral"]])
        y_max = max([x[1] for x in state["spiral"]])
        all_filled = x_min <= 0 and x_max >= LED_NUM_STRIPS - 1 and y_min <= 0 and y_max >= LED_STRIP_NUM_LEDS_PER_STRIP - 1

    if not "spiral" in state or all_filled:
        state["spiral"] = [[begin[0], begin[1]]]
        state["spiral_dir"] = SPIRAL_DIRECTIONS[0]
        state["spiral_arm_len"] = 2



    if frame_id % 5 == 0:
        last = state["spiral"][-1]
        next_dir = SPIRAL_DIRECTIONS[(SPIRAL_DIRECTIONS.index(state["spiral_dir"]) + 1) % 4]
        next = [
            last[0] + next_dir[0],
            last[1] + next_dir[1]
        ]
        if state["spiral_arm_len"] > 1 and next not in state["spiral"]:
            state["spiral_arm_len"] += 1
            state["spiral_dir"] = next_dir
            state["spiral"].append(next)
        else:
            next = [
                last[0] + state["spiral_dir"][0],
                last[1] + state["spiral_dir"][1]
            ]
            state["spiral"].append(next)

    for strip in range(LED_NUM_STRIPS):
        for led in range(LED_STRIP_NUM_LEDS_PER_STRIP):
            if [strip, led] not in state["spiral"]:
                val = [0, 0, 0, 0]
            else:
                val = [255, 255, 255, 255]
            # calculate a proper white value from the RGB values
            luma = val[0] * 0.299 + val[1] * 0.587 + val[2] * 0.114
            col_normalized = val + [luma * 2]  # add white channel
            # clamp values
            for i in range(4):
                # if col_normalized[i] > 1.0:
                #     print("Warning: color value out of range", col_normalized[i])
                col_normalized[i] = max(0.0, col_normalized[i])
            # apply gamma correction
            gamma = 2.2
            for i in range(4):
                col_normalized[i] = col_normalized[i] ** gamma
            # clamp values
            for i in range(4):
                col_normalized[i] = min(1.0, max(0.0, col_normalized[i]))
            # add dither noise before quantization to 8 bit
            noise_scale = (1.0 / 255) * 0.5
            for i in range(4):
                col_normalized[i] += ledrand.uniform(
                    -noise_scale * 0.5, noise_scale * 0.5
                )
                col_normalized[i] = min(1.0, max(0.0, col_normalized[i]))
            r, g, b, w = [int(col_normalized[i] * 255) for i in range(4)]
            led_frame_data += struct.pack("BBBB", int(b), int(g), int(r), int(w))
    return led_frame_data
def process_pixels_static_color(time_since, frame_id, state, dimensions):
    LED_NUM_STRIPS = dimensions[0]
    LED_STRIP_NUM_LEDS_PER_STRIP = dimensions[1]
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP, 0
    )
    color = 0x00000000
    if "static" in state:
        color = state["static"]
    r = (color >> 16) & 0xFF
    g = (color >> 8) & 0xFF
    b = color & 0xFF
    w = (color >> 24) & 0xFF
    for _ in range(LED_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP):
        led_frame_data += struct.pack("BBBB", r, g, b, w)
    return led_frame_data

# on and off in slow fashion
def process_pixels_2s_second_black_white(time_since, frame_id, state, dimensions):
    LED_NUM_STRIPS = dimensions[0]
    LED_STRIP_NUM_LEDS_PER_STRIP = dimensions[1]
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP, 0
    )
    is_black = int(time_since * 4.5) % 2 == 0
    color = [0, 0, 0, 0] if is_black else [255, 255, 255, 255]
    r, g, b, w = color
    for _ in range(LED_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP):
        led_frame_data += struct.pack("BBBB", r, g, b, w)
    return led_frame_data


# process_pixels_white_noise generates a white noise pattern
def process_pixels_white_noise(time_since, frame_id, state, dimensions):
    LED_NUM_STRIPS = dimensions[0]
    LED_STRIP_NUM_LEDS_PER_STRIP = dimensions[1]
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP, 0
    )

    for strip in range(LED_NUM_STRIPS):
        for led in range(LED_STRIP_NUM_LEDS_PER_STRIP):
            r = random.randint(0, 255)
            g = random.randint(0, 255)
            b = random.randint(0, 255)
            w = random.randint(0, 255)
            # add RGB values to led_frame_data
            led_frame_data += struct.pack("BBBB", int(b), int(g), int(r), int(w))
    return led_frame_data

def init_lamp(server, ip_pton, ip_str, port):
    global g_enable_runtime_stats
    sock = server["socket"]
    args = server["args"]
    # lamp_idx = len(server["lamps"])
    lamp_idx = 0
    # find first free
    for i in range(len(server["lamps"])):
        if server["lamps"][i].ip_pton == ip_pton:
            print("Error: lamp with same IP already connected", ip_str)
            return False
    while True:
        lamp_exists = False
        for i in range(len(server["lamps"])):
            if server["lamps"][i].idx == lamp_idx:
                lamp_exists = True
                break
        if lamp_exists:
            lamp_idx += 1
        else:
            break
    lamp = Lamp(
        lamp_idx,
        "Lamp" + str(lamp_idx + 1),
        ip_str,
        port,
        ip_pton,
    )
    # disable heartbeat and runtime stats
    lamp.request_heartbeat(sock, enable_heartbeat=False, enable_runtime_stats=False)

    if not lamp.load_config(sock):
        print("Error: could not load config for IP", ip_str)
        return False

    # print("Lamp current configuration", lamp)

    # update lamp config
    if args.fps is not None:
        lamp.config.frame_rate = args.fps
    if args.brightness is not None:
        lamp.config.max_brightness = args.brightness
    if args.heartbeat_interval is not None:
        lamp.config.heartbeat_interval_frames = args.heartbeat_interval
    if args.strips_enabled is not None:
        lamp.config.strips_enable = args.strips_enabled
    if args.dimensions is not None:
        lamp.config.dimensions = args.dimensions
    if args.rmt_clock_hz is not None:
        lamp.config.rmt_clock_hz = args.rmt_clock_hz
    if args.rgbw is not None:
        lamp.config.is_rgbw = int(args.rgbw)
    if args.dbg is not None:
        lamp.config.debug_bits = args.dbg
    if args.static is not None:
        lamp.program_state["static"] = args.static
    if args.image is not None:
        lamp.program_state["image"] = args.image

    lamp.send_config(sock)
    # remove all lamps with same ip from server["lamps"]
    newLamps = [l for l in server["lamps"] if ip_pton != l.ip_pton]
    newLamps.append(lamp)
    print("Lamp connected", lamp.name, lamp.ip)
    server["lamps"] = newLamps
    lamp.request_heartbeat(sock, enable_heartbeat=True, enable_runtime_stats=g_enable_runtime_stats)
    return True

# second thread function to recv heartbeats
def recv_udp_packets(*args):
    server = args[0]
    while True:
        sock = server["socket"]
        if sock == None:
            break
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except IOError as msg:
            print("sock.recvfrom", msg)
            continue
        except KeyboardInterrupt:
            return
        if len(data) > 0:
            pkt_type, _ = packet_hdr.unpack(data[:4])
            ip = addr[0]
            ip_pton = socket.inet_pton(sock.family, ip)
            if pkt_type == PKT_TYPE_ANNOUNCE_BROADCAST:
              init_lamp(server, ip_pton, ip, addr[1])
              continue
            else:
              lamp = None
              for l in server["lamps"]:
                  if ip_pton == l.ip_pton:
                      lamp = l
                      break
              if lamp == None:
                print("Error: received unexpected packet ", pkt_type, "from unknown lamp", ip)
                continue
              lamp.handle_packet(server, data, pkt_type)
        else:
            print("Received packet with invalid size", len(data))


def is_valid_ipv6_address(address):
    try:
        socket.inet_pton(socket.AF_INET6, address)
    except socket.error:  # not a valid address
        return False
    return True


async def websocket_handler(websocket):
    global g_server, g_abs_scale, g_abs_speed, g_enable_runtime_stats
    while True:
        try:
          message = await websocket.recv()
          obj = json.loads(message)
          if "type" in obj:
              if PKT_TYPE_JSON_CLIENT_LIST == obj["type"]:
                  response_obj = {"type": PKT_TYPE_JSON_CLIENT_LIST, "clients": []}
                  for i, lamp in enumerate(g_server["lamps"]):
                      lamp_config_json = json.dumps(lamp.config, cls=LampConfigEncoder)
                      lamp_config_obj = json.loads(lamp_config_json)
                      response_obj["clients"].append(
                          {
                              "hostname": lamp.name,
                              "ip": lamp.ip,
                              "config": lamp_config_obj,
                              "stats": lamp.stats,
                          }
                      )
                  response_json_str = json.dumps(response_obj)
                  await websocket.send(response_json_str)
              elif PKT_TYPE_CONFIG_ALL == obj["type"]:
                  response_obj = {"type": obj["type"], "lamps": []}
                  for i, lamp in enumerate(g_server["lamps"]):
                      lamp_config_json = json.dumps(lamp.config, cls=LampConfigEncoder)
                      lamp_config_obj = json.loads(lamp_config_json)
                      lamp_config_obj["scale"] = g_abs_scale
                      lamp_config_obj["speed"] = g_abs_speed
                      response_obj["lamps"].append(
                          {
                              "idx": i,
                              "name": lamp.name,
                              "ip": lamp.ip,
                              "config": lamp_config_obj,
                              "stats": lamp.stats,
                          }
                      )
                  await websocket.send(json.dumps(response_obj))
              elif PKT_TYPE_CONFIG_SINGLE == obj["type"]:
                  if "cfgId" in obj and "value" in obj:
                      cfgId = obj["cfgId"]
                      cfgValue = obj["value"]
                      if cfgId == CFG_ID_FRAME_RATE:
                          # for lamp in g_server["lamps"]:
                          #     lamp.request_heartbeat(g_server["socket"], enable_heartbeat=False, enable_runtime_stats=False)
                          for lamp in g_server["lamps"]:
                              lamp.config.frame_rate = int(cfgValue)
                              lamp.update_config(g_server["socket"], CFG_ID_FRAME_RATE, lamp.config.frame_rate)
                              # lamp.send_config(g_server["socket"])
                          # for lamp in g_server["lamps"]:
                          #     lamp.request_heartbeat(g_server["socket"], enable_heartbeat=True, enable_runtime_stats=g_enable_runtime_stats)
                      if cfgId == CFG_ID_MAX_BRIGHTNESS:
                          for lamp in g_server["lamps"]:
                              lamp.config.max_brightness = int(cfgValue)
                              lamp.update_config(g_server["socket"], CFG_ID_MAX_BRIGHTNESS, lamp.config.max_brightness)
                      if cfgId == CFG_ID_HEARTBEAT_INTERVAL_FRAMES:
                          for lamp in g_server["lamps"]:
                              lamp.config.heartbeat_interval_frames = int(cfgValue)
                              lamp.update_config(g_server["socket"], CFG_ID_HEARTBEAT_INTERVAL_FRAMES, lamp.config.heartbeat_interval_frames)
                      if cfgId == CFG_ID_TEMPO:
                          g_abs_speed = float(cfgValue)
                      if cfgId == CFG_ID_SCALE:
                          g_abs_scale = float(cfgValue)
                      

        except websockets.exceptions.ConnectionClosedError as e:
          print("Connection closed")
          break
        except websockets.exceptions.ConnectionClosedOK as e:
          print("Connection closed")
          break
        except:
          print("Error reading websocket message:", sys.exc_info()[0])
          # print stack trace
          traceback.print_exc()
          continue

async def websocket_serve():
    async with serve(websocket_handler, WEBSOCKET_LISTEN_ADDR, WEBSOCKET_LISTEN_PORT):
        await asyncio.get_running_loop().create_future()  # run forever

def webserver_thread():
    httpd_server = None
    try:
        # create http server serving index.html
        HTTP_DIRECTORY = os.path.join(os.path.dirname(__file__), 'web')
        os.chdir(HTTP_DIRECTORY)
        socketserver.TCPServer.allow_reuse_address = True
        httpd_server = socketserver.TCPServer((HTTP_ADDR, HTTP_PORT), SimpleHTTPRequestHandler)
        httpd_server.serve_forever()
    except KeyboardInterrupt:
        pass
    if (httpd_server != None):
        httpd_server.shutdown()
        httpd_server.server_close()

def websocket_thread():
    try:
        asyncio.run(websocket_serve())
    except KeyboardInterrupt:
        pass
    # stop asyncio event loop
    asyncio.get_event_loop().stop()

def main(args=None):
    global g_time_start, g_num_frames_all_lamps, g_server
    if args is None:
        print("Args is None")
        return
    port = args.port if args.port is not None else LED_CONTROLLER_UDP_PORT

    # create socket
    if args.ipv6:
        sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("", LED_CONTROLLER_UDP_PORT))
    
    # create dict with server data
    server = {"socket": sock, "port": port, "lamps": [], "args": args}
    g_server = server

    # create webserver
    webserver_server_thread = threading.Thread(target=webserver_thread, daemon=True)
    webserver_server_thread.start()
    
    # create websocket server
    websocket_server_thread = threading.Thread(target=websocket_thread, daemon=True)
    websocket_server_thread.start()

    g_time_start = time.time()

    # create second thread to receive heartbeats using the same socket
    thread = threading.Thread(target=recv_udp_packets, args=(server,), daemon=True)
    thread.start()

    time_last_fps = time.time()
    avg_fps = 1.0
    if len(server["lamps"]) > 0:
      avg_fps = server["lamps"][0].config.frame_rate

    try:
        while True:
            try:# Collect events until released
                tmNow = time.time()
                time.sleep(0.05)
                

                lamps_timed_out = []
                for lamp in server["lamps"]:
                    tmOut = 5.0
                    if lamp.get_time_last_packet() < tmNow - tmOut:
                        lamps_timed_out.append(lamp)
                
                if len(lamps_timed_out) > 0:
                    for lamp in lamps_timed_out:
                        lamp.request_heartbeat(sock, enable_heartbeat=False, enable_runtime_stats=False)
                        server["lamps"].remove(lamp)
                        print("Lamp timed out", lamp.name, lamp.ip)

                if g_num_frames_all_lamps > 32 and tmNow - time_last_fps > 5.0:
                    current_fps = g_num_frames_all_lamps / (tmNow - time_last_fps)
                    avg_fps = 0.5 * avg_fps + 0.5 * current_fps
                    target_fps = 0 if len(server["lamps"]) == 0 else server["lamps"][0].config.frame_rate
                    lamp_frame_indices = [lamp.get_frame_id_max() for lamp in server["lamps"]]
                    lamp_frame_indices = ",".join([str(i) for i in lamp_frame_indices])
                    print(
                        "FPS:",
                        "Target",
                        target_fps,
                        "Current",
                        current_fps,
                        "Avg",
                        avg_fps,
                        lamp_frame_indices,
                    )
                    g_num_frames_all_lamps = 0
                    time_last_fps = tmNow
            except KeyboardInterrupt:
                break
    finally:
        if sock != None:
            for lamp in server["lamps"]:
                lamp.request_heartbeat(
                    sock, enable_heartbeat=False, enable_runtime_stats=False
                )
            # sleep 0.5s
            time.sleep(0.5)
        sock.close()
        server["socket"] = None


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="RGB LED Controller")
    parser.add_argument(
        "--port",
        type=int,
        help="UDP port of the LED controller",
        default=LED_CONTROLLER_UDP_PORT,
    )
    parser.add_argument("-6", "--ipv6", action="store_true", help="Use IPv6")
    parser.add_argument("-f", "--fps", type=int, help="Frames per second")
    parser.add_argument("-b", "--brightness", type=int, help="Max brightness")
    parser.add_argument(
        "-i", "--heartbeat-interval", type=int, help="Heartbeat interval"
    )
    parser.add_argument(
        "-s",
        "--strips-enabled",
        type=lambda x: int(x, 16),
        help="Stripes enable bitmask",
    )
    parser.add_argument(
        "-d", "--dimensions", type=int, nargs=2, help="Dimensions width height"
    )
    parser.add_argument("-c", "--rmt-clock-hz", type=int, help="RMT clock Hz")
    parser.add_argument("--rgbw", type=int, help="RGBW mode (0 or 1)")
    parser.add_argument("--dbg", "--debug-bits", type=int, help="Set debug bits")
    # add arg "--program [name]", if no name is supplied it will print the list of available programs
    parser.add_argument(
      "--program",
      type=str,
      help="Name of the program to run"
    )
    # add argument to show image or folder of images
    parser.add_argument(
      "--image",
      type=str,
      help="Image or folder of images to display"
    )
    # add argument to allow fullscreen color without animation
    parser.add_argument(
      "--static",
      type=lambda x: int(x, 16),
      help="Static color mode"
    )
    args = parser.parse_args()
    # if no program is supplied, print the list of available programs
    if args.program is None:
      print("Available programs:")
      for program in get_program_names():
        print(program)
    else:
      # if a program is supplied, check if it is a valid program
      if args.program not in get_program_names():
        print("Error: invalid program name")
        print("Available programs:")
        for program in get_program_names():
          print(program)
      else:
        # if the program is valid, set the current program to the supplied program
        CUR_PROGRAM = args.program
    main(args)

