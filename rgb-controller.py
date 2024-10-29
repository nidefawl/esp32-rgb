import socket
import threading
import time
import struct
import random
import math

# packet types
PKT_TYPE_HEARTBEAT     = 0
PKT_TYPE_CONFIG_ALL    = 1
PKT_TYPE_CONFIG_SINGLE = 2
PKT_TYPE_LED_FRAME     = 3
PKT_TYPE_RUNTIME_STATS = 4
PKT_TYPE_REQUEST_HEARTBEAT = 5
PKT_TYPE_READ_CONFIG   = 6

# config types
CFG_ID_DIMENSIONS_WIDTH  = 0
CFG_ID_DIMENSIONS_HEIGHT = 1
CFG_ID_MAX_BRIGHTNESS    = 2
CFG_ID_FRAME_RATE        = 3
CFG_ID_STRIPS_ENABLE     = 4 # bitmask of strips to enable
CFG_ID_ENABLE_DEBUG      = 5 # set debug variable (Bit 0: show buffer positions in Display)
CFG_ID_HEARTBEAT_INTERVAL_FRAMES = 6 # send heartbeat every n frames

# network settings
LED_CONTROLLER_DEFAULT_IP = "192.168.188.60"
LED_CONTROLLER_UDP_PORT = 54321

# default settings
FPS_DEFAULT = 60
MAX_BRIGHTNESS_DEFAULT = 127
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

DISPLAY_CONFIG_VERSION = 1
DISPLAY_CONFIG_MAGIC = 0x12345678

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
g_time_start = 0
g_num_frames_all_lamps = 0
ledrand = random.Random()

palettes = []
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[1.0,1.0,1.0],[0.0,0.33,0.67]))
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[1.0,1.0,1.0],[0.0,0.10,0.20]))
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[1.0,1.0,1.0],[0.3,0.20,0.20]))
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[1.0,1.0,0.5],[0.8,0.90,0.30]))
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[1.0,0.7,0.4],[0.0,0.15,0.20]))
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[2.0,1.0,0.0],[0.5,0.20,0.25]))
palettes.append(([0.8,0.5,0.4],[0.2,0.4,0.2],[2.0,1.0,1.0],[0.0,0.25,0.25]))

# vec3 pal( in float t, in vec3 a, in vec3 b, in vec3 c, in vec3 d )
# {
#     return a + b*cos( 6.28318*(c*t+d) );
# }
# python implementation of the above GLSL function
def pal(t, a, b, c, d):
    return [a[i] + b[i] * math.cos(6.28318 * (c[i] * t + d[i])) for i in range(3)]

# python class representing a lamp config
class LampConfig:
    '''Class representing a lamp configuration'''
    # default cstr
    def __init__(self, dimensions, is_rgbw, max_brightness, frame_rate, strips_enable, debug_bits, heartbeat_interval_frames, rmt_clock_hz):
        self.dimensions = dimensions
        self.is_rgbw = 1*is_rgbw
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
    def __init__(self, dimensions, is_rgbw, max_brightness, frame_rate, strips_enable, debug_bits, heartbeat_interval_frames, rmt_clock_hz):
        self.dimensions = dimensions
        self.is_rgbw = 1*is_rgbw
        self.max_brightness = max_brightness
        self.frame_rate = frame_rate
        self.strips_enable = strips_enable
        self.debug_bits = debug_bits
        self.heartbeat_interval_frames = heartbeat_interval_frames
        self.rmt_clock_hz = rmt_clock_hz
    def pack(self):
        return config_message_all.pack(DISPLAY_CONFIG_VERSION, DISPLAY_CONFIG_MAGIC, self.rmt_clock_hz, self.dimensions[0], self.dimensions[1], self.heartbeat_interval_frames, self.strips_enable, self.debug_bits, self.frame_rate, self.max_brightness, self.is_rgbw)
    def __str__(self):
        return "dimensions: " + str(self.dimensions) + ", is_rgbw: " + str(self.is_rgbw) + ", max_brightness: " + str(self.max_brightness) + ", frame_rate: " + str(self.frame_rate) + ", strips_enable: " + str(self.strips_enable) + ", debug_bits: " + str(self.debug_bits) + ", heartbeat_interval_frames: " + str(self.heartbeat_interval_frames) + ", rmt_clock_hz: " + str(self.rmt_clock_hz)
    def __repr__(self):
        return self.__str__()
        
# python class representing a lamp
class Lamp:
    '''Class representing a lamp'''
    def __init__(self, name, ip, port, ip_pton):
        self.name = name
        self.ip = ip
        self.port = port
        self.config = LampConfig.default()
        self.frames_sent = 0
        self.ip_pton = ip_pton
        self.program_state = {}
        self.stats = {}
    def __str__(self):
        return "name: " + self.name + ", ip: " + self.ip + ", port: " + str(self.port) + ", config: " + str(self.config)
    def __repr__(self):
        return self.__str__()
    def request_heartbeat(self, sock, enable_heartbeat, enable_runtime_stats):
        flags = 0
        if enable_heartbeat:
            flags |= 1
        if enable_runtime_stats:
            flags |= 2
        print("Requesting heartbeat", "enable_heartbeat", enable_heartbeat, "enable_runtime_stats", enable_runtime_stats)
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
        print("Sending config all", len(pkt_cfg_all))
        sock.sendto(pkt_cfg_all, (self.ip, self.port))
        return sock

    def send_heartbeat(self, sock):
        pkt_heartbeat = bytearray(
            packet_hdr.pack(PKT_TYPE_HEARTBEAT, heartbeat_message.size)
        )
        pkt_heartbeat += heartbeat_message.pack(0, 0)
        print("Sending heartbeat", len(pkt_heartbeat))
        sock.sendto(pkt_heartbeat, (self.ip, self.port))
        return sock

    def load_config(self, sock):
        pkt_read_config = bytearray(
            packet_hdr.pack(PKT_TYPE_READ_CONFIG, 0)
        )
        print("Requesting config")
        sock.sendto(pkt_read_config, (self.ip, self.port))
        time_start = time.time()
        while True:
            #read pack packets, (possibly some we are not interested in)
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                if time.time() - time_start > 5.0:
                    print("Timeout reading config")
                    break
                continue
            except IOError as msg:
                print(msg)
                continue
            except KeyboardInterrupt:
                return False
            
            if len(data) > config_message.size and socket.inet_pton(sock.family, addr[0]) == self.ip_pton:
                pkt_type, _ = packet_hdr.unpack(data[:4])
                if pkt_type == PKT_TYPE_CONFIG_ALL:
                    version, magic, rmt_clock_hz, dimensions_width, dimensions_height, heartbeat_interval_frames, strips_enable, debug_flags, frame_rate, max_brightness, is_rgbw = config_message_all.unpack(data[4:])
                    print("Received config all", "version", version, "magic", magic, "rmt_clock_hz", rmt_clock_hz, "dimensions_width", dimensions_width, "dimensions_height", dimensions_height, "heartbeat_interval_frames", heartbeat_interval_frames, "strips_enable", strips_enable, "debug_flags", debug_flags, "frame_rate", frame_rate, "max_brightness", max_brightness, "is_rgbw", is_rgbw)
                    self.config = LampConfig([dimensions_width, dimensions_height], is_rgbw, max_brightness, frame_rate, strips_enable, debug_flags, heartbeat_interval_frames, rmt_clock_hz)
                    print(self.config)
                    return True

        return False

    def gen_frame(self, sock, frame_id):
      time_since = frame_id / self.config.frame_rate
      led_frame_data = process_pixels_rainbow_circle(time_since, frame_id, self.program_state, self.config.dimensions)

      size_led_frame_data = len(led_frame_data)
      led_frame_packet = bytearray(
          packet_hdr.pack(PKT_TYPE_LED_FRAME, size_led_frame_data)
      )
      led_frame_packet += led_frame_data
      # send led frame packet
      sock.sendto(led_frame_packet, (self.ip, self.port))
      self.frames_sent += 1

    def get_frames_sent(self):
      return self.frames_sent 
    
    def handle_packet(self, sock, data, pkt_type):
      global g_num_frames_all_lamps
      if pkt_type == PKT_TYPE_HEARTBEAT:
          frame_id, buf_fill = heartbeat_message.unpack(data[4:])
          if frame_id == 0:
            print("Received first heartbeat from", self.name, "frame_id", frame_id, "buf_fill", buf_fill)
          num_frames_to_generate = self.config.heartbeat_interval_frames
          if buf_fill < 32:
            num_frames_to_generate += 1
          if buf_fill > 40:
            num_frames_to_generate -= 1

          for _ in range(num_frames_to_generate):
              self.gen_frame(sock, frame_id)
              frame_id += 1
          g_num_frames_all_lamps += num_frames_to_generate
      elif pkt_type == PKT_TYPE_RUNTIME_STATS:
          # put stuff in a dictionary
          lamp_stats = {}
          lamp_stats["read_index"], lamp_stats["write_index"], lamp_stats["num_callbacks"], lamp_stats["num_frames"], lamp_stats["num_buffer_underrun"], lamp_stats["num_heartbeats"], lamp_stats["fps_actual"], lamp_stats["fps_callbacks"] = runtime_stats_message.unpack(data[4:])
          self.stats = lamp_stats
          print(self.stats)
      elif pkt_type == PKT_TYPE_CONFIG_ALL:
          version, magic, rmt_clock_hz, dimensions_width, dimensions_height, heartbeat_interval_frames, strips_enable, debug_flags, frame_rate, max_brightness, is_rgbw = config_message_all.unpack(data[4:])
          print("Received config all", "version", version, "magic", magic, "rmt_clock_hz", rmt_clock_hz, "dimensions_width", dimensions_width, "dimensions_height", dimensions_height, "heartbeat_interval_frames", heartbeat_interval_frames, "strips_enable", strips_enable, "debug_flags", debug_flags, "frame_rate", frame_rate, "max_brightness", max_brightness, "is_rgbw", is_rgbw)
          self.config = LampConfig([dimensions_width, dimensions_height], is_rgbw, max_brightness, frame_rate, strips_enable, debug_flags, heartbeat_interval_frames, rmt_clock_hz)
          print(self.config)

def process_pixels_rainbow_circle(time_since, frame_id, state, dimensions):
    global ledrand
    LED_NUM_STRIPS = dimensions[0]
    LED_STRIP_NUM_LEDS_PER_STRIP = dimensions[1]
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPS * LED_STRIP_NUM_LEDS_PER_STRIP, 0
    )
    speed2 = 33
    f = math.cos(time_since * speed2 * math.pi / 180) * 0.5 + 0.5
    f2 = math.cos(time_since * speed2 *1.1* math.pi / 180) * 0.5 + 0.5
    f3 = math.cos(time_since * speed2 *1.01* math.pi / 180) * 0.5 + 0.5
    f4 = math.cos(time_since * speed2 *1.05* math.pi / 180) * 0.5 + 0.5
    for strip in range(LED_NUM_STRIPS):
        for led in range(LED_STRIP_NUM_LEDS_PER_STRIP):
            y0to1 = led / (LED_STRIP_NUM_LEDS_PER_STRIP - 1)
            x0to1 = 0.5 if LED_NUM_STRIPS == 1 else strip / (LED_NUM_STRIPS - 1)
            # use distance from center to determine a continuous gradient circle
            dst = math.sqrt((x0to1 - 0.5) ** 2 + (y0to1 - 0.5) ** 2)
            hue = dst * 360 * (0.3 + 5.7 * f)
            speed = 360.0
            idx_palette = int(time_since * 0.1) % len(palettes)
            if idx_palette % 2 == 0:
              speed = -speed
            
            hue -= time_since * speed
            hue = hue % 360

            palette = palettes[idx_palette]
            val = pal(hue/360.0, palette[0], palette[1], palette[2], palette[3])
            # # calculate a proper white value from the RGB values
            luma = val[0] * 0.299 + val[1] * 0.587 + val[2] * 0.114
            col_normalized = val + [luma*2] # add white channel
            # col_normalized = val + [1] # add white channel

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
            # noise_scale = (1.0/255) * 0.5
            # for i in range(4):
            #     col_normalized[i] += ledrand.uniform(-noise_scale * 0.5, noise_scale * 0.5)
            #     col_normalized[i] = min(1.0, max(0.0, col_normalized[i]))

            r, g, b, w = [int(col_normalized[i] * 255) for i in range(4)]
            led_frame_data += struct.pack("BBBB", int(b), int(g), int(r), int(w))
    return led_frame_data

# on and off in slow fashion
def process_2s_second_black_white(time_since, frame_id, state, dimensions):
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
          print(msg)
          continue
      except KeyboardInterrupt:
          return
      if len(data) > heartbeat_message.size:
          pkt_type, _ = packet_hdr.unpack(data[:4])
          # get ip as string and port
          ip = addr[0]
          # get lamp by ip
          lamp = None
          ip_pton = socket.inet_pton(sock.family, ip)
          for l in server["lamps"]:
              # if l.ip == ip:
              if ip_pton == l.ip_pton:
                  lamp = l
                  break
          if lamp == None:
            print("Error: received heartbeat from unknown lamp", ip)
            continue
          lamp.handle_packet(sock, data, pkt_type)
      else:
          print("Received packet with invalid size", len(data))

def is_valid_ipv6_address(address):
  try:
      socket.inet_pton(socket.AF_INET6, address)
  except socket.error:  # not a valid address
      return False
  return True
def main(args = None):
    global g_time_start, g_num_frames_all_lamps
    if args is None:
        print("Args is None")
        return
    if args.ips is None or len(args.ips) == 0:
        print("Error: IP address required")
        return
    print ("IPs: ", args.ips)
    port = args.port if args.port is not None else LED_CONTROLLER_UDP_PORT
    is_ip6 = is_valid_ipv6_address(args.ips[0])
    if args.ipv6 and not is_ip6:
        print("Error: invalid IPv6 address")
        return
    if not args.ipv6 and is_ip6:
        args.ipv6 = True

    # create socket
    if args.ipv6:
      sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    else:
      sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', LED_CONTROLLER_UDP_PORT))

    # create dict with server data
    server = {
        "socket": sock,
        "port": port,
        "lamps": []
    }
    for lamp_ip in args.ips:
      lamp = Lamp("Lamp", lamp_ip, port, socket.inet_pton(sock.family, lamp_ip))
      # disable heartbeat and runtime stats
      lamp.request_heartbeat(sock, enable_heartbeat=False, enable_runtime_stats=False)

      if not lamp.load_config(sock):
          print("Error: could not load config")
          return

      print("Lamp current configuration", lamp)

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

      print("Lamp new configuration", lamp)
      lamp.send_config(sock)
      server["lamps"].append(lamp)


    g_time_start = time.time()

    # create second thread to receive heartbeats using the same socket
    thread = threading.Thread(target=recv_udp_packets, args=(server,), daemon=True)
    thread.start()
    for lamp in server["lamps"]:
      lamp.request_heartbeat(sock, enable_heartbeat=True, enable_runtime_stats=False)

    time_last_fps = time.time()
    avg_fps = lamp.config.frame_rate
    try:
      while True:
          try:
            tmNow = time.time()
            time.sleep(1)
            if g_num_frames_all_lamps > 4 and tmNow - time_last_fps > 5.0:
                current_fps = g_num_frames_all_lamps / (tmNow - time_last_fps)
                avg_fps = 0.5 * avg_fps + 0.5 * current_fps
                print("FPS:", "Target", lamp.config.frame_rate, "Current", current_fps, "Avg", avg_fps)
                g_num_frames_all_lamps = 0
                time_last_fps = tmNow
          except KeyboardInterrupt:
              break
    finally:
      if sock != None:
          for lamp in server["lamps"]:
              lamp.request_heartbeat(sock, enable_heartbeat=False, enable_runtime_stats=False)
          # sleep 0.5s
          time.sleep(0.5)
      sock.close()
      server["socket"] = None
    


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='RGB LED Controller')
  parser.add_argument('--ips', type=str, nargs='+', help='List of IP addresses of the LED controller')
  parser.add_argument('--port', type=int, help='UDP port of the LED controller', default=LED_CONTROLLER_UDP_PORT)
  parser.add_argument('-6', '--ipv6', action='store_true', help='Use IPv6')
  parser.add_argument('-f', '--fps', type=int, help='Frames per second')
  parser.add_argument('-b', '--brightness', type=int, help='Max brightness')
  parser.add_argument('-i', '--heartbeat-interval', type=int, help='Heartbeat interval')
  parser.add_argument('-s', '--strips-enabled', type=lambda x: int(x, 16), help='Stripes enable bitmask')
  parser.add_argument('-d', '--dimensions', type=int, nargs=2, help='Dimensions width height')
  parser.add_argument('-c', '--rmt-clock-hz', type=int, help='RMT clock Hz')
  parser.add_argument('--rgbw', type=int, help='RGBW mode (0 or 1)')
  parser.add_argument('--dbg', '--debug-bits', type=int, help='Set debug bits')
  args = parser.parse_args()
  main(args)
