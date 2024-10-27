import socket
import threading
import time
import struct
import random
import sys
import math

LED_NUM_STRIPES = 10
LED_NUM_LEDS_PER_STRIPE = 30

# packet types
PKT_TYPE_HEARTBEAT = 0
PKT_TYPE_CONFIG = 1
PKT_TYPE_LED_FRAME = 2

# config types
CFG_ID_MAX_BRIGHTNESS = 1
CFG_ID_FRAME_RATE = 2
CFG_ID_STRIPES_ENABLE = 3
CFG_ID_ENABLE_DEBUG = 4
CFG_ID_HEARTBEAT_INTERVAL_FRAMES = 5
CFG_NUM_CONFIG = 6

# network settings
# LED_CONTROLLER_IP = "192.168.188.10"
LED_CONTROLLER_IP = "fe80:0000:0000:0000:b2a7:32ff:fe16:cc24"
# LED_CONTROLLER_IP = "fe80:0000:0000:0000:da13:2aff:fe7d:9eac"
# LED_CONTROLLER_IP = "192.168.188.60"
LED_CONTROLLER_UDP_PORT = 54321
USE_IPV6 = False
USE_IPV6 = True
# packet defs relating to https://github.com/nidefawl/esp32-rgb/blob/main/main/rgb-network-types.h
# packet header, u16 packet type, u16 packet size
packet_hdr = struct.Struct("<H H")

# heartbeat message, u64 frame id, u64 fps, u64 heartbeat interval
heartbeat_message = struct.Struct("<Q Q Q")

# config message, u32 config id, u32 config value
config_message = struct.Struct("<I I")

# led frame message: u32 frame size (in number of LEDS), u32 frame offset (in number of LEDS), then led data (LED data is 4 bytes per LED (WRGB))
# pointer to frame data is not included in struct, it is added later
led_frame_message = struct.Struct("<I I")


FPS = 80
MAX_BRIGHTNESS = 255
HEARTBEAT_INTERVAL = 20
frame_id = 0
time_start = 0
program_state = {}
num_frames = 0
USE_HEARTBEAT_FRAME_GEN = True

# vec3 pal( in float t, in vec3 a, in vec3 b, in vec3 c, in vec3 d )
# {
#     return a + b*cos( 6.28318*(c*t+d) );
# }
# python implementation of the above GLSL function

def pal(t, a, b, c, d):
    return [a[i] + b[i] * math.cos(6.28318 * (c[i] * t + d[i])) for i in range(3)]

def test_pal():
    # vec3                col = pal( p.x, vec3(0.5,0.5,0.5),vec3(0.5,0.5,0.5),vec3(1.0,1.0,1.0),vec3(0.0,0.33,0.67) );
    px = 0.5
    val = pal(px, [0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [1.0, 1.0, 1.0], [0.0, 0.33, 0.67])
    print(val)

def process_static_black(time_since, frame_id, state):
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPES * LED_NUM_LEDS_PER_STRIPE, 0
    )

    for stripe in range(LED_NUM_STRIPES):
        for led in range(LED_NUM_LEDS_PER_STRIPE):
            r = 0
            g = 0
            b = 0
            w = 0
            # add RGB values to led_frame_data
            led_frame_data += struct.pack("BBBB", int(b), int(g), int(r), int(w))
    return led_frame_data

ledrand = random.Random()
palettes = []
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[1.0,1.0,1.0],[0.0,0.33,0.67]))
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[1.0,1.0,1.0],[0.0,0.10,0.20]))
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[1.0,1.0,1.0],[0.3,0.20,0.20]))
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[1.0,1.0,0.5],[0.8,0.90,0.30]))
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[1.0,0.7,0.4],[0.0,0.15,0.20]))
palettes.append(([0.5,0.5,0.5],[0.5,0.5,0.5],[2.0,1.0,0.0],[0.5,0.20,0.25]))
palettes.append(([0.8,0.5,0.4],[0.2,0.4,0.2],[2.0,1.0,1.0],[0.0,0.25,0.25]))

def process_pixels_rainbow_circle(time_since, frame_id, state):
    global ledrand
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPES * LED_NUM_LEDS_PER_STRIPE, 0
    )
    if not "polarity" in state:
        state["polarity"] = 1
    speed2 = 33
    f = math.cos(time_since * speed2 * math.pi / 180) * 0.5 + 0.5
    f2 = math.cos(time_since * speed2 *1.1* math.pi / 180) * 0.5 + 0.5
    f3 = math.cos(time_since * speed2 *1.01* math.pi / 180) * 0.5 + 0.5
    f4 = math.cos(time_since * speed2 *1.05* math.pi / 180) * 0.5 + 0.5
    for stripe in range(LED_NUM_STRIPES):
        for led in range(LED_NUM_LEDS_PER_STRIPE):
            y0to1 = led / (LED_NUM_LEDS_PER_STRIPE - 1)
            x0to1 = stripe / (LED_NUM_STRIPES - 1)
            # use distance from center to determine a continuous gradient circle
            dst = math.sqrt((x0to1 - 0.5) ** 2 + (y0to1 - 0.5) ** 2)
            hue = dst * 360 *  0.5
            speed = 100.0
            hue += time_since * speed
            hue = hue % 360
            idx_palette = int(time_since * 0.1) % len(palettes)
            palette = palettes[idx_palette]
            val = pal(hue/360.0, palette[0], palette[1], palette[2], palette[3])
            col_normalized = val + [1.0] # add white channel

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
            noise_scale = (1.0/255) * 0.5
            for i in range(4):
                col_normalized[i] += ledrand.uniform(-noise_scale * 0.5, noise_scale * 0.5)
                col_normalized[i] = min(1.0, max(0.0, col_normalized[i]))

            r, g, b, w = [int(col_normalized[i] * 255) for i in range(4)]
            # r =  b = w = 0
            led_frame_data += struct.pack("BBBB", int(b), int(g), int(r), int(w))
    return led_frame_data

# process_pixels_white_noise generates a white noise pattern
def process_pixels_white_noise(time_since, frame_id, state):
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPES * LED_NUM_LEDS_PER_STRIPE, 0
    )

    for stripe in range(LED_NUM_STRIPES):
        for led in range(LED_NUM_LEDS_PER_STRIPE):
            r = random.randint(0, 255)
            g = random.randint(0, 255)
            b = random.randint(0, 255)
            w = random.randint(0, 255)
            # add RGB values to led_frame_data
            led_frame_data += struct.pack("BBBB", int(b), int(g), int(r), int(w))
    return led_frame_data
def gen_frame(sock, frame_id, program_state):
    # tmNow = time.time()
    time_since = (frame_id / FPS)
    led_frame_data = process_pixels_rainbow_circle(time_since, frame_id, program_state)

    size_led_frame_data = len(led_frame_data)
    led_frame_packet = bytearray(
        packet_hdr.pack(PKT_TYPE_LED_FRAME, size_led_frame_data)
    )
    led_frame_packet += led_frame_data
    # send led frame packet
    sock.sendto(led_frame_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))

# second thread function to recv heartbeats
def recv_heartbeats(*args):
    global frame_id, program_state, num_frames
    sock = args[0]
    while 1:
        # recvfrom UDP IP and port
        data, addr = sock.recvfrom(1024)
        if len(data) > heartbeat_message.size:
            pkt_type, pkt_size = packet_hdr.unpack(data[:4])
            if pkt_type == PKT_TYPE_HEARTBEAT and USE_HEARTBEAT_FRAME_GEN:
                frame_id, fps, heartbeat_interval = heartbeat_message.unpack(data[4:])
                # print("Received heartbeat", frame_id, fps, heartbeat_interval)
                # generate heartbeat_interval frames
                for i in range(heartbeat_interval):
                    gen_frame(sock, frame_id, program_state)
                    num_frames += 1
                    frame_id += 1
        else:
            print("Received packet with invalid size", len(data))
def main():
    global frame_id, time_start, num_frames, program_state
    test_pal()
    # create socket
    if USE_IPV6:
        sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # create second thread to receive heartbeats using the same socket
    thread = threading.Thread(target=recv_heartbeats, args=(sock,))
    thread.start()

    # heartbeat has no function right now, send it anyways
    heartbeat_packet = packet_hdr.pack(PKT_TYPE_HEARTBEAT, heartbeat_message.size)
    heartbeat_packet += heartbeat_message.pack(frame_id, FPS, 0)
    sock.sendto(heartbeat_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))

    # configure max brightness
    config_packet = packet_hdr.pack(PKT_TYPE_CONFIG, config_message.size)
    config_packet += config_message.pack(CFG_ID_MAX_BRIGHTNESS, MAX_BRIGHTNESS)
    sock.sendto(config_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))
    
    # configure frame rate
    config_packet = packet_hdr.pack(PKT_TYPE_CONFIG, config_message.size)
    config_packet += config_message.pack(CFG_ID_FRAME_RATE, FPS)
    sock.sendto(config_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))

    # configure 10 active strips bitmask
    strip_mask = 0x3FF
    # strip_mask = 0x3F # 6 stripes
    config_packet = packet_hdr.pack(PKT_TYPE_CONFIG, config_message.size)
    config_packet += config_message.pack(CFG_ID_STRIPES_ENABLE, strip_mask)
    sock.sendto(config_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))

    # configure debug mode
    config_packet = packet_hdr.pack(PKT_TYPE_CONFIG, config_message.size)
    config_packet += config_message.pack(CFG_ID_ENABLE_DEBUG, int(1))
    # sock.sendto(config_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))

    # configure heartbeat interval
    config_packet = packet_hdr.pack(PKT_TYPE_CONFIG, config_message.size)
    config_packet += config_message.pack(CFG_ID_HEARTBEAT_INTERVAL_FRAMES, HEARTBEAT_INTERVAL)
    if USE_HEARTBEAT_FRAME_GEN:
      sock.sendto(config_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))


    time_start = time.time()
    br = 1.0 * 255
    set_fullscreen_color(sock, br, br, br)
    time_last_fps = time.time()
    time_last_frame = time.time()
    avg_fps = FPS
    while 1:
        tmNow = time.time()
        tmDelta = tmNow - time_last_frame
        if not USE_HEARTBEAT_FRAME_GEN:
            if tmDelta < (1.0/FPS) - 0.0008:
                time.sleep(0.001)
                continue
            time_since = tmNow - time_start
            led_frame_data = process_pixels_rainbow_circle(time_since, frame_id, program_state)

            size_led_frame_data = len(led_frame_data)
            led_frame_packet = bytearray(
                packet_hdr.pack(PKT_TYPE_LED_FRAME, size_led_frame_data)
            )
            led_frame_packet += led_frame_data
            # send led frame packet
            sock.sendto(led_frame_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))
        else:
            time.sleep(1)
        time_last_frame = tmNow
        frame_id += 1
        if num_frames > 4 and tmNow - time_last_fps > 5.0:
            current_fps = num_frames / (tmNow - time_last_fps)
            avg_fps = 0.5 * avg_fps + 0.5 * current_fps
            print("FPS Target", FPS, "- Current FPS", current_fps, "- Avg FPS", avg_fps)
            num_frames = 0
            time_last_fps = tmNow


def set_fullscreen_color(sock, r, g, b):
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPES * LED_NUM_LEDS_PER_STRIPE, 0
    )
    for stripe in range(LED_NUM_STRIPES):
        for led in range(LED_NUM_LEDS_PER_STRIPE):
            led_frame_data += struct.pack("BBBB", int(r), int(g), int(b), 0)

    size_led_frame_data = len(led_frame_data)
    led_frame_packet = bytearray(
        packet_hdr.pack(PKT_TYPE_LED_FRAME, size_led_frame_data)
    )
    led_frame_packet += led_frame_data
    sock.sendto(led_frame_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))


if __name__ == "__main__":
    main()
