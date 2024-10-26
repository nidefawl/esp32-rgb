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
CFG_NUM_CONFIG = 4

# network settings
LED_CONTROLLER_IP = "192.168.188.10"
LED_CONTROLLER_UDP_PORT = 54321

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
HEARTBEAT_INTERVAL = 8
frame_id = 0
time_start = 0
program_state = {}
num_frames = 0
USE_HEARTBEAT_FRAME_GEN = True

def process_pixels_rainbow_circle(time_since, frame_id, state):
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPES * LED_NUM_LEDS_PER_STRIPE, 0
    )

    for stripe in range(LED_NUM_STRIPES):
        for led in range(LED_NUM_LEDS_PER_STRIPE):
            y0to1 = led / (LED_NUM_LEDS_PER_STRIPE - 1)
            x0to1 = stripe / (LED_NUM_STRIPES - 1)
            # create a rainbow cirlce pattern
            # use distance from center to determine a continuous rainbow gradient circle
            dst = math.sqrt((x0to1 - 0.5) ** 2 + (y0to1 - 0.5) ** 2)
            # dst *= 1.5
            hue = dst * 360 * 1.25
            speed = 300
            hue -= time_since * speed
            hue = -hue % 360
            # convert hue to RGB
            r = 0
            g = 0
            b = 0
            w = 0

            if hue < 60:
                r = 255
                g = hue / 60 * 255
            elif hue < 120:
                r = (120 - hue) / 60 * 255
                g = 255
            elif hue < 180:
                g = 255
                b = (hue - 120) / 60 * 255
            elif hue < 240:
                g = (240 - hue) / 60 * 255
                b = 255
            elif hue < 300:
                r = (hue - 240) / 60 * 255
                b = 255
            else:
                r = 255
                b = (360 - hue) / 60 * 255
            w = 255
            col_normalized = [r/255, g/255, b/255, w/255]
            # apply gamma correction
            gamma = 2.2
            for i in range(4):
                col_normalized[i] = col_normalized[i] ** gamma
            # add RGB values to led_frame_data
            r = int(col_normalized[0] * 255)
            g = int(col_normalized[1] * 255)
            b = int(col_normalized[2] * 255)
            w = int(col_normalized[3] * 255)
            led_frame_data += struct.pack("BBBB", int(b), int(g), int(r), int(w))
    return led_frame_data

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
    time_since = frame_id / FPS
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
        # data, addr = sock.recvfrom(1024)
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
    # create socket
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
    # br = 0.25 * 255
    # set_fullscreen_color(sock, br, br, br)
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
            avg_fps = 0.9 * avg_fps + 0.1 * current_fps
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
