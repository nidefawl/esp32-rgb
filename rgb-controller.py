import socket
import time
import struct
import random
import sys
import math

# A simple script that configures  the device and sends an animation

LED_CONTROLLER_IP = "192.168.188.10"
LED_CONTROLLER_UDP_PORT = 54321

LED_NUM_STRIPES = 10
LED_NUM_LEDS_PER_STRIPE = 30

PKT_TYPE_HEARTBEAT = 0
PKT_TYPE_CONFIG = 1
PKT_TYPE_LED_FRAME = 2

CFG_ID_MAX_BRIGHTNESS = 1
CFG_ID_FRAME_RATE = 2
CFG_ID_STRIPES_ENABLE = 3
CFG_NUM_CONFIG = 4
# packet defs relating to https://github.com/nidefawl/esp32-rgb/blob/main/main/rgb-network-types.h
packet_hdr = struct.Struct("<H H")
heartbeat_message = struct.Struct("<Q")
config_message = struct.Struct("<I I")
# led frame message: frame size, frame id, then led data. LED data is 4 bytes per LED (RGBW) and appended after 
led_frame_message = struct.Struct("<I I")


FPS = 100
MAX_BRIGHTNESS = 80
frame_id = 0

# creates a rainbow circle pattern
def process_pixels_rainbow_circle(time_since, frame_id, state):
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPES * LED_NUM_LEDS_PER_STRIPE, 0
    )

    for stripe in range(LED_NUM_STRIPES):
        for led in range(LED_NUM_LEDS_PER_STRIPE):
            x0to1 = led / (LED_NUM_LEDS_PER_STRIPE - 1)
            y0to1 = stripe / (LED_NUM_STRIPES - 1)
            # create a rainbow cirlce pattern
            # use distance from center to determine a continuous rainbow gradient circle
            dst = math.sqrt((x0to1 - 0.5) ** 2 + (y0to1 - 0.5) ** 2)
            # dst *= 1.5
            hue = dst * 360
            speed = 100 * 3
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
            led_frame_data += struct.pack("BBBB", int(r), int(g), int(b), int(w))
    return led_frame_data

# creates a high frequency white noise pattern, which is just random colors
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
            led_frame_data += struct.pack("BBBB", int(r), int(g), int(b), int(w))
    return led_frame_data

def main():
    global frame_id
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # heartbeat has no function right now, send it anyways
    heartbeat_packet = packet_hdr.pack(PKT_TYPE_HEARTBEAT, heartbeat_message.size)
    heartbeat_packet += heartbeat_message.pack(random.randint(0, 1000000))
    sock.sendto(heartbeat_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))

    # configure the device
    config_packet = packet_hdr.pack(PKT_TYPE_CONFIG, config_message.size)
    config_packet += config_message.pack(CFG_ID_MAX_BRIGHTNESS, MAX_BRIGHTNESS)
    sock.sendto(config_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))
    config_packet = packet_hdr.pack(PKT_TYPE_CONFIG, config_message.size)
    config_packet += config_message.pack(CFG_ID_FRAME_RATE, FPS)
    sock.sendto(config_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))

    time_start = time.time()

    # set the color of the LEDs to a specific color
    set_fullscreen_color(sock, 133, 133, 133)

    state = {}
    while 1:
        time_since = time.time() - time_start
        led_frame_data = process_pixels_rainbow_circle(time_since, frame_id, state)
        # led_frame_data = process_pixels_white_noise(time_since, frame_id, state)

        size_led_frame_data = len(led_frame_data)
        led_frame_packet = bytearray(
            packet_hdr.pack(PKT_TYPE_LED_FRAME, size_led_frame_data)
        )
        led_frame_packet += led_frame_data

        sock.sendto(led_frame_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))

        # sleep according to the frame rate
        time.sleep(1.0 / FPS)
        frame_id += 1


def set_fullscreen_color(sock, r, g, b):
    led_frame_data = led_frame_message.pack(
        LED_NUM_STRIPES * LED_NUM_LEDS_PER_STRIPE, 0
    )
    for stripe in range(LED_NUM_STRIPES):
        for led in range(LED_NUM_LEDS_PER_STRIPE):
            led_frame_data += struct.pack("BBBB", r, g, b, 0)

    size_led_frame_data = len(led_frame_data)
    led_frame_packet = bytearray(
        packet_hdr.pack(PKT_TYPE_LED_FRAME, size_led_frame_data)
    )
    led_frame_packet += led_frame_data
    sock.sendto(led_frame_packet, (LED_CONTROLLER_IP, LED_CONTROLLER_UDP_PORT))


if __name__ == "__main__":
    main()
