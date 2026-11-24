import time
import math
import esp32
import hardware
from machine import Pin
import M5
# ============================================================
# 200-Line Copy-Paste Atom Gadget: PIXMOB IR Controller
# UIFlow2 MicroPython / AtomS3 family
#
# Supported:
#   AtomS3      : LCD + IMU + IR(GPIO4)
#   AtomS3R     : LCD + IMU + IR(GPIO47)
#   AtomS3 Lite : RGB LED + IR(GPIO4)
#
# Operations:
#   Short press : Send current color to PIXMOB
#   Long press  : Cycle through colors
#   Tilt       : AtomS3 / AtomS3R directly select color by tilt direction
#
# Note:
#   After experimenting with setCallback in REPL,
#   reset the device once before execution to avoid ghost callbacks.
# ============================================================
VERSION = "pixmob-atom-simple-canvas-board-irpin-flash-20260616"
# ============================================================
# PIXMOB IR Payloads
# ============================================================
P_WAKE = (700, 6300)
# Base color payloads derived from danielweidman/flipper-pixmob-ir-codes / PixMob_main.ir
# License: MIT License
# Copyright (c) 2022 Dani Weidman
# This is not logic but "IR signal data".
# Since it is not meant to be read, treat it as data definition for the 200-Line Logic.
P_RED = (1400,1400,700,700,700,1400,700,2800,700,2100,1400,700,700,700,700,1400,1400,2800,1400,2800,700)
P_GRN = (1400,1400,700,700,700,700,1400,2800,700,1400,700,1400,700,1400,700,1400,1400,2800,1400,2800,700)
P_YEL = (1400,1400,700,700,700,700,1400,2800,700,2100,700,700,700,1400,700,1400,1400,2800,1400,2800,700)
P_BLU = (700,700,700,2100,1400,700,700,2800,700,1400,700,700,700,1400,1400,700,700,1400,700,700,700,700,700,700,700,2100,700)
P_MAG = (700,700,700,700,1400,1400,1400,2800,700,2100,1400,2100,700,700,700,700,1400,2100,700,700,700,2100,700)
P_ORG = (700,700,700,700,1400,1400,1400,2800,700,2100,700,700,700,1400,700,700,700,1400,700,2100,1400,2800,700)
P_WHT = (700,700,700,2100,1400,700,700,2800,700,1400,700,700,700,2800,1400,1400,700,2100,700,1400,700,1400,700)
P_CYN = (700,700,700,700,1400,1400,1400,2800,700,1400,700,700,1400,1400,700,1400,1400,2800,700,700,700,2100,700)
# ============================================================
# Constants
# ============================================================
DEFAULT_IR_PIN = 4
GAP_MS = 40
REPEAT = 3
# PIXMOB X2 has been confirmed to illuminate without wake, so use the known successful condition as default.
# If an individual unit is unstable, try setting to True.
USE_WAKE = False
# The built-in RGB LED of the AtomS3 Lite is bright, so reduce the RGB values to dim the display.
# Dimming the values themselves makes the behavior easier to read than using hardware.RGB.set_brightness().
LED_SCALE = 0.30
# For AtomS3 Lite, brighten only during transmission to show difference from normal display.
SEND_LED_SCALE = 1.00
# Tilt detection threshold.
# Do not change color when near horizontal to avoid unintended color changes.
TILT_THRESHOLD2 = 0.35 * 0.35
TILT_POLL_MS = 50
COLOR_LOCK_MS = 220
# On actual AtomS3, IMU x/y are opposite to the intuitive screen direction, so correct them.
# Adjust here if the orientation differs on AtomS3R.
IMU_X_SIGN = -1
IMU_Y_SIGN = -1
# The front/back direction is a bit hard to select, so strengthen the y direction slightly.
TILT_Y_GAIN = 1.25
# For fillArc drawing angle.
# If the arc appears upside down, change to 1.
ARC_Y_SIGN = -1
# drawCenterString is horizontally centered.
# Since y is close to the text drawing start position, 60px slightly above the center of a 128px screen looks best.
TEXT_Y = 60
# ============================================================
# Data Tables
# ============================================================
COLORS = (
    ("RED",     (255,   0,   0), P_RED),
    ("ORANGE",  (255,  80,   0), P_ORG),
    ("YELLOW",  (255, 255,   0), P_YEL),
    ("GREEN",   (  0, 255,   0), P_GRN),
    ("BLUE",    (  0,   0, 255), P_BLU),
    ("CYAN",    (  0, 255, 255), P_CYN),
    ("MAGENTA", (255,   0, 255), P_MAG),
    ("WHITE",   (255, 255, 255), P_WHT),
)
# name, dx, dy, color_index
# Create the judgment vector and the on-screen arc layout from the same definition.
# This makes it harder for the "tilt direction" and the "color layout on screen" to misalign.
TILT_DIRECTIONS = (
    ("FRONT",     0.0,      1.0,      0),
    ("FRONT_R",   0.7071,   0.7071,   1),
    ("RIGHT",     1.0,      0.0,      2),
    ("BACK_R",    0.7071,  -0.7071,   3),
    ("BACK",      0.0,     -1.0,      4),
    ("BACK_L",   -0.7071,  -0.7071,   5),
    ("LEFT",     -1.0,      0.0,      6),
    ("FRONT_L",  -0.7071,   0.7071,   7),
)
# enum_name, has_display, has_imu, ir_pin, label
# The AtomS3R's built-in IR LED uses a different GPIO from the AtomS3/AtomS3 Lite.
# Spread board differences into a visible correspondence table instead of scattering them in if statements.
BOARD_FEATURES = (
    ("M5AtomS3",     True,  True,  4,  "M5AtomS3"),
    ("M5AtomS3R",    True,  True,  47, "M5AtomS3R"),
    ("M5AtomS3Lite", False, False, 4,  "M5AtomS3Lite"),
)
# ============================================================
# State
# ============================================================
has_display = False
has_imu = False
led_available = False
ir_pin = DEFAULT_IR_PIN
current_color_idx = 0
pending_ir_send = False
pending_next_color = False
last_tilt_poll_ms = 0
last_color_change_ms = 0
rmt = None
gfx = None
rgb_led = None
# ============================================================
# Helpers
# ============================================================
def rgb_to_int(rgb):
    return (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]
def dim_rgb(rgb, scale=LED_SCALE):
    # The AtomS3 Lite's RGB LED is still sufficiently noticeable even when dimmed.
    # Reduce only the brightness while preserving the hue.
    r, g, b = rgb
    return (
        int(r * scale),
        int(g * scale),
        int(b * scale),
    )
def set_rgb_led(rgb, scale=LED_SCALE):
    # For AtomS3 Lite.
    # Since the built-in LED is WS2812/NeoPixel based, use hardware.RGB in UIFlow2 MicroPython.
    if not led_available:
        return
    color = rgb_to_int(dim_rgb(rgb, scale))
    try:
        rgb_led.fill_color(color)
        rgb_led.write()
    except Exception as e:
        print("RGB LED update failed:", e)
# ============================================================
# Display UI
# ============================================================
def draw_arc_segment(dx, dy, color_int, selected):
    # Draw an 8-direction ring using fillArc.
    # It is shorter than placing small circles and looks more natural.
    angle = int(math.atan2(dy * ARC_Y_SIGN, dx) * 57.2958)
    inner, outer = (42, 56) if selected else (48, 52)
    gfx.fillArc(
        64, 64,
        inner, outer,
        angle - 18,
        angle + 18,
        color_int
    )
def update_display_or_led():
    c_name, rgb, _ = COLORS[current_color_idx]
    if not has_display:
        set_rgb_led(rgb)
        print("COLOR:", c_name)
        return
    # Drawing directly to Display may show intermediate steps (fillScreen → arcs → text).
    # Draw on a Canvas first and then push to suppress flickering.
    gfx.fillScreen(0x000000)
    for _name, dx, dy, c_idx in TILT_DIRECTIONS:
        _dot_name, dot_rgb, _pulses = COLORS[c_idx]
        draw_arc_segment(
            dx,
            dy,
            rgb_to_int(dot_rgb),
            c_idx == current_color_idx
        )
    gfx.drawCenterString(c_name, 64, TEXT_Y)
    gfx.push(0, 0)
def show_sending():
    # During transmission, do not display text; briefly fill the entire screen with the selected color.
    # For AtomS3 Lite, illuminate brighter than normal display to show the transmission timing.
    _c_name, rgb, _pulses = COLORS[current_color_idx]
    if has_display:
        gfx.fillScreen(rgb_to_int(rgb))
        gfx.push(0, 0)
    else:
        set_rgb_led(rgb, SEND_LED_SCALE)
# ============================================================
# Device / Board
# ============================================================
def detect_device_features():
    global has_display, has_imu, ir_pin
    board = M5.getBoard()
    for enum_name, disp, imu, pin, label in BOARD_FEATURES:
        if board == getattr(M5.BOARD, enum_name, None):
            has_display = disp
            has_imu = imu
            ir_pin = pin
            print("Board:", label)
            print(
                "has_display:", has_display,
                "has_imu:", has_imu,
                "ir_pin:", ir_pin
            )
            return
    # For unknown Atom-like compatible devices, assume IR GPIO4 equivalent to AtomS3.
    has_display = False
    has_imu = False
    ir_pin = DEFAULT_IR_PIN
    print("Board: unknown:", board)
    print(
        "has_display:", has_display,
        "has_imu:", has_imu,
        "ir_pin:", ir_pin
    )
def init_gfx():
    global gfx
    if has_display:
        # 128x128 off-screen drawing area.
        # Since it is a small Canvas matching the AtomS3 screen size, memory load is light.
        gfx = M5.Display.newCanvas(128, 128)
def init_led():
    global led_available, rgb_led
    if has_display:
        # For LCD-equipped models, use screen UI, so RGB LED is not used.
        led_available = False
        return
    try:
        rgb_led = hardware.RGB()
        led_available = len(rgb_led) > 0
        print("RGB LED count:", len(rgb_led))
    except Exception as e:
        led_available = False
        rgb_led = None
        print("RGB LED unavailable:", e)
# ============================================================
# Tilt Logic
# ============================================================
def read_tilt_color_index():
    if not has_imu:
        return None
    try:
        x, y, z = M5.Imu.getAccel()
    except Exception as e:
        print("IMU read failed:", e)
        return None
    # Correct so that screen direction matches hand movement.
    x *= IMU_X_SIGN
    y *= IMU_Y_SIGN
    y *= TILT_Y_GAIN
    # Do not change color when near horizontal.
    # Prioritize the remote-control feel of "pressing while level sends the current color."
    if x * x + y * y < TILT_THRESHOLD2:
        return None
    best_score = -999
    best_idx = current_color_idx
    # Choose the direction with the maximum dot product with each direction vector.
    # Easier to swap direction definitions than writing eight if/elif branches.
    for _name, dx, dy, c_idx in TILT_DIRECTIONS:
        score = x * dx + y * dy
        if score > best_score:
            best_score = score
            best_idx = c_idx
    return best_idx
def update_tilt_selection():
    global current_color_idx
    global last_tilt_poll_ms, last_color_change_ms
    now = time.ticks_ms()
    # Reading the IMU every loop causes fine fluctuations in the display, so thin it out a bit.
    if time.ticks_diff(now, last_tilt_poll_ms) < TILT_POLL_MS:
        return
    last_tilt_poll_ms = now
    idx = read_tilt_color_index()
    if idx is None or idx == current_color_idx:
        return
    # Lock color changes briefly to prevent rapid switching near boundaries.
    if time.ticks_diff(now, last_color_change_ms) < COLOR_LOCK_MS:
        return
    current_color_idx = idx
    last_color_change_ms = now
    update_display_or_led()
# ============================================================
# Button callbacks
# ============================================================
def on_click(state):
    global pending_ir_send
    # Do not execute IR transmission inside the callback.
    # Offload heavy processing to the main loop to avoid clogs around M5.update().
    pending_ir_send = True
def on_hold(state):
    global pending_next_color
    # Long press cycles colors.
    # Enables basic operation even on models without IMU like AtomS3 Lite.
    pending_next_color = True
def next_color():
    global current_color_idx
    current_color_idx = (current_color_idx + 1) % len(COLORS)
    update_display_or_led()
# ============================================================
# IR
# ============================================================
def init_rmt():
    global rmt
    # Let RMT handle the 38kHz carrier.
    # On the Python side, only pass the raw pulse train for PIXMOB.
    print("IR pin:", ir_pin)
    rmt = esp32.RMT(
        0,
        pin=Pin(ir_pin),
        resolution_hz=1_000_000,
        tx_carrier=(38000, 50, 1),
        idle_level=0
    )
def send_ir_signal():
    _c_name, _rgb, pulses = COLORS[current_color_idx]
    payload = P_WAKE + pulses if USE_WAKE else pulses
    show_sending()
    for _ in range(REPEAT):
        rmt.write_pulses(payload, True)
        time.sleep_ms(GAP_MS)
    update_display_or_led()
# ============================================================
# Main
# ============================================================
M5.begin()
print("VERSION:", VERSION)
detect_device_features()
init_gfx()
init_led()
init_rmt()
# After experimenting in REPL, reset once before execution to avoid ghost callbacks.
M5.BtnA.setCallback(type=M5.BtnA.CB_TYPE.WAS_CLICKED, cb=on_click)
M5.BtnA.setCallback(type=M5.BtnA.CB_TYPE.WAS_HOLD, cb=on_hold)
update_display_or_led()
while True:
    M5.update()
    update_tilt_selection()
    if pending_ir_send:
        pending_ir_send = False
        send_ir_signal()
    if pending_next_color:
        pending_next_color = False
        next_color()
    time.sleep_ms(10)