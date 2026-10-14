from machine import Pin
from servo import Servo
import time, math

L1 = 3.0
L2 = 3.0
INTERP = 12
STEP = 0.03

DEBOUNCE_MS = 100
last_press = 0
switch_flag = False

SHAPES = {
    "heart": [
        (1.25, 3.24), (0.46, 4.14), (0.12, 4.76), (0.41, 5.21), (0.86, 5.26),
        (1.25, 4.87), (1.64, 5.26), (2.09, 5.21), (2.38, 4.76), (2.04, 4.14),
        (1.25, 3.24),
    ],
    "star": [
        (1.25, 5.32), (0.97, 4.52), (0.12, 4.5), (0.8, 3.99), (0.55, 3.18),
        (1.25, 3.66), (1.95, 3.18), (1.7, 3.99), (2.38, 4.5), (1.53, 4.52),
        (1.25, 5.32),
    ],
}
NAMES = ("heart", "star")

def interpolate(pts, n):
    out = []
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        for j in range(n):
            f = j / n
            out.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f))
    out.append(pts[-1])
    return out

def ik(x, y):
    c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    t2 = math.atan2(math.sqrt(1 - c2 * c2), c2)
    t1 = math.atan2(y, x) - math.atan2(L2 * math.sin(t2), L1 + L2 * math.cos(t2))
    return int(round(math.degrees(t1))), int(round(math.degrees(t2)))

def build(name):
    return [ik(x, y) for x, y in interpolate(SHAPES[name], INTERP)]

def button_handler(pin):
    global switch_flag, last_press
    now = time.ticks_ms()
    if time.ticks_diff(now, last_press) > DEBOUNCE_MS:
        last_press = now
        switch_flag = True

button_switch = Pin(34, Pin.IN, Pin.PULL_UP)
button_switch.irq(trigger=Pin.IRQ_RISING, handler=button_handler)

servo1 = Servo(Pin(19))
servo2 = Servo(Pin(18))

servo1.write_angle(0)
servo2.write_angle(0)
time.sleep(2)

idx = 0
angle_pairs = build(NAMES[idx])

while True:
    for t1, t2 in angle_pairs:
        if switch_flag:
            switch_flag = False
            idx = (idx + 1) % len(NAMES)
            print("state:", NAMES[idx])
            angle_pairs = build(NAMES[idx])
            break
        print("t1:", t1, "t2:", t2)
        servo1.write_angle(t1)
        servo2.write_angle(t2)
        time.sleep(STEP)