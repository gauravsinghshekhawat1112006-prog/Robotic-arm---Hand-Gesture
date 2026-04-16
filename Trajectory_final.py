
import numpy as np
import math
import time
import serial
import struct
from math import pi
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame

# ================= ROBOT =================
L1 = 12
L2 = 18.5

dh_params = np.array([
    [0., L1, 0., math.radians(90)],
    [0., L2, 0., math.radians(-90)]
])

robot = RobotSerial(dh_params)

# ================= SERIAL =================
SERIAL_PORT = "COM12"
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# ================= CONTROL =================
CONTROL_FREQ = 20
CONTROL_PERIOD = 1.0 / CONTROL_FREQ

# ================= POINTS =================
P1 = np.array([6, -10, 0.0])
P2 = np.array([16, -10, 0.0])
NEUTRAL = np.array([12, 0, 0.0])

abc = np.array([0.5 * pi, 0., pi])

GRIP_OPEN = 90
GRIP_CLOSE = 45

# ================= SERVO =================
def convert_to_servo(theta1, theta2):
    deg1 = math.degrees(theta1)
    deg2 = math.degrees(theta2)

    s1 = abs(deg1)
    s2 = 90 + deg2

    s1 = int(np.clip(s1, 0, 180))
    s2 = int(np.clip(s2, 0, 180))
    return s1, s2

# ================= BASE ANGLE (PURE) =================
def get_base_angle(x, z):
    return math.degrees(math.atan2(z, x))   # PURE angle (-180 to 180)

# ================= SERIAL SEND =================
def send_all(base_angle, a1, a2, g):
    
    base = int(base_angle)

    direction = 1 if base >= 0 else 0
    angle_mag = abs(base)

    checksum = (a1 + a2 + g + angle_mag + direction) % 256

    data = struct.pack('BBBBBBB',
                       255,
                       a1,
                       a2,
                       g,
                       angle_mag,
                       direction,
                       checksum)

    ser.write(data)

# ================= IK =================
def ik(point):
    x, y, z = point

    # Base rotation using XZ
    base_angle = get_base_angle(x, z)

    # Project into planar arm space
    r = math.sqrt(x**2 + z**2)

    if r > (L1 + L2) or r < abs(L1 - L2):
        return None, None

    # Arm operates in XY after projection
    planar_point = np.array([r, 0, y])

    frame = Frame.from_euler_3(abc, planar_point.reshape(3, 1))
    robot.inverse(frame)

    return robot.axis_values.copy(), base_angle

# ================= LINEAR MOTION =================
def move_linear(start, end, grip, steps=40):

    for t in np.linspace(0, 1, steps):

        xyz = start * (1 - t) + end * t

        th, base_angle = ik(xyz)
        if th is None:
            continue

        s1, s2 = convert_to_servo(th[0], th[1])

        send_all(base_angle, s1, s2, grip)

        time.sleep(CONTROL_PERIOD)

print("Starting Pick & Place (Angle-Based Stepper Control)")

while True:

    # Neutral
    move_linear(NEUTRAL, NEUTRAL, GRIP_OPEN)

    # Move to P1
    move_linear(NEUTRAL, P1, GRIP_OPEN)

    # Pick
    time.sleep(0.5)
    move_linear(P1, P1, GRIP_CLOSE, steps=10)

    # Move to P2
    move_linear(P1, P2, GRIP_CLOSE)

    # Place
    time.sleep(0.5)
    move_linear(P2, P2, GRIP_OPEN, steps=10)

    # Back to Neutral
    move_linear(P2, NEUTRAL, GRIP_OPEN)
