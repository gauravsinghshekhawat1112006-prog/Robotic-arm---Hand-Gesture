import numpy as np
import math
import time
import serial
import struct
from math import pi
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame

L1 = 12
L2 = 7

dh_params = np.array([
    [0., L1, 0., math.radians(90)],
    [0., L2, 0., math.radians(-90)]
])

robot = RobotSerial(dh_params)

SERIAL_PORT = "COM3"
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

CONTROL_FREQ = 25
CONTROL_PERIOD = 1.0 / CONTROL_FREQ

P1 = np.array([6, -10, 0.0])
P2 = np.array([16, -10, 0.0])

points = [P1, P2, P1]
# points =[np.array([13.0, 3.0, 0.0]),   # 0°
#     np.array([12.12, 5.12, 0.0]), # 45°
#     np.array([10.0, 6.0, 0.0]),   # 90°
#     np.array([7.88, 5.12, 0.0]),  # 135°
#     np.array([7.0, 3.0, 0.0]),    # 180°
#     np.array([7.88, 0.88, 0.0]),  # 225°
#     np.array([10.0, 0.0, 0.0]),   # 270°
#     np.array([12.12, 0.88, 0.0]),  # 315°
#     np.array([13.0, 3.0, 0.0])
#     ]

num_points_per_edge = 40

abc = np.array([0.5 * pi, 0., pi])

# ---- SERVO ----
def convert_to_servo(theta1, theta2):
    deg1 = math.degrees(theta1)
    deg2 = math.degrees(theta2)

    # ---- SHOULDER ----
    # s1 = 90-deg1   # no +90 shift
    s1 = abs(deg1)
    # ---- ELBOW ----
    s2 = 90 + deg2   # because deg2 is negative

    s1 = int(np.clip(s1, 0, 180))
    s2 = int(np.clip(s2, 0, 180))
    return s1, s2

def send_servo(a1, a2):
    print(a1, a2)
    checksum = (a1 + a2) % 256
    data = struct.pack('BBBB', 255, a1, a2, checksum)
    ser.write(data)

# ---- TRAJECTORY ----
trajectory = []

for i in range(len(points) - 1):
    start = points[i]
    end = points[i + 1]

    for t in np.linspace(0, 1, num_points_per_edge):
        xyz = start * (1 - t) + end * t

        # workspace safety
        r = math.sqrt(xyz[0]**2 + xyz[1]**2)
        if r > (L1 + L2) or r < abs(L1 - L2):
            continue

        # ✅ FIXED FRAME CALL
        frame = Frame.from_euler_3(abc, xyz.reshape(3, 1))

        robot.inverse(frame)

        trajectory.append(robot.axis_values.copy())



print("Starting...")
idx = 0
next_tick = time.time()

# print(trajectory)
while True:
    th1, th2 = trajectory[idx]
    idx = (idx + 1) % len(trajectory)

    s1, s2 = convert_to_servo(th1, th2)
    send_servo(s1, s2)

    next_tick += CONTROL_PERIOD
    sleep_time = next_tick - time.time()

    if sleep_time > 0:
        time.sleep(sleep_time)
    else:
        next_tick = time.time()