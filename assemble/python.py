# import numpy as np
# import math
# import time
# import serial
# import struct
# from math import pi
# from visual_kinematics.RobotSerial import RobotSerial
# from visual_kinematics.Frame import Frame

# L1 = 12
# L2 = 7

# dh_params = np.array([
#     [0., L1, 0., math.radians(90)],
#     [0., L2, 0., math.radians(-90)]
# ])

# robot = RobotSerial(dh_params)

# SERIAL_PORT = "COM3"
# BAUD_RATE = 115200
# ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
# time.sleep(2)

# CONTROL_FREQ = 25
# CONTROL_PERIOD = 1.0 / CONTROL_FREQ

# P1 = np.array([6, -10, 0.0])
# P2 = np.array([16, -10, 0.0])

# points = [P1, P2, P1]

# num_points_per_edge = 40

# abc = np.array([0.5 * pi, 0., pi])

# # ---- SERVO ----
# def convert_to_servo(theta1, theta2):
#     deg1 = math.degrees(theta1)
#     deg2 = math.degrees(theta2)

#     # ---- SHOULDER ----
#     # s1 = 90-deg1   # no +90 shift
#     s1 = abs(deg1)
#     # ---- ELBOW ----
#     s2 = 90 + deg2   # because deg2 is negative

#     s1 = int(np.clip(s1, 0, 180))
#     s2 = int(np.clip(s2, 0, 180))
#     return s1, s2

# def send_servo(a1, a2):
#     print(a1, a2)
#     checksum = (a1 + a2) % 256
#     data = struct.pack('BBBB', 255, a1, a2, checksum)
#     ser.write(data)

# # ---- TRAJECTORY ----
# trajectory = []

# for i in range(len(points) - 1):
#     start = points[i]
#     end = points[i + 1]

#     for t in np.linspace(0, 1, num_points_per_edge):
#         xyz = start * (1 - t) + end * t

#         # workspace safety
#         r = math.sqrt(xyz[0]**2 + xyz[1]**2)
#         if r > (L1 + L2) or r < abs(L1 - L2):
#             continue

#         # ✅ FIXED FRAME CALL
#         frame = Frame.from_euler_3(abc, xyz.reshape(3, 1))

#         robot.inverse(frame)

#         trajectory.append(robot.axis_values.copy())



# print("Starting...")
# idx = 0
# next_tick = time.time()

# # print(trajectory)
# while True:
#     th1, th2 = trajectory[idx]
#     idx = (idx + 1) % len(trajectory)

#     s1, s2 = convert_to_servo(th1, th2)
#     send_servo(s1, s2)

#     next_tick += CONTROL_PERIOD
#     sleep_time = next_tick - time.time()

#     if sleep_time > 0:
#         time.sleep(sleep_time)
#     else:
#         next_tick = time.time()



import numpy as np
import math
import time
import serial
import struct
from math import pi, atan2, sqrt
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame

# ---- CONFIG ----
L1 = 12
L2 = 18.5

SERIAL_PORT     = "COM4"
BAUD_RATE       = 115200
CONTROL_FREQ    = 20
CONTROL_PERIOD  = 1.0 / CONTROL_FREQ
STEPS_PER_REV   = 3200
DEG_PER_STEP    = 360.0 / STEPS_PER_REV   # 0.1125 deg/step

GRIPPER_OPEN    = 90
GRIPPER_CLOSED  = 50

# ---- INIT ----
dh_params = np.array([
    [0., L1, 0., math.radians(90)],
    [0., L2, 0., math.radians(-90)]
])
robot = RobotSerial(dh_params)

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

abc = np.array([0.5 * pi, 0., pi])

# ---- WAYPOINTS ----
# P1 = np.array([6.0,  -10.0, 0.0])
# P2 = np.array([16.0, -10.0, 5.0])
P1 = np.array([8.0,  -6.0,  -2.0])   # r=10.0,  dist=10.2  ✓
P2 = np.array([22.0, -14.0, 8.0])   # r=26.1,  dist=27.3  ✓
NUM_STEPS_PER_EDGE = 100

# ---- HELPERS ----
def convert_to_servo(theta1, theta2):
    s1 = int(np.clip(abs(math.degrees(theta1)),        0, 180))
    s2 = int(np.clip(90 + math.degrees(theta2),        0, 180))
    return s1, s2

def angle_to_steps(delta_deg):
    steps     = int(round(abs(delta_deg) / DEG_PER_STEP))
    direction = 1 if delta_deg >= 0 else 2
    return steps, direction

# DEG_PER_STEP = 360.0 / STEPS_PER_REV  # 0.1125 deg/step

def build_segment(start, end, n, gripper_angle):
    segment  = []
    az_start = math.degrees(atan2(start[1], start[0]))
    az_end   = math.degrees(atan2(end[1],   end[0]))

    total_delta = az_end - az_start        # signed degrees
    step_deg    = total_delta / 10          # fixed degrees per tick  ← send this
    direction   = 1 if total_delta >= 0 else 2

    # encode step_deg as an integer 0–255 (scaled by 10 to keep 1 decimal)
    step_val = int(round(abs(step_deg) * 100))
    step_val = min(step_val, 255)

    ts = np.linspace(0, 1, n)

    for j, t in enumerate(ts):
        xyz     = start * (1 - t) + end * t
        x, y, z = xyz

        r    = sqrt(x**2 + y**2)
        dist = sqrt(r**2 + z**2)
        if dist > (L1 + L2) or dist < abs(L1 - L2):
            continue

        frame = Frame.from_euler_3(abc, np.array([r, z, 0.0]).reshape(3, 1))
        robot.inverse(frame)
        theta1, theta2 = robot.axis_values

        shoulder, elbow = convert_to_servo(theta1, theta2)
        segment.append((shoulder, elbow, step_val, direction, gripper_angle))

    return segment


def send_packet(shoulder, elbow, step_val, direction, gripper):
    """
    Packet (7 bytes):
    [0xFF, shoulder, elbow, step_val, direction, gripper, checksum]

    step_val = abs(step_deg) * 10   e.g. 1.125 deg → 11
    direction = 1 (CW) or 2 (CCW)
    """
    checksum = (shoulder + elbow + step_val + direction + gripper) % 256
    data = struct.pack('BBBBBBB',
                       0xFF,
                       shoulder, elbow,
                       step_val,
                       direction,
                       gripper,
                       checksum)
    ser.write(data)
    print(f"Shoulder={shoulder:3d}  Elbow={elbow:3d}  "
          f"StepAngle={step_val/10:.2f}°  Dir={'CW ' if direction==1 else 'CCW'}  "
          f"Gripper={'OPEN  ' if gripper==GRIPPER_OPEN else 'CLOSED'}")
# ---- BUILD FULL MISSION ----
#
# Mission sequence:
#   1. Open gripper,  move P1 → P2   (approach with open gripper)
#   2. Close gripper, stay at P2     (grip — single hold packet)
#   3. Close gripper, move P2 → P1   (carry object back)
#   4. Open gripper,  stay at P1     (release — single hold packet)
#   repeat
#
def build_mission():
    mission = []

    # Segment 1: P1 → P2, gripper open
    mission += build_segment(P1, P2, NUM_STEPS_PER_EDGE, GRIPPER_OPEN)

    # Grip at P2: send one packet with closed gripper, arm stays at P2
    last = mission[-1]
    mission.append((last[0], last[1], 0, 1, GRIPPER_CLOSED))

    # Segment 2: P2 → P1, gripper closed (carrying)
    mission += build_segment(P2, P1, NUM_STEPS_PER_EDGE, GRIPPER_CLOSED)

    # Release at P1: send one packet with open gripper, arm stays at P1
    last = mission[-1]
    mission.append((last[0], last[1], 0, 1, GRIPPER_OPEN))

    return mission

mission = build_mission()
print(f"Mission built: {len(mission)} packets")

# ---- CONTROL LOOP ----
print("Starting mission loop...")

idx       = 0
next_tick = time.time()

while True:
    shoulder, elbow, steps, direction, gripper = mission[idx]
    idx = (idx + 1) % len(mission)

    send_packet(shoulder, elbow, steps, direction, gripper)

    next_tick += CONTROL_PERIOD
    sleep_time = next_tick - time.time()
    if sleep_time > 0:
        time.sleep(sleep_time)
    else:
        next_tick = time.time()