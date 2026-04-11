import time
import serial
import struct
import numpy as np

SERIAL_PORT = "COM3"
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# --- Constants ---
CONTROL_FREQ = 25
DT = 1.0 / CONTROL_FREQ
MAX_SPEED = 20  

# --- State Variables ---
angle1 = 90.0   
angle2 = 90.0
gripper = 90
last_gesture = None
last_press_time = 0
DEBOUNCE_TIME = 0.5 

def send_servo(a1, a2, g):
    a1, a2 = int(a1), int(a2)
    checksum = (a1 + a2 + g) % 256
    data = struct.pack('BBBBB', 255, a1, a2, g, checksum)
    ser.write(data)
    print(f"Sent -> A1: {a1}, A2: {a2}, G: {g}")

print("Gesture-based control started")
next_tick = time.time()

while True:
    axis1 = 0.0
    axis2 = 0.0
    
    if gesture == "up":
        axis1 = 1.0
    elif gesture == "down":
        axis1 = -1.0
        
    if gesture == "right":
        axis2 = 1.0
    elif gesture == "left":
        axis2 = -1.0

    now = time.time()
    if gesture == "fist" and last_gesture != "fist":
        if now - last_press_time > DEBOUNCE_TIME:
            gripper = 0 if gripper == 90 else 90
            last_press_time = now
    
    last_gesture = gesture

    # 3. VELOCITY CALCULATION (Same as your RC logic)
    vel1 = axis1 * MAX_SPEED 
    vel2 = axis2 * MAX_SPEED
  
    angle1 += vel1 * DT
    angle2 += vel2 * DT

    # Keep angles within servo bounds
    angle1 = np.clip(angle1, 0, 180)
    angle2 = np.clip(angle2, 0, 180)

    # 4. SEND DATA
    send_servo(angle1, angle2, gripper)

    # 5. TIMING CONTROL
    next_tick += DT
    sleep_time = next_tick - time.time()
    if sleep_time > 0:
        time.sleep(sleep_time)
    else:
        next_tick = time.time()