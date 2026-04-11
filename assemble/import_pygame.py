import pygame
import time
import serial
import struct
import numpy as np

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

SERIAL_PORT = "COM4"
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

CONTROL_FREQ = 25
DT = 1.0 / CONTROL_FREQ

MAX_SPEED = 20  

angle1 = 90.0   
angle2 = 90.0

def send_servo(a1, a2, g, step1, step2):
    a1 = int(a1)
    a2 = int(a2)

    checksum = (a1 + a2 + g + step1 + step2) % 256
    data = struct.pack('BBBBBBB', 255, a1, a2, g, step1, step2, checksum)
    ser.write(data)

def apply_deadzone(val, dz=0.5):
    return 0 if abs(val) < dz else val

print("Velocity-based joystick control started")

gripper = 90
prev_button = 0

send_servo(angle1, angle2, gripper, 0, 0)
time.sleep(2)

next_tick = time.time()

last_press_time = 0
DEBOUNCE_TIME = 0.3  

step1 = 0
step2 = 0
while True:
    pygame.event.pump()

    axis1 = joystick.get_axis(1)   
    axis2 = -joystick.get_axis(3)

    button = joystick.get_button(0)
    step1_btn = joystick.get_axis(4)  # clockwise
    step2_btn = joystick.get_axis(5)  # anticlockwise

    now = time.time()

    if button == 1 and prev_button == 0:
        if now - last_press_time > DEBOUNCE_TIME:
            gripper = 0 if gripper == 90 else 90
            last_press_time = now

    prev_button = button

    axis1 = apply_deadzone(axis1)
    axis2 = apply_deadzone(axis2)

    vel1 = axis1 * MAX_SPEED 
    vel2 = axis2 * MAX_SPEED
  
    angle1 += vel1 * DT
    angle2 += vel2 * DT

    angle1 = np.clip(angle1, 0, 180)
    angle2 = np.clip(angle2, 0, 180)

    # stepper commands (1 or 0)
    # stepper commands (threshold based)
    step1 = 1 if step1_btn > 0.5 else 0
    step2 = 1 if step2_btn > 0.5 else 0
    print(step1_btn, step2_btn)
    
    
    send_servo(angle1, angle2, gripper, step1, step2)

    next_tick += DT
    sleep_time = next_tick - time.time()

    if sleep_time > 0:
        time.sleep(sleep_time)
    else:
        next_tick = time.time()