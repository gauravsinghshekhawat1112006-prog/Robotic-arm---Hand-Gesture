import serial
import struct
import time

SERIAL_PORT = "COM3"
BAUD_RATE = 115200
SYNC_BYTE = 0xFF

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

def send_angles(angle1, angle2):
    angle1 = max(0, min(180, int(angle1)))
    angle2 = max(0, min(180, int(angle2)))
    ser.write(struct.pack('BBB', SYNC_BYTE, angle1, angle2))
    print(f"Sent -> angle1: {angle1}  angle2: {angle2}")

# Edit these as needed
commands = [
    (90,  90),
    (45,  135),
    (0,   180),
    (135, 45),
    (180, 0),
    (90,  90),
]

for angle1, angle2 in commands:
    send_angles(angle1, angle2)
    time.sleep(1)  # wait 1 second between each command

ser.close()