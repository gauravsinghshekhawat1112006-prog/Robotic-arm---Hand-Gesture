import cv2
import mediapipe as mp
import numpy as np
import asyncio
import websockets
import threading
import json

# ================== WEBSOCKET CONFIG ==================
WEBSOCKET_URL = "ws://10.61.75.150:81"
SEND_INTERVAL = 0.1     

output = [0,0,0,0]
lock = threading.Lock()

# ================== WEBSOCKET SENDER ==================
async def websocket_sender():
    global output

    while True:
        try:
            async with websockets.connect(WEBSOCKET_URL) as ws: 
                print("Connected to ESP8266 WebSocket")

                while True:
                    with lock:
                        outputs = output.copy()
                        x,y,z,b = outputs
                        checksum = (x+y+z+b)/256
                        outputs = [x, y, z, b, checksum]
                        gesture = json.dumps(outputs)

                    await ws.send(gesture)
                    print(f"Sent → {gesture}")

                    await asyncio.sleep(SEND_INTERVAL)  

        except (websockets.ConnectionClosed, Exception) as e:
            print(f"WebSocket error: {e}. Retrying in 3s...")
            await asyncio.sleep(3)

def start_websocket():
    asyncio.run(websocket_sender()) 

# Start WebSocket in a separate thread
ws_thread = threading.Thread(target=start_websocket, daemon=True)
ws_thread.start()

# --- INIT ---
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    min_detection_confidence=0.8,
    min_tracking_confidence=0.8,
    max_num_hands=2
)

# --- STATE ---
theta1 = 0.0
theta2 = 0.0
gripper = 0
base = 0

STEP = 0.8
MAX_ANGLE = 180
MIN_ANGLE = 0

BASE_MAX = 360
BASE_MIN = 0

# --- GESTURE DETECTION ---
def detect_gesture(hand_lms, hand_label):
    lm = hand_lms.landmark

    thumb_tip, thumb_ip = lm[4], lm[3]
    index_tip, index_pip = lm[8], lm[6]
    middle_tip, middle_pip = lm[12], lm[10]
    ring_tip, ring_pip = lm[16], lm[14]
    pinky_tip, pinky_pip = lm[20], lm[18]

    thumb = thumb_tip.y < thumb_ip.y
    index = index_tip.y < index_pip.y
    middle = middle_tip.y < middle_pip.y
    ring = ring_tip.y < ring_pip.y
    pinky = pinky_tip.y < pinky_pip.y

    fingers = [thumb, index, middle, ring, pinky]
    total = sum(fingers)

    # RIGHT HAND → theta1 + base
    if hand_label == "Right":
        if index and not middle:
            return "theta1_inc"

        if index and middle:
            return "theta1_dec"

        # Base control
        if total >= 4:
            return "base_cw"

        if total == 0 or (total == 1 and thumb):
            return "base_ccw"

    # LEFT HAND → theta2 + gripper
    if hand_label == "Left":
        if index and not middle and not ring and not pinky:
            return "theta2_inc"

        if index and middle and not ring and not pinky:
            return "theta2_dec"

        if total == 0 or (total == 1 and thumb):
            return "grip_close"

        if total >= 4:
            return "grip_open"

    return None

# --- CAMERA ---
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    results = hands.process(rgb)

    # Default commands (no hand)
    command = {"theta1": 0, "theta2": 0, "gripper": 0, "base": 0}

    if results.multi_hand_landmarks:
        for i, hand_lms in enumerate(results.multi_hand_landmarks):

            mp_draw.draw_landmarks(frame, hand_lms,
                                   mp_hands.HAND_CONNECTIONS)

            hand_label = results.multi_handedness[i].classification[0].label

            gesture = detect_gesture(hand_lms, hand_label)

            if gesture == "theta1_inc":
                command["theta1"] = 1

            elif gesture == "theta1_dec":
                command["theta1"] = -1

            elif gesture == "theta2_inc":
                command["theta2"] = 1

            elif gesture == "theta2_dec":
                command["theta2"] = -1

            elif gesture == "grip_open":
                command["gripper"] = 1

            elif gesture == "grip_close":
                command["gripper"] = -1

            elif gesture == "base_cw":
                command["base"] = 1

            elif gesture == "base_ccw":
                command["base"] = -1

    # --- APPLY CHANGES ---
    theta1 += command["theta1"] * STEP
    theta2 += command["theta2"] * STEP
    gripper += command["gripper"] * STEP
    base += command["base"] * STEP

    # Clamp angles
    theta1 = max(MIN_ANGLE, min(MAX_ANGLE, theta1))
    theta2 = max(MIN_ANGLE, min(MAX_ANGLE, theta2))
    gripper = max(0, min(90, gripper))
    base = max(BASE_MIN, min(BASE_MAX, base))

    # --- OUTPUT LIST ---
    with lock:
        output = [
            round(theta1, 2),
            round(theta2, 2),
            round(gripper, 2),
            round(base, 2)
        ]

    print(output)

    # --- DISPLAY ---
    cv2.putText(frame, f"Theta1: {int(theta1)}", (50, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    cv2.putText(frame, f"Theta2: {int(theta2)}", (50, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    cv2.putText(frame, f"Gripper: {gripper}", (50, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    cv2.putText(frame, f"Base: {int(base)}", (50, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    cv2.imshow("Robot Control", frame)

    # Loop delay ≈ 0.1 sec
    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

