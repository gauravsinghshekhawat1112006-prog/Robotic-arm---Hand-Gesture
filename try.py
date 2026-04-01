import cv2
import mediapipe as mp
import numpy as np

# --- INIT ---
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(min_detection_confidence=0.8,
                       min_tracking_confidence=0.8)

# --- SMOOTHING ---
history_x, history_y = [], []
SMOOTH_SIZE = 5

# --- STABILITY ---
prev_status = None
stable_count = 0
STABLE_THRESHOLD = 5
last_executed = None

# --- COMMAND HANDLER ---
def handle_gesture(status):
    if status == "GRAB":
        print("Action: Pick object")
    elif status == "FREE":
        print("Action: Release object")
    elif status == "FORWARD":
        print("Action: Move Forward")
    elif status == "STOP":
        print("Action: Stop Robot")


# --- GESTURE DETECTION (IMPROVED) ---
def detect_gesture(hand_lms):
    lm = hand_lms.landmark

    # Finger tips & joints
    thumb_tip, thumb_ip = lm[4], lm[3]
    index_tip, index_pip = lm[8], lm[6]
    middle_tip, middle_pip = lm[12], lm[10]
    ring_tip, ring_pip = lm[16], lm[14]
    pinky_tip, pinky_pip = lm[20], lm[18]

    # --- Finger states ---
    index = index_tip.y < index_pip.y
    middle = middle_tip.y < middle_pip.y
    ring = ring_tip.y < ring_pip.y
    pinky = pinky_tip.y < pinky_pip.y

    # Thumb: use BOTH x and y (more reliable)
    thumb = thumb_tip.y < thumb_ip.y

    # --- Count ---
    fingers_up = [thumb, index, middle, ring, pinky]
    total = sum(fingers_up)

    # --- Pinch ---
    pinch_dist = np.linalg.norm(
        np.array([thumb_tip.x, thumb_tip.y]) -
        np.array([index_tip.x, index_tip.y])
    )

    # --- LOGIC ---

    # 👍 Thumbs up (ONLY thumb up)
    if thumb and not index and not middle and not ring and not pinky:
        return "FORWARD"

    # ✊ Fist (all closed)
    if total == 0:
        return "STOP"

    # 🤏 Pinch
    if pinch_dist < 0.05:
        return "GRAB"

    # 🖐 Open palm
    if total >= 4:
        return "FREE"

    return "UNKNOWN"


# --- CAMERA ---
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    results = hands.process(rgb)

    status = "DETECTING..."

    if results.multi_hand_landmarks:
        for hand_lms in results.multi_hand_landmarks:

            mp_draw.draw_landmarks(frame, hand_lms,
                                   mp_hands.HAND_CONNECTIONS)

            # --- SMOOTH DOT ---
            index_tip = hand_lms.landmark[8]
            h, w, _ = frame.shape
            cx, cy = int(index_tip.x * w), int(index_tip.y * h)

            history_x.append(cx)
            history_y.append(cy)

            if len(history_x) > SMOOTH_SIZE:
                history_x.pop(0)
                history_y.pop(0)

            smooth_x = int(sum(history_x) / len(history_x))
            smooth_y = int(sum(history_y) / len(history_y))

            # --- DETECT ---
            current_status = detect_gesture(hand_lms)

            # --- STABILITY FILTER ---
            if current_status == prev_status:
                stable_count += 1
            else:
                stable_count = 0

            prev_status = current_status

            if stable_count > STABLE_THRESHOLD:
                status = current_status

                # Execute only once per change
                if status != last_executed:
                    handle_gesture(status)
                    last_executed = status
            else:
                status = "DETECTING..."

            # --- DISPLAY TEXT ---
            if status == "FORWARD":
                text = "Move Forward"
                color = (0, 255, 255)

            elif status == "STOP":
                text = "Stop"
                color = (255, 0, 0)

            elif status == "FREE":
                text = "Gripper Free"
                color = (0, 255, 0)

            elif status == "GRAB":
                text = "Gripper Grab"
                color = (0, 0, 255)

            else:
                text = "Detecting..."
                color = (200, 200, 200)

            cv2.circle(frame, (smooth_x, smooth_y), 15, color, cv2.FILLED)

            cv2.putText(frame, text, (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

            cv2.putText(frame, f"Status: {status}", (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    cv2.imshow("Gesture Control", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()