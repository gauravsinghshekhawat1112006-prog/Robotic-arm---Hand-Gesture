import cv2
import mediapipe as mp
import numpy as np

# --- INIT ---
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(min_detection_confidence=0.8,
                       min_tracking_confidence=0.8,
                       max_num_hands=2)

# --- STABILITY (SEPARATE FOR BOTH HANDS) ---
prev_status = {"Left": None, "Right": None}
stable_count = {"Left": 0, "Right": 0}
last_executed = {"Left": None, "Right": None}

STABLE_THRESHOLD = 5

# --- COMMAND HANDLER ---
def handle_gesture(left_status, right_status):
    print(f"LEFT: {left_status} | RIGHT: {right_status}")

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

    # --- RIGHT HAND → θ1 ---
    if hand_label == "Right":
        if index and not middle and not ring and not pinky:
            return "theta1 Increase"

        if index and middle and not ring and not pinky:
            return "theta1 Decrease"

    # --- LEFT HAND → θ2 + GRIPPER ---
    if hand_label == "Left":
        if index and not middle and not ring and not pinky:
            return "theta2 Increase"

        if index and middle and not ring and not pinky:
            return "theta2 Decrease"

        if not index and not middle and not ring and not pinky:
            return "Gripper Close"

        if total >= 4:
            return "Gripper Open"

    return "Detecting..."

# --- CAMERA ---
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    results = hands.process(rgb)

    # Default outputs (if hand not detected)
    output = {"Left": "0", "Right": "0"}

    if results.multi_hand_landmarks:
        for i, hand_lms in enumerate(results.multi_hand_landmarks):

            mp_draw.draw_landmarks(frame, hand_lms,
                                   mp_hands.HAND_CONNECTIONS)

            hand_label = results.multi_handedness[i].classification[0].label

            current_status = detect_gesture(hand_lms, hand_label)

            # --- STABILITY PER HAND ---
            if current_status == prev_status[hand_label]:
                stable_count[hand_label] += 1
            else:
                stable_count[hand_label] = 0

            prev_status[hand_label] = current_status

            if stable_count[hand_label] > STABLE_THRESHOLD:
                if current_status != last_executed[hand_label]:
                    last_executed[hand_label] = current_status

                output[hand_label] = current_status
            else:
                output[hand_label] = "Detecting..."

            # Display which hand
            cv2.putText(frame, f"{hand_label} Hand", (10, 30 + i*30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    # --- HANDLE BOTH HANDS TOGETHER ---
    handle_gesture(output["Left"], output["Right"])

    # --- DISPLAY OUTPUT ---
    cv2.putText(frame, f"LEFT: {output['Left']}", (50, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    cv2.putText(frame, f"RIGHT: {output['Right']}", (50, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    cv2.imshow("Dual Hand Robot Control", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()