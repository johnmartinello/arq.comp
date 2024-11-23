import cv2
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

def count_fingers(hand_landmarks, handedness):
    landmarks = hand_landmarks.landmark

    finger_tips_ids = [mp_hands.HandLandmark.INDEX_FINGER_TIP,
                       mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                       mp_hands.HandLandmark.RING_FINGER_TIP,
                       mp_hands.HandLandmark.PINKY_TIP]

    finger_dips_ids = [mp_hands.HandLandmark.INDEX_FINGER_DIP,
                       mp_hands.HandLandmark.MIDDLE_FINGER_DIP,
                       mp_hands.HandLandmark.RING_FINGER_DIP,
                       mp_hands.HandLandmark.PINKY_DIP]

    raised_fingers = 0

    
    for tip_id, dip_id in zip(finger_tips_ids, finger_dips_ids):
        if landmarks[tip_id].y < landmarks[dip_id].y:  
            raised_fingers += 1

    thumb_tip = landmarks[mp_hands.HandLandmark.THUMB_TIP]
    thumb_ip = landmarks[mp_hands.HandLandmark.THUMB_IP]
    wrist = landmarks[mp_hands.HandLandmark.WRIST]
    
    if handedness == "Left":
        if thumb_tip.x > thumb_ip.x and wrist.x < thumb_tip.x:  # Thumb to the right of wrist for left hand facing left
            raised_fingers += 1
        elif thumb_tip.x < thumb_ip.x and wrist.x > thumb_tip.x:  # Thumb to the left of wrist for left hand facing right
            raised_fingers += 1
    else:
        if thumb_tip.x < thumb_ip.x and wrist.x > thumb_tip.x:  # Thumb to the left of wrist for right hand facing right
            raised_fingers += 1
        elif thumb_tip.x > thumb_ip.x and wrist.x < thumb_tip.x:  # Thumb to the right of wrist for right hand facing left
            raised_fingers += 1

    return raised_fingers

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 

with mp_hands.Hands(
    model_complexity=0,  
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    max_num_hands=2) as hands:

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)

        right_hand_fingers = 0
        left_hand_fingers = 0

       
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
            for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                
                handedness = results.multi_handedness[i].classification[0].label
                if handedness == "Right":
                    handedness = "Left"
                else:
                    handedness = "Right"

                raised_fingers = count_fingers(hand_landmarks, handedness)

                if handedness == "Right":
                    right_hand_fingers = raised_fingers
                else:
                    left_hand_fingers = raised_fingers

                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())

        flipped_image = cv2.flip(image, 1)

        # Step 7: Display the frames and print the sum for both hands on the flipped image
        cv2.putText(flipped_image, f'Right Hand: {right_hand_fingers}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(flipped_image, f'Left Hand: {left_hand_fingers}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(flipped_image, f'SUM: {left_hand_fingers + right_hand_fingers}', (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow('Hand Finger Count', flipped_image)

        if cv2.waitKey(5) & 0xFF == 27:  # Press 'Esc' to exit
            break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()