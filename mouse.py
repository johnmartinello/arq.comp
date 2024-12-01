import cv2
import numpy as np
import GPio.RPi as GPIO

GPIO.setmode(GPIO.BCM)
servo_pin_x = 20
servo_pin_y = 21
GPIO.setup(servo_pin_x, GPIO.OUT)
GPIO.setup(servo_pin_y, GPIO.OUT)

# Initialize PWM for servos
pwm_x = GPIO.PWM(servo_pin_x, 50)  # 50Hz frequency
pwm_y = GPIO.PWM(servo_pin_y, 50)

cap = cv2.VideoCapture(0)

# Set camera resolution
cap.set(3, 480)
cap.set(4, 320)

_, frame = cap.read()
rows, cols, _ = frame.shape

x_medium = int(cols / 2)
center = int(cols / 2)
position = 90 # degrees

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # BLUE COLOR
    low_blue = np.array([94, 80, 2])
    high_blue = np.array([126, 255, 255])
    
    blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
    _, contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)

    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)
        
        x_medium = int((x + x + w) / 2)
        break
    
    cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)

        # Move servo motor
    if x_medium < center -30:
        position += 1
    elif x_medium > center + 30:
        position -= 1
    pwm.setServoPosition(0, position)

    cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
    
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    
    if key == 27:
        break
    
cap.release()
cv2.destroyAllWindows()
