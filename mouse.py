import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# GPIO setup for servos
GPIO.setmode(GPIO.BCM)
servo_pin_x = 20
servo_pin_y = 21
GPIO.setup(servo_pin_x, GPIO.OUT)
GPIO.setup(servo_pin_y, GPIO.OUT)

# Initialize PWM for servos
pwm_x = GPIO.PWM(servo_pin_x, 50)  # 50Hz frequency
pwm_y = GPIO.PWM(servo_pin_y, 50)
pwm_x.start(7.5)  # Center position (90 degrees)
pwm_y.start(7.5)

def set_servo_angle(pwm, angle):
    duty = 2.5 + (angle / 18.0)  # Convert angle to duty cycle
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.02)

# Initialize camera
cap = cv2.VideoCapture(0)

# Set camera resolution
cap.set(3, 480)
cap.set(4, 320)

_, frame = cap.read()
rows, cols, _ = frame.shape

x_medium = int(cols / 2)
y_medium = int(rows / 2)
center_x = int(cols / 2)
center_y = int(rows / 2)
position_x = 90  # degrees
position_y = 90  # degrees

try:
    while True:
        _, frame = cap.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # BLUE COLOR
        low_blue = np.array([94, 80, 2])
        high_blue = np.array([126, 255, 255])
        
        blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)
            x_medium = int((x + x + w) / 2)
            y_medium = int((y + y + h) / 2)
            break
        
        cv2.line(frame, (x_medium, 0), (x_medium, rows), (0, 255, 0), 2)
        cv2.line(frame, (0, y_medium), (cols, y_medium), (0, 255, 0), 2)

        # Move servo motor
        if x_medium < center_x - 30:
            position_x += 1
        elif x_medium > center_x + 30:
            position_x -= 1

        if y_medium < center_y - 30:
            position_y += 1
        elif y_medium > center_y + 30:
            position_y -= 1

        position_x = max(0, min(180, position_x))
        position_y = max(0, min(180, position_y))

        set_servo_angle(pwm_x, position_x)
        set_servo_angle(pwm_y, position_y)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)
        
        if key == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    pwm_x.stop()
    pwm_y.stop()
    GPIO.cleanup()
