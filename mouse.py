import cv2
import RPi.GPIO as GPIO
import time
import numpy as np

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

try:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # Flip the frame if needed
        frame = cv2.flip(frame, -1)
        
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define color range for object to track (e.g., red color)
        lower_color = np.array([0, 70, 50])
        upper_color = np.array([10, 255, 255])

        # Create a mask for the color
        mask = cv2.inRange(hsv, lower_color, upper_color)
        
        # Find contours in the mask
        if int(cv2.__version__.split('.')[0]) >= 4:
            # For OpenCV 4.x
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            # For OpenCV 3.x
            _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Proceed if any contours are found
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            # Get the bounding rectangle of the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)
            # Calculate the center of the object
            object_center_x = x + w // 2
            object_center_y = y + h // 2
            
            # Map coordinates to servo angles (0-180)
            height, width, _ = frame.shape
            servo_x = np.interp(object_center_x, [0, width], [0, 180])
            servo_y = np.interp(object_center_y, [0, height], [0, 180])
            
            # Update servo positions
            set_servo_angle(pwm_x, servo_x)
            set_servo_angle(pwm_y, servo_y)
            
            # Draw a circle at the object's center
            cv2.circle(frame, (object_center_x, object_center_y), 5, (0, 255, 0), -1)
            # Draw the bounding rectangle
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Display the frame
        cv2.imshow('Object Tracking', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    pwm_x.stop()
    pwm_y.stop()
    GPIO.cleanup()
