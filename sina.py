import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Set up GPIO
pan_servo_pin = 18
tilt_servo_pin = 19
GPIO.setmode(GPIO.BCM)
GPIO.setup(pan_servo_pin, GPIO.OUT)
GPIO.setup(tilt_servo_pin, GPIO.OUT)
pan_pwm = GPIO.PWM(pan_servo_pin, 50)  # 50Hz frequency
tilt_pwm = GPIO.PWM(tilt_servo_pin, 50)  # 50Hz frequency
pan_pwm.start(7.5)  # Initial position
tilt_pwm.start(7.5)  # Initial position

# Function to set servo angle
def set_servo_angle(pwm, angle):
    duty = 2.5 + (angle / 18.0) * 10.0
    pwm.ChangeDutyCycle(duty)

# Initialize camera
cap = cv2.VideoCapture(0)

# Initialize variables for motion detection
ret, frame1 = cap.read()
ret, frame2 = cap.read()

# Set initial servo positions
pan_angle = 90
tilt_angle = 90
set_servo_angle(pan_pwm, pan_angle)
set_servo_angle(tilt_pwm, tilt_angle)

try:
    while True:
        # Calculate the absolute difference between the current frame and the previous frame
        diff = cv2.absdiff(frame1, frame2)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour
        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            # Get the bounding box of the largest contour
            (x, y, w, h) = cv2.boundingRect(max_contour)
            cv2.rectangle(frame1, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Calculate the center of the bounding box
            center_x = x + w // 2
            center_y = y + h // 2

            # Calculate the error between the center of the bounding box and the center of the frame
            frame_center_x = frame1.shape[1] // 2
            frame_center_y = frame1.shape[0] // 2
            error_x = center_x - frame_center_x
            error_y = center_y - frame_center_y

            # Adjust servo angles based on the error
            if abs(error_x) > 20:  # Threshold to avoid jitter
                if error_x > 0:
                    pan_angle -= 1  # Move left
                else:
                    pan_angle += 1  # Move right

            if abs(error_y) > 20:  # Threshold to avoid jitter
                if error_y > 0:
                    tilt_angle -= 1  # Move down
                else:
                    tilt_angle += 1  # Move up

            # Clamp servo angles to valid range
            pan_angle = max(0, min(180, pan_angle))
            tilt_angle = max(0, min(180, tilt_angle))
            set_servo_angle(pan_pwm, pan_angle)
            set_servo_angle(tilt_pwm, tilt_angle)

        # Display the resulting frame
        cv2.imshow('Frame', frame1)

        # Update the frames
        frame1 = frame2
        ret, frame2 = cap.read()

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    pan_pwm.stop()
    tilt_pwm.stop()
    GPIO.cleanup()

import cv2
import numpy as np
import time

# Initialize camera
cap = cv2.VideoCapture(0)

# Initialize variables for motion detection
ret, frame1 = cap.read()
ret, frame2 = cap.read()

try:
    while True:
        # Calculate the absolute difference between the current frame and the previous frame
        diff = cv2.absdiff(frame1, frame2)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour
        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            # Get the bounding box of the largest contour
            (x, y, w, h) = cv2.boundingRect(max_contour)
            cv2.rectangle(frame1, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Calculate the center of the bounding box
            center_x = x + w // 2
            center_y = y + h // 2

            # Calculate the error between the center of the bounding box and the center of the frame
            frame_center_x = frame1.shape[1] // 2
            frame_center_y = frame1.shape[0] // 2
            error_x = center_x - frame_center_x
            error_y = center_y - frame_center_y

            # Print the calculated x, y coordinates to correct the camera position
            print(f"Error X: {error_x}, Error Y: {error_y}")

        # Display the resulting frame
        cv2.imshow('Frame', frame1)

        # Update the frames
        frame1 = frame2
        ret, frame2 = cap.read()

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Clean up
    cap.release()
    cv2.destroyAllWindows()