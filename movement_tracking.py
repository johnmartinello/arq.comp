import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# GPIO setup for servos
GPIO.setwarnings(False)
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
    # Ensure angle stays within servo limits (0 to 180 degrees)
    angle = max(0, min(180, angle))
    duty = 2.5 + (angle / 18.0)  # Convert angle to duty cycle (0 to 12.5)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.02)

# Initialize video capture
cap = cv2.VideoCapture(0)

# cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,1080)

# Check if the camera is opened correctly
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Read the first frame and process it
ret, frame1 = cap.read()
if not ret:
    print("Failed to grab frame")
    exit()

# Resize frame to a smaller size for faster processing
frame1 = cv2.resize(frame1, (320, 240))
gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
gray1 = cv2.GaussianBlur(gray1, (21, 21), 0)

# Initial servo positions
position_x = 90
position_y = 90

# Smoothing factor
smooth_factor = 0.9  # Increase smoothness for more gradual movements

cv2.namedWindow("Motion Tracking", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Motion Tracking", 1020, 800)



while True:
    # Read the next frame
    ret, frame2 = cap.read()
    if not ret:
        break

    # Resize frame and convert to grayscale
    frame2 = cv2.resize(frame2, (320, 240))
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.GaussianBlur(gray2, (21, 21), 0)

    # Compute the absolute difference between the two frames
    diff = cv2.absdiff(gray1, gray2)

    # Threshold the difference image
    _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

    # Dilate the thresholded image to fill in holes
    thresh = cv2.dilate(thresh, None, iterations=2)

    # Find contours on the thresholded image
    try:
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    except ValueError:
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Filter out small contours
        filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]
        if filtered_contours:
            # Get the largest contour
            largest_contour = max(filtered_contours, key=cv2.contourArea)
            # Get the bounding rectangle for the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)
            # Calculate the center of the object
            object_center_x = x + w // 2
            object_center_y = y + h // 2

            # Map coordinates to servo angles (0-180)
            height, width = frame2.shape[:2]
            target_x = np.interp(object_center_x, [0, width], [130, 20])
            target_y = np.interp(object_center_y, [0, height], [90, 0])

            # Smooth the movement
            position_x += (target_x - position_x) * smooth_factor
            position_y += (target_y - position_y) * smooth_factor

            # Update servo positions
            set_servo_angle(pwm_x, position_x)
            set_servo_angle(pwm_y, position_y)

            # Draw a circle at the object's center
            cv2.circle(frame2, (object_center_x, object_center_y), 5, (0, 255, 0), -1)
            # Draw the bounding rectangle
            cv2.rectangle(frame2, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Show the result
    cv2.imshow("Motion Tracking", frame2)

    # Update the previous frame
    gray1 = gray2.copy()

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
pwm_x.stop()
pwm_y.stop()
GPIO.cleanup()


