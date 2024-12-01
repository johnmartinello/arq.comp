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
# center at 90 degrees
pwm_x.start(7.5)
pwm_y.start(7.5)

def set_servo_angle(pwm, angle):
    duty = 2.5 + (angle / 18.0)  # Convert angle to duty cycle
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.02)

# Initialize video capture
cap = cv2.VideoCapture(0)

# Read the first frame
ret, frame1 = cap.read()
if not ret:
    print("Failed to grab frame")
    exit()

# Rotate the frame by 90 degrees clockwise
frame1 = cv2.transpose(frame1)
frame1 = cv2.flip(frame1, 1)

# Convert frame to grayscale and blur it
gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
gray1 = cv2.GaussianBlur(gray1, (21, 21), 0)

# Initial servo positions
position_x = 180
position_y = 180

# Smoothing factor
smooth_factor = 0.3

while True:
    # Read the next frame
    ret, frame2 = cap.read()
    if not ret:
        break

    # Rotate the frame by 90 degrees clockwise
    frame2 = cv2.transpose(frame2)
    frame2 = cv2.flip(frame2, 1)

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

    # Define color range for red color
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for the red color
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Invert the red mask to filter out red color
    mask = cv2.bitwise_not(red_mask)

    # Apply the mask to the frame
    frame2 = cv2.bitwise_and(frame2, frame2, mask=mask)

    # Convert frame to grayscale and blur it
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.GaussianBlur(gray2, (21, 21), 0)

    # Compute the absolute difference between the two frames
    diff = cv2.absdiff(gray1, gray2)

    # Threshold the difference image
    _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

    # Dilate the thresholded image to fill in holes
    thresh = cv2.dilate(thresh, None, iterations=2)

    # Find contours on the thresholded image
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Optional: Filter out small contours
        filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]
        if filtered_contours:
            # Combine all contours into one
            all_contours = np.vstack(filtered_contours)
            # Get the bounding rectangle for the combined contours
            x, y, w, h = cv2.boundingRect(all_contours)
            # Calculate the center of the object
            object_center_x = x + w // 2
            object_center_y = y + h // 2

            # Map coordinates to servo angles (0-180)
            height, width, _ = frame2.shape
            target_x = np.interp(object_center_x, [0, width], [0, 180])
            target_y = np.interp(object_center_y, [0, height], [0, 180])

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
