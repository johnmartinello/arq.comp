import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

class ArUcoMarkerTracker:
    def __init__(self, x_servo_pin=20, y_servo_pin=21, forward_x=7.5, forward_y=7.5):
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Servo pins setup
        self.x_servo_pin = x_servo_pin
        self.y_servo_pin = y_servo_pin
        GPIO.setup(self.x_servo_pin, GPIO.OUT)
        GPIO.setup(self.y_servo_pin, GPIO.OUT)
        
        # PWM setup for servos
        self.x_pwm = GPIO.PWM(self.x_servo_pin, 50)  # 50 Hz
        self.y_pwm = GPIO.PWM(self.y_servo_pin, 50)  # 50 Hz
        
        # Start PWM
        self.x_pwm.start(0)
        self.y_pwm.start(0)
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        
        # Servo calibration 
        self.x_center = forward_x  # Forward-facing X position
        self.y_center = forward_y  # Forward-facing Y position
        
        # ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Set initial position to forward
        self.set_servo_angle(self.x_pwm, self.x_center)
        self.set_servo_angle(self.y_pwm, self.y_center)
        
    def set_servo_angle(self, pwm, angle):
        """Convert angle to duty cycle and set servo position"""
        duty = angle / 18 + 2.5
        pwm.ChangeDutyCycle(duty)
        
    def track_aruco_marker(self):
        try:
            while True:
                # Capture frame
                ret, frame = self.cap.read()
                if not ret:
                    break
                
                # Convert to grayscale for ArUco detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Detect ArUco markers
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, 
                    self.aruco_dict, 
                    parameters=self.aruco_params
                )
                
                # If markers are detected
                if ids is not None and len(ids) > 0:
                    # Draw detected markers
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    
                    # Find the first detected marker
                    marker_corners = corners[0][0]
                    
                    # Calculate marker center
                    marker_center_x = int(np.mean(marker_corners[:, 0]))
                    marker_center_y = int(np.mean(marker_corners[:, 1]))
                    
                    # Calculate frame center
                    frame_center_x = frame.shape[1] // 2
                    frame_center_y = frame.shape[0] // 2
                    
                    # Calculate error (deviation from center)
                    error_x = marker_center_x - frame_center_x
                    error_y = marker_center_y - frame_center_y
                    
                    # Simple proportional control
                    # Adjust these gain values to control servo sensitivity
                    kp_x = 0.1
                    kp_y = 0.1
                    
                    # Calculate new servo positions
                    self.x_center += error_x * kp_x
                    self.y_center += error_y * kp_y
                    
                    # Constrain servo positions
                    self.x_center = max(2.5, min(12.5, self.x_center))
                    self.y_center = max(2.5, min(12.5, self.y_center))
                    
                    # Update servo positions
                    self.set_servo_angle(self.x_pwm, self.x_center)
                    self.set_servo_angle(self.y_pwm, self.y_center)
                    
                    # Draw crosshair at marker center
                    cv2.drawMarker(frame, (marker_center_x, marker_center_y), 
                                   (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                
                # Display the frame
                cv2.imshow('ArUco Marker Tracking', frame)
                
                # Exit on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        except KeyboardInterrupt:
            print("Tracking stopped by user")
        
        finally:
            # Cleanup
            self.cap.release()
            cv2.destroyAllWindows()
            self.x_pwm.stop()
            self.y_pwm.stop()
            GPIO.cleanup()

# Run the tracker
if __name__ == "__main__":
    tracker = ArUcoMarkerTracker()
    tracker.track_aruco_marker()
