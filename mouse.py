import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

class BlueObjectTracker:
    def __init__(self, x_servo_pin=20, y_servo_pin=21):
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
        
        # Servo calibration (may need adjustment)
        self.x_center = 7.5  # Neutral position
        self.y_center = 7.5  # Neutral position
        
    def set_servo_angle(self, pwm, angle):
        """Convert angle to duty cycle and set servo position"""
        duty = angle / 18 + 2.5
        pwm.ChangeDutyCycle(duty)
        
    def track_blue_object(self):
        try:
            while True:
                # Capture frame
                ret, frame = self.cap.read()
                if not ret:
                    break
                
                # Convert to HSV color space
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                # Define blue color range in HSV
                lower_blue = np.array([100, 50, 50])
                upper_blue = np.array([140, 255, 255])
                
                # Create mask for blue color
                mask = cv2.inRange(hsv, lower_blue, upper_blue)
                
                # Find contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    # Find the largest blue object
                    largest_contour = max(contours, key=cv2.contourArea)
                    
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    
                    # Draw rectangle around the object
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    
                    # Calculate object center
                    object_center_x = x + w // 2
                    object_center_y = y + h // 2
                    
                    # Calculate frame center
                    frame_center_x = frame.shape[1] // 2
                    frame_center_y = frame.shape[0] // 2
                    
                    # Calculate error (deviation from center)
                    error_x = object_center_x - frame_center_x
                    error_y = object_center_y - frame_center_y
                    
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
                
                # Display the frame
                cv2.imshow('Blue Object Tracking', frame)
                
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
    tracker = BlueObjectTracker()
    tracker.track_blue_object()
