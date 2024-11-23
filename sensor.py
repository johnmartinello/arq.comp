import pyautogui
import time
from math import floor
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("RPi.GPIO not found - running in test mode")

class MouseServoController:
    def __init__(self, servo_pin_x=18, servo_pin_y=23, min_angle=0, max_angle=180, test_mode=False):
        self.test_mode = test_mode or not GPIO_AVAILABLE
        
        if not self.test_mode:
            # Initialize GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(servo_pin_x, GPIO.OUT)
            GPIO.setup(servo_pin_y, GPIO.OUT)
            
            # Setup servo PWM
            self.servo_x = GPIO.PWM(servo_pin_x, 50)  # 50Hz frequency
            self.servo_y = GPIO.PWM(servo_pin_y, 50)
            self.servo_x.start(0)
            self.servo_y.start(0)
        
        # Configuration
        self.servo_pin_x = servo_pin_x
        self.servo_pin_y = servo_pin_y
        self.min_angle = min_angle
        self.max_angle = max_angle
        
        # Get screen resolution
        self.screen_width, self.screen_height = pyautogui.size()
        
        # Store last angles to avoid unnecessary updates
        self.last_angle_x = None
        self.last_angle_y = None
        
    def angle_to_duty_cycle(self, angle):
        """Convert angle (0-180) to duty cycle (2-12)"""
        duty = angle / 18 + 2
        return duty
        
    def map_position_to_angle(self, position, max_position):
        """Map screen position to servo angle"""
        return (position / max_position) * (self.max_angle - self.min_angle) + self.min_angle
    
    def move_servos(self, angle_x, angle_y):
        """Move servos to specified angles"""
        # Ensure angles are within bounds
        angle_x = max(self.min_angle, min(self.max_angle, angle_x))
        angle_y = max(self.min_angle, min(self.max_angle, angle_y))
        
        # Check if angles have changed significantly (avoid tiny movements)
        if (self.last_angle_x is None or abs(angle_x - self.last_angle_x) > 0.5 or
            self.last_angle_y is None or abs(angle_y - self.last_angle_y) > 0.5):
            
            if self.test_mode:
                print(f"Target Position - X: {angle_x:.1f}°, Y: {angle_y:.1f}°")
            else:
                duty_x = self.angle_to_duty_cycle(angle_x)
                duty_y = self.angle_to_duty_cycle(angle_y)
                self.servo_x.ChangeDutyCycle(duty_x)
                self.servo_y.ChangeDutyCycle(duty_y)
                time.sleep(0.05)  # Small delay to allow servos to reach position
            
            self.last_angle_x = angle_x
            self.last_angle_y = angle_y
        
    def track_mouse(self):
        """Main loop to track mouse position and control servos"""
        print(f"Starting mouse tracking in {'TEST' if self.test_mode else 'HARDWARE'} mode.")
        print("Move your mouse around the screen to control servos.")
        print("Press Ctrl+C to exit.")
        
        try:
            while True:
                # Get current mouse position
                x, y = pyautogui.position()
                
                # Convert positions to angles
                angle_x = self.map_position_to_angle(x, self.screen_width)
                angle_y = self.map_position_to_angle(y, self.screen_height)
                
                # Move servos (or print in test mode)
                self.move_servos(angle_x, angle_y)
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nStopping mouse tracking...")
            self.cleanup()
            
    def cleanup(self):
        """Cleanup GPIO on program exit"""
        if not self.test_mode:
            self.servo_x.stop()
            self.servo_y.stop()
            GPIO.cleanup()

def main():
    # Check if running on Raspberry Pi or in test mode
    test_mode = not GPIO_AVAILABLE
    
    # Create controller instance
    controller = MouseServoController(
        servo_pin_x=18,    # GPIO pin number for X-axis servo
        servo_pin_y=23,    # GPIO pin number for Y-axis servo
        min_angle=0,       # Minimum servo angle
        max_angle=180,     # Maximum servo angle
        test_mode=test_mode
    )
    
    # Start tracking
    controller.track_mouse()

if __name__ == "__main__":
    main()