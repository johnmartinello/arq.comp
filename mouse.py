import pyautogui
import time
import RPi.GPIO as GPIO
from math import floor

class MouseServoSimulator:
    def __init__(self, min_angle=0, max_angle=180):
        # Configuration
        self.min_angle = min_angle
        self.max_angle = max_angle
        
        # Get screen resolution
        self.screen_width, self.screen_height = pyautogui.size()
        
        # Store last angles to avoid unnecessary updates
        self.last_angle_x = None
        self.last_angle_y = None
        
        # Visual display settings
        self.update_counter = 0
        self.display_interval = 20  # Update display every 20 iterations
        
        # GPIO setup for servos
        GPIO.setmode(GPIO.BCM)
        self.servo_pin_x = 20
        self.servo_pin_y = 21
        GPIO.setup(self.servo_pin_x, GPIO.OUT)
        GPIO.setup(self.servo_pin_y, GPIO.OUT)
        
        # Initialize PWM for servos
        self.pwm_x = GPIO.PWM(self.servo_pin_x, 50)  # 50Hz frequency
        self.pwm_y = GPIO.PWM(self.servo_pin_y, 50)
        self.pwm_x.start(7.5)  # Center position (90 degrees)
        self.pwm_y.start(7.5)
        
    def map_position_to_angle(self, position, max_position):
        """Map screen position to servo angle"""
        return (position / max_position) * (self.max_angle - self.min_angle) + self.min_angle
    
    def set_servo_angle(self, pwm, angle):
        """Set the servo angle"""
        duty = 2.5 + (angle / 18.0)  # Convert angle to duty cycle
        pwm.ChangeDutyCycle(duty)
        time.sleep(0.02)
    
    def display_position(self, x, y, angle_x, angle_y):
        """Display a visual representation of the servo positions"""
        # Clear screen (Windows)
        if self.update_counter % self.display_interval == 0:
            print('\033[2J\033[H', end='')  # Clear screen and move cursor to top
            print("Mouse Position Servo Simulator")
            print("============================")
            print(f"Screen Resolution: {self.screen_width}x{self.screen_height}")
            print(f"Mouse Position: X={x}, Y={y}")
            print(f"Servo Angles: X={angle_x:.1f}°, Y={angle_y:.1f}°")
            print("\nVisual Representation:")
            print("X-Axis Servo: ", end='')
            self.draw_servo_position(angle_x)
            print("\nY-Axis Servo: ", end='')
            self.draw_servo_position(angle_y)
            print("\n\nPress Ctrl+C to exit")
        
        self.update_counter += 1

    def draw_servo_position(self, angle):
        """Draw a simple ASCII representation of servo position"""
        total_width = 50
        position = int((angle - self.min_angle) / (self.max_angle - self.min_angle) * total_width)
        bar = '-' * position + '|' + '-' * (total_width - position)
        print(f"[{bar}] {angle:.1f}°")

    def track_mouse(self):
        """Main loop to track mouse position and control servos"""
        print("Starting mouse tracking simulation.")
        print("Move your mouse around the screen to see servo positions.")
        print("Press Ctrl+C to exit.")
        
        try:
            while True:
                # Get current mouse position
                x, y = pyautogui.position()
                
                # Map positions to angles
                angle_x = self.map_position_to_angle(x, self.screen_width)
                angle_y = self.map_position_to_angle(y, self.screen_height)
                
                # Update servos
                if angle_x != self.last_angle_x:
                    self.set_servo_angle(self.pwm_x, angle_x)
                    self.last_angle_x = angle_x
                if angle_y != self.last_angle_y:
                    self.set_servo_angle(self.pwm_y, angle_y)
                    self.last_angle_y = angle_y
                
                # Display positions
                self.display_position(x, y, angle_x, angle_y)
                
                # Short delay
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            # Clean up GPIO on exit
            self.pwm_x.stop()
            self.pwm_y.stop()
            GPIO.cleanup()
            print("Servo control stopped.")

if __name__ == "__main__":
    simulator = MouseServoSimulator()
    simulator.track_mouse()
