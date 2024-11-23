import pyautogui
import time
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
        
    def map_position_to_angle(self, position, max_position):
        """Map screen position to servo angle"""
        return (position / max_position) * (self.max_angle - self.min_angle) + self.min_angle
    
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
        """Main loop to track mouse position and simulate servos"""
        print("Starting mouse tracking simulation.")
        print("Move your mouse around the screen to see servo positions.")
        print("Press Ctrl+C to exit.")
        
        try:
            while True:
                # Get current mouse position
                x, y = pyautogui.position()
                
                # Convert positions to angles
                angle_x = self.map_position_to_angle(x, self.screen_width)
                angle_y = self.map_position_to_angle(y, self.screen_height)
                
                # Check if angles have changed significantly
                if (self.last_angle_x is None or abs(angle_x - self.last_angle_x) > 0.1 or
                    self.last_angle_y is None or abs(angle_y - self.last_angle_y) > 0.1):
                    
                    # Display the position
                    self.display_position(x, y, angle_x, angle_y)
                    
                    self.last_angle_x = angle_x
                    self.last_angle_y = angle_y
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.0001)
                
        except KeyboardInterrupt:
            print("\nStopping mouse tracking simulation...")

def main():
    # Create simulator instance
    simulator = MouseServoSimulator(
        min_angle=0,    # Minimum servo angle
        max_angle=180   # Maximum servo angle
    )
    
    # Start tracking
    simulator.track_mouse()

if __name__ == "__main__":
    main()