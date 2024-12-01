import pyautogui
import time
from math import floor

class MouseServoSimulator:
    def __init__(self, min_angle=0, max_angle=180, sensitivity=0.2):
        # Configuration
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.sensitivity = sensitivity  # Threshold for angle change detection
        
        # Get screen resolution
        self.screen_width, self.screen_height = pyautogui.size()
        
        # Store last positions and angles to avoid unnecessary updates
        self.last_x = None
        self.last_y = None
        self.last_angle_x = None
        self.last_angle_y = None
        
        # Performance and display settings
        self.update_counter = 0
        self.display_interval = 10  # Reduced update interval for more responsiveness
        self.max_update_frequency = 0.05  # Maximum update frequency (20 times per second)
        
        # Performance tracking
        self.last_update_time = 0

    def map_position_to_angle(self, position, max_position):
        """Map screen position to servo angle with improved precision"""
        mapped_angle = (position / max_position) * (self.max_angle - self.min_angle) + self.min_angle
        return round(mapped_angle, 2)  # Round to two decimal places for precision

    def display_position(self, x, y, angle_x, angle_y):
        """Display a more informative visual representation of servo positions"""
        # Clear screen (cross-platform approach)
        print('\033[2J\033[H', end='')
        
        print("🖱️  Mouse Position Servo Simulator 🤖")
        print("====================================")
        print(f"Screen Resolution: {self.screen_width}x{self.screen_height}")
        print(f"Mouse Position:    X={x:4d}, Y={y:4d}")
        print(f"Servo Angles:      X={angle_x:6.2f}°, Y={angle_y:6.2f}°")
        print("\nVisual Servo Representation:")
        print("X-Axis Servo: ", end='')
        self.draw_servo_position(angle_x)
        print("\nY-Axis Servo: ", end='')
        self.draw_servo_position(angle_y)
        print("\n\n💡 Tip: Move mouse to see real-time servo simulation")
        print("🚪 Press Ctrl+C to exit")

    def draw_servo_position(self, angle):
        """Enhanced ASCII representation of servo position"""
        total_width = 50
        position = int((angle - self.min_angle) / (self.max_angle - self.min_angle) * total_width)
        bar = '█' * position + '|' + '░' * (total_width - position)
        print(f"[{bar}] {angle:6.2f}°")

    def is_significant_change(self, current_x, current_y):
        """Determine if mouse position change is significant"""
        if self.last_x is None or self.last_y is None:
            return True
        
        # Calculate distance moved
        distance = ((current_x - self.last_x) ** 2 + (current_y - self.last_y) ** 2) ** 0.5
        return distance > (self.screen_width * self.sensitivity / 100)

    def track_mouse(self):
        """Optimized main loop for mouse tracking"""
        print("🖱️  Starting mouse tracking simulation...")
        print("Move your mouse to see servo positions.")
        print("Press Ctrl+C to exit.")

        try:
            while True:
                # Get current time and check update frequency
                current_time = time.time()
                if current_time - self.last_update_time < self.max_update_frequency:
                    time.sleep(0.01)  # Small sleep to reduce CPU usage
                    continue

                # Get current mouse position
                x, y = pyautogui.position()

                # Check if position change is significant
                if self.is_significant_change(x, y):
                    # Convert positions to angles
                    angle_x = self.map_position_to_angle(x, self.screen_width)
                    angle_y = self.map_position_to_angle(y, self.screen_height)

                    # Check if angles have changed significantly
                    if (self.last_angle_x is None or 
                        abs(angle_x - self.last_angle_x) > self.sensitivity or
                        self.last_angle_y is None or 
                        abs(angle_y - self.last_angle_y) > self.sensitivity):

                        # Periodic display to reduce screen flickering
                        self.update_counter += 1
                        if self.update_counter % self.display_interval == 0:
                            self.display_position(x, y, angle_x, angle_y)

                        # Update tracking variables
                        self.last_x = x
                        self.last_y = y
                        self.last_angle_x = angle_x
                        self.last_angle_y = angle_y
                        self.last_update_time = current_time

        except KeyboardInterrupt:
            print("\n Stopping mouse tracking simulation...")

def main():
    # Create simulator instance with customizable parameters
    simulator = MouseServoSimulator(
        min_angle=0,      # Minimum servo angle
        max_angle=180,    # Maximum servo angle
        sensitivity=0.2   # Adjust mouse movement sensitivity (0.1 - 1.0)
    )
    
    # Start tracking
    simulator.track_mouse()

if __name__ == "__main__":
    main()
