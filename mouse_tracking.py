import pyautogui
import time
import RPi.GPIO as GPIO
from math import floor

class MouseServoTracking:
    def __init__(self, min_angle=0, max_angle=180):
        self.min_angle = min_angle
        self.max_angle = max_angle
        
        self.screen_width, self.screen_height = pyautogui.size()
        
        self.last_angle_x = None
        self.last_angle_y = None
        
        self.update_counter = 0
        self.display_interval = 10
        

        GPIO.setmode(GPIO.BCM)
        self.servo_pin_x = 20
        self.servo_pin_y = 21
        GPIO.setup(self.servo_pin_x, GPIO.OUT)
        GPIO.setup(self.servo_pin_y, GPIO.OUT)
        
 
        self.pwm_x = GPIO.PWM(self.servo_pin_x, 50) 
        self.pwm_y = GPIO.PWM(self.servo_pin_y, 50)
        self.pwm_x.start(12.5)
        self.pwm_y.start(12.5)
        
    def map_position_to_angle(self, position, max_position):
        return (position / max_position) * (self.max_angle - self.min_angle) + self.min_angle
    
    def set_servo_angle(self, pwm, angle):
        
        angle = max(0, min(180, angle)) 
        duty = 2.5 + (angle / 18.0) 
        pwm.ChangeDutyCycle(duty)
        time.sleep(0.02)
    
    def display_position(self, x, y, angle_x, angle_y):
        if self.update_counter % self.display_interval == 0:
            print('\033[2J\033[H', end='')
            print("Posicao mouse")
            print("============================")
            print(f"Resolucao tela: {self.screen_width}x{self.screen_height}")
            print(f"Posicao mouse: X={x}, Y={y}")
            print(f"Angulos: X={angle_x:.1f}°, Y={angle_y:.1f}°")
            print()
            print("X-Axis Servo: ", end='')
            self.draw_servo_position(angle_x)
            print("\nY-Axis Servo: ", end='')
            self.draw_servo_position(angle_y)
            print("\n\nCtrl+C para sair")
        
        self.update_counter += 1

    def draw_servo_position(self, angle):
        total_width = 50
        position = int((angle - self.min_angle) / (self.max_angle - self.min_angle) * total_width)
        bar = '-' * position + '|' + '-' * (total_width - position)
        print(f"[{bar}] {angle:.1f}°")

    def track_mouse(self):

        print("Iniciando mouse tracking.")
        print("Mova o mouse.")
        
        try:
            while True:
                x, y = pyautogui.position()
                
                angle_x = self.map_position_to_angle(x, self.screen_width)
                angle_y = self.map_position_to_angle(y, self.screen_height)
                
                angle_x = max(0, min(180, angle_x))
                angle_y = max(0, min(180, angle_y))
                
                if angle_x != self.last_angle_x:
                    self.set_servo_angle(self.pwm_x, angle_x)
                    self.last_angle_x = angle_x
                if angle_y != self.last_angle_y:
                    self.set_servo_angle(self.pwm_y, angle_y)
                    self.last_angle_y = angle_y
                
                self.display_position(x, y, angle_x, angle_y)
                
                time.sleep(0.1)

        except KeyboardInterrupt:
            pass
        finally:
            self.pwm_x.stop()
            self.pwm_y.stop()
            GPIO.cleanup()
            print("Movimento servo finalizado.")

if __name__ == "__main__":
    simulator = MouseServoTracking()
    simulator.track_mouse()