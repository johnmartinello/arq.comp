# -*- coding: utf-8 -*-
"""
ArUco Marker Tracking with Servo Control using RPi.GPIO
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import RPi.GPIO as GPIO
import math
from time import sleep

print("Starting")

def findArucoMarkers(img, markerSize=4, totalMarkers=250, draw=True):
    arucoDict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    arucoParam = aruco.DetectorParameters_create()
    bbox, ids, rejected = aruco.detectMarkers(img, arucoDict, parameters=arucoParam)
    
    if draw:
        aruco.drawDetectedMarkers(img,bbox)
    return bbox, ids, rejected

def getCentre(bboxtl,bboxbr):
        centre = int((bboxtl[0]+bboxbr[0])/2), int((bboxtl[1]+bboxbr[1])/2)
        return centre

def getDesired(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Get aruco
    bbox, ids, rejected = findArucoMarkers(img, markerSize=4, totalMarkers=250, draw=True)
    
    if np.any(ids == None):
        ret = 0
        centre = None
    else:
        bboxtl = bbox[0][0][0][0],bbox[0][0][0][1]
        bboxbr = bbox[0][0][2][0],bbox[0][0][2][1]

        centre = getCentre(bboxtl,bboxbr)
        ret = 1
        print(centre)
    return centre, ret

class arucoMarker():
    def __init__(self):
        self.bboxtl = None
        self.bboxbr = None
        self.centre = None
    
def getArucos(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Get aruco
    arucos = list()
    bboxs, ids, rejected = findArucoMarkers(img, markerSize=4, totalMarkers=250, draw=True)
    
    if np.any(ids == None):
        ret = 0
    else:
        for bbox in bboxs:
            aruco = arucoMarker()
            aruco.bboxtl = bbox[0][0][0],bbox[0][0][1]
            aruco.bboxbr = bbox[0][2][0],bbox[0][2][1]
            aruco.centre = getCentre(aruco.bboxtl,aruco.bboxbr)
            
            arucos.append(aruco)
        
        ret = 1
    return arucos, ret

def getAdjustment(windowMax, x):
    normalised_adjustment = x/windowMax - 0.5
    adjustment_magnitude = abs(round(normalised_adjustment,1))

    if normalised_adjustment>0:
        adjustment_direction = -1
    else:
        adjustment_direction = 1
        
    return adjustment_magnitude, adjustment_direction

def rescale_frame(frame, percent):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

# Setup GPIO for servo control
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.PWM)  # X-axis servo on GPIO 20
GPIO.setup(21, GPIO.PWM)  # Y-axis servo on GPIO 21

# Initialize PWM
servo_x = GPIO.PWM(20, 50)  # 50 Hz (20ms PWM period)
servo_y = GPIO.PWM(21, 50)  # 50 Hz (20ms PWM period)
servo_x.start(7.5)  # Start at neutral position
servo_y.start(7.5)  # Start at neutral position

# Constants for servo control
cx = -1  # Have to change sign because this servo rotates in the wrong direction
cy = 1

Kp = 80
Kd = 10

# Duty cycle conversion function
def angle_to_duty_cycle(angle):
    # Convert angle to duty cycle (0-180 degrees)
    # Typically 2.5% duty cycle is 0 degrees, 12.5% is 180 degrees
    duty = 2.5 + (angle / 180.0) * 10.0
    return min(max(duty, 2.5), 12.5)

# Servo angle and variables
servo1_now = 0
servo2_now = 0
sleep(2)
print("Initialized servos.")

# Open video capture
vid = cv2.VideoCapture(0)
print("Video started")

try:
    while(True):
        # Get image
        ret, img = vid.read()
        img = rescale_frame(img,50)
        window = img.shape
        
        # Get arucos
        arucos, ret = getArucos(img)
        
        if ret == 0:
            pass
        else:
            # Calculate AB (pixel error)
            A = (0,0)
            B = arucos[0].centre
            
            # show image
            cv2.imshow("image",img)
            cv2.waitKey(1)
            
            # Get adjustment
            xmag, xdir = getAdjustment(window[0],B[1])
            ymag, ydir = getAdjustment(window[1],B[0])

            if xmag is not None:
                # Proportional
                adj_Kpx = cx*Kp*xdir*xmag
                adj_Kpy = cy*Kp*ydir*ymag
                
                # Derivative
                xmag_old = xmag
                ymag_old = ymag
                
                adj_Kdx = cx*Kd*xdir*(xmag-xmag_old)
                adj_Kdy = cy*Kd*ydir*(ymag-ymag_old)
                            
                # Adjustment
                adjustment_x = adj_Kpx + adj_Kdx
                adjustment_y = adj_Kpy + adj_Kdy
                
                # Update servo angles
                servo1_now = servo1_now + adjustment_x
                servo2_now = servo2_now + adjustment_y
                
                # Reset line of sight if instructed to look out of bounds            
                if (servo1_now > 90 or servo1_now < -90):
                    servo1_now = 0
                if (servo2_now > 90 or servo2_now < -90):
                    servo2_now = 0

                # Convert angles to duty cycles and set servos
                servo_x.ChangeDutyCycle(angle_to_duty_cycle(servo1_now))
                servo_y.ChangeDutyCycle(angle_to_duty_cycle(servo2_now))
                sleep(0.00001)
            
            xmag = 0
            xdir = 0
            ymag = 0
            ydir = 0

except KeyboardInterrupt:
    # Clean up on keyboard interrupt
    servo_x.stop()
    servo_y.stop()
    GPIO.cleanup()
    vid.release()
    cv2.destroyAllWindows()
