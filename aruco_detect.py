#This file live streams

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from cv2 import aruco
import numpy
import math

#This is the class in which the loop that controls the camera is defined
class camera_control:
    def __init__(self):
        self.initialize_camera()
        self.output_val = "No Aruco Detected"
        self.refresh_speed = .25
        self.output_num = 0
        
    def initialize_camera(self):
        # initialize the camera and grab a reference to the raw camera capture
        self.camera = PiCamera()
        self.rawCapture = PiRGBArray(self.camera)
        self.camera.iso = 100
        # Wait for the automatic gain control to settle
        time.sleep(2)
        # Now fix the values
        self.camera.shutter_speed = self.camera.exposure_speed
        self.camera.exposure_mode = 'off'
        g = self.camera.awb_gains
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = g
         
        # allow the camera to warmup
        time.sleep(0.1)
        self.camera.framerate = 32
    
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        
        self.parameters = aruco.DetectorParameters_create()
        
    def run_loop(self):
        end_time = time.time() + self.refresh_speed;
        self.output_num = 0
        while(time.time() < end_time):
            self.camera.capture(self.rawCapture, format="bgr")
            image = self.rawCapture.array
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(image, self.aruco_dict, parameters=self.parameters)
            if(type(ids) == numpy.ndarray):
                self.output_num = 1
                xcenter = (corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0] + corners[0][0][3][0])/4
                ycenter = (corners[0][0][0][1]+corners[0][0][1][1]+corners[0][0][2][1] + corners[0][0][3][1])/4
                if (ycenter > image.shape[0]/2):
                    self.output_num += 1
                if (xcenter > image.shape[1]/2):
                    self.output_num += 2
                self.rawCapture.truncate(0)
                break;
            self.rawCapture.truncate(0)
        if(self.output_num == 0):
            self.output_val = "No Aruco Found"
        elif(self.output_num == 1):
            self.output_val = "NW"
        elif(self.output_num == 2):
            self.output_val = "SW"
        elif(self.output_num == 3):
            self.output_val = "NE"
        elif(self.output_num == 4):
            self.output_val = "SE"
        else:
            self.output_val = "Error"
        print(self.output_val)

x = camera_control()
x.run_loop()
               
               
                