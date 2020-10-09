#This file live streams

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from cv2 import aruco
import numpy
import math
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import smbus2
constPi = 3.14159
constVal = 6
lcd_columns = 16
lcd_rows = 2
# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
bus = smbus2.SMBus(1)

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
        #Set time parameters
        end_time = time.time() + self.refresh_speed;
        self.output_num = 0
        #This loop grabs an image, queries for Aruco images, and exits if it finds one.
        #Otherwise, this loop exits after .25 seconds
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

#Actual code that runs on project back-end
x = camera_control()
i = 0
data = 0
reset = 5
while True:
    userVar = input("Enter 0 to return wheel to 0: ")

    x.run_loop()
    if(i % 4 == 0):
        print(x.output_val)
    i += 1
    data = x.output_num
    if (userVar == "r"):
        bus.write_byte(0x08, reset)
        lcd.clear()
        lcd.text_direction = lcd.LEFT_TO_RIGHT;
        lcd.message = "Set Point: Reset"
    else: 
        if(data == 0):
            continue
        bus.write_byte(0x08,data)
        if(data == 1):
            lcd.clear()
            lcd.text_direction = lcd.LEFT_TO_RIGHT;
            lcd.message = "Set Point: 0"
        elif(data == 2):
            lcd.clear()
            lcd.text_direction = lcd.LEFT_TO_RIGHT;
            lcd.message = "Set Point: 1.5707"
        elif(data == 3):
            lcd.clear()
            lcd.text_direction = lcd.LEFT_TO_RIGHT;
            lcd.message = "Set Point: 3.1415"
        elif(data == 4):
            lcd.clear()
            lcd.text_direction = lcd.LEFT_TO_RIGHT;
            lcd.message = "Set Point: 4.7123"
    
        
    position = bus.read_byte(0x08)
    print(position)
    newPosition = ((position * 2 * constPi)/constVal) % (2*constPi)
    lcd.text_direction = lcd.LEFT_TO_RIGHT;
    lcd.message =  "\nPos:" + str(newPosition)
        
    
    
            
                
