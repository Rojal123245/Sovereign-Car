import RPi.GPIO as gpio
from time import time, sleep
import sys
import random
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import math

speed_of_sound = 340
dutyCycle = 100
halt = False
prev_echo = 0
minLineLength = 5
maxLineGap = 10
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
commands = {'right':'1011','left':'1110','forward':'1010','back':'0101','pivotleft':'0110','pivotright':'1001','stop':'0000'}
initTime= None

def init():
    gpio.setwarnings(False)
    gpio.setmode(gpio.BOARD)

    #ir
    gpio.setup(3,gpio.IN)
    gpio.setup(5,gpio.IN)

    #motors
    gpio.setup(7, gpio.OUT)
    gpio.setup(11, gpio.OUT)
    gpio.setup(13, gpio.OUT)
    gpio.setup(15, gpio.OUT )

    #pwm
    gpio.setup(33,gpio.OUT)
    gpio.setup(35,gpio.OUT)

    #ultrasonic
    gpio.setup(40,gpio.IN)
    
    global pwm0, pwm1
    pwm0 = gpio.PWM(33,100)
    pwm0.start(0)

    pwm1 = gpio.PWM(35,100)
    pwm1.start(0)
    

def reset():
    #pwm0.stop()
    #pwm1.stop()
    gpio.cleanup()

reset()
init()

last_command = None
def move(command):
    global last_command
    if command == last_command or command not in commands:
        return
    #print(command)
    
    last_command = command

    pins = [7,11,13,15]

    for i in range(4):
      b = int(commands[command][i]) != 0
      print(command, pins[i],'->',b)
      gpio.output(pins[i],b)



for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
     image = frame.array
     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

  # gray = get_cropped(gray)

     blurred = cv2.GaussianBlur(gray, (5, 5), 0)
     edged = cv2.Canny(blurred, 85, 85)
     lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)

     cv2.imshow("Frame",edged)
     key = cv2.waitKey(1) & 0xFF
     rawCapture.truncate(0)
     if key == ord("q"):
       move('stop')
       break 

     elif key >= ord('0') and key <= ord('9'):
       dutyCycle = (10-(key-48)) * 10

     if not halt:
        ir_left = gpio.input(3)
        ir_right = gpio.input(5)
        out = 2*ir_left + ir_right
        if out == 0:
            move('forward')
        elif out == 1:
            move('pivotright')
            sleep(0.2)
        elif out == 2:
            move('pivotleft')
            sleep(0.2)
        elif out == 3:
            move('stop')

        pwm0.ChangeDutyCycle(dutyCycle)
        pwm1.ChangeDutyCycle(dutyCycle)

     arduino_out = gpio.input(40)

     print(arduino_out)
     
     if arduino_out:
         halt = True
         move('stop')
     else:
         halt = False
        
        
        
        

    




