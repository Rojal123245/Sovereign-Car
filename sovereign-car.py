import RPi.GPIO as gpio
import time
import sys
import random
from picamera.array import PiRGBArray
#import RPi.GPIO as GPIO
from picamera import PiCamera
import cv2
import numpy as np
import math

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(7, gpio.OUT)
    gpio.setup(11, gpio.OUT)
    gpio.setup(13, gpio.OUT)
    gpio.setup(15, gpio.OUT)
def reset():
    gpio.cleanup()
    
init()

cmd = {'right':'1011','left':'1110','front':'1010','back':'0101','pivotleft':'0110','pivotright':'1001','stop':'0000'}

last_key = None
def decode(key):
   global last_key
   if last_key == key or key not in cmd:
          return

   print(key)

   last_key = key
   reset()
   init()
   pins = [7,11,13,15]
   for i in range(4):
      b = int(cmd[key][i]) != 0
      print(pins[i],'->',b)
      gpio.output(pins[i],b)
      

theta=0
minLineLength = 5
maxLineGap = 10
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
   image = frame.array
   gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
   blurred = cv2.GaussianBlur(gray, (5, 5), 0)
   edged = cv2.Canny(blurred, 85, 85)
   lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)
   if lines is not None:
       for x in range(0, len(lines)):
           for x1,y1,x2,y2 in lines[x]:
               cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
               theta=theta+math.atan2((y2-y1),(x2-x1))
   #print(theta)GPIO pins were connected to arduino for servo steering control
   threshold=10
   print(theta)
   if(theta>threshold):
      decode('pivotleft')
      print("left")
   if(theta<-threshold):
      decode('pivotright')
      print("right")
   if(abs(theta)<threshold):
      decode('front')
      print ("straight")
   theta=0
   cv2.imshow("Frame",image)
   key = cv2.waitKey(1) & 0xFF
   rawCapture.truncate(0)
   if key == ord("q"):
       decode('stop')
       break
   if key == ord('w'):
       decode('front')
   elif key == ord('a'):
       decode('pivotleft')
   elif key == ord('s'):
       decode('pivotright')
   elif key == ord('z'):
       decode('back')
