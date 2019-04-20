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
    gpio.setmode(gpio.BCM)
    gpio.setup(4, gpio.OUT)
    gpio.setup(17, gpio.OUT)
   # gpio.setup(27, gpio.OUT)
   # gpio.setup(15, gpio.OUT)
   # gpio.setup(12, gpio.OUT)


    gpio.output(4,0)
    #gpio.output(17,1)
 #   gpio.output(27,0)
  #  gpio.output(12,1)
   # time.sleep(2)
   # pwm = gpio.PWM(17, 100)
   # pwm.start(100)
   # pwm.ChangeDutyCycle(50)

    b = True
    while True:
        gpio.output(17,b)
        time.sleep(0.03)
        b = not b

def pwm(time):
    gpio.output(4,0)
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
      pwm = gpio.PWM(12, 1000)
      pwm.start(10)

