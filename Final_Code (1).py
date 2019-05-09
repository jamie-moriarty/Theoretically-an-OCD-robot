'''for servo motor'''
import sys
sys.path.append('/home/pi/Adafruit-Raspberry-Pi-Python-Code/Adafruit_PWM_Servo_Driver/')
print(sys.path)
from Adafruit_PWM_Servo_Driver import PWM
import time
'''for motors'''
import RPi.GPIO as GPIO
import time  
'''for circle detection:'''
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import os
import numpy as np
os.system('sudo modprobe bcm2835-v4l2')

GPIO.setmode(GPIO.BCM)

'''Set up pins for Motor 1'''
pinA1 = 23       # IN1, motor 1
pinA2 = 24       # IN2, motor 1
pinAE = 25      # E1, motor 1

GPIO.setup(pinA1, GPIO.OUT)
GPIO.setup(pinA2, GPIO.OUT)
GPIO.setup(pinAE, GPIO.OUT)

'''Set up pins for Motor 2'''
pinB1 = 6     # IN3, motor 2
pinB2 = 12    # IN4, motor 2
pinBE = 5    # E2, motor 2

GPIO.setup(pinB1, GPIO.OUT)
GPIO.setup(pinB2, GPIO.OUT)
GPIO.setup(pinBE, GPIO.OUT)

'''Set up pins for top and bottom button'''
pinTOP = 27         #grey wire - top
pinBOT = 22         #white wire - bottom

GPIO.setup(pinTOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pinBOT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

'''servo set up'''
pwm = PWM(0x40)
servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  #print("%d us per period", % pulseLength)
  pulseLength /= 4096                     # 12 bits of resolution
  #print("%d us per bit" ,% pulseLength)
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

pwm.setPWMFreq(60)

def servo_off():
  pwm.setPWM(0,0,0)
        
def lift_up():
  pwm.setPWM(0,0,servoMax)
  print("lifting up")

def set_down():
  pwm.setPWM(0,0,servoMin)
  print("setting ball down")

'''detection set up'''
h=200
w=300
centerx = w/2
centery = h/2
camera = PiCamera()
camera.resolution = (w, h)
camera.framerate = 16
rawCapture = PiRGBArray(camera, size=(w, h))
time.sleep(0.1)

'''blob detection set up'''

params = cv2.SimpleBlobDetector_Params()

params.minThreshold = 10
params.maxThreshold = 250

params.filterByArea = True
params.minArea = 1500
params.maxArea = 1000000

params.filterByCircularity = False
params.minConvexity = .87

params.filterByInertia = False
params.minInertiaRatio = .01

params.filterByColor = True
params.blobColor = 255
params.filterByConvexity = False

detector = cv2.SimpleBlobDetector_create(params)

'''dc motor driver set up'''
def set_for(motor):
    if motor == 1:
        GPIO.output(pinAE, 0)
        GPIO.output(pinA1, 1)
        GPIO.output(pinA2, 0)
        GPIO.output(pinAE, 1)
    elif motor ==2:
        GPIO.output(pinBE, 0)
        GPIO.output(pinB1, 1)
        GPIO.output(pinB2, 0)
        GPIO.output(pinBE, 1)

def set_back(motor):
    if motor == 1:
        GPIO.output(pinAE, 0)
        GPIO.output(pinA1, 0)
        GPIO.output(pinA2, 1)
        GPIO.output(pinAE, 1)
    elif motor ==2:
        GPIO.output(pinBE, 0)
        GPIO.output(pinB1, 0)
        GPIO.output(pinB2, 1)
        GPIO.output(pinBE, 1)

def rolling_stop(motor):
    if motor == 1:
        GPIO.output(pinA1, 1)
        GPIO.output(pinA2, 1)
        GPIO.output(pinAE, 0)
    elif motor ==2:
        GPIO.output(pinB1, 1)
        GPIO.output(pinB2, 1)
        GPIO.output(pinBE, 0)
    
def shutoff():
    rolling_stop(1)
    rolling_stop(2)
    print("motors are off")

'''ultrasonic range finder'''
GPIO_TRIGGER = 13
GPIO_ECHO = 16

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
 
def distance():
    GPIO.output(GPIO_TRIGGER, 1)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, 0)

    StartTime = time.time()
    StopTime = time.time()

    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2
    distance = round(distance, 2)
    return distance


#hsv - hue, saturation, value (3 nos 0-255)

def find_yellow_ball():
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image_RGB = frame.array
        new_image_RGB = cv2.flip(image_RGB, -1) #for upside down camera
        copy_RGB = new_image_RGB.copy()
        grey = cv2.cvtColor(new_image_RGB, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("flipped grey", grey)
        blur = cv2.GaussianBlur(new_image_RGB, (15,15), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        #lower_yellow = np.array([15, 200, 40])
        #upper_yellow = np.array([40, 255, 255])
        lower_yellow = np.array([10, 220, 100])
        upper_yellow = np.array([50, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        #mask = cv2.erode(mask, None, iterations = 2)
        #mask = cv2.dilate(mask, None, iterations = 2)
        cv2.imshow("mask", mask)
        img_circles = None
        img_circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 150,
                                        param1=50, param2=10, minRadius=20, maxRadius=200)
        if img_circles is not None:
            img_circles = np.uint16(np.around(img_circles))
            try:
                print(hsv[img_circles[0,0][0], img_circles[0,0][1]])
            except:
                print("error")
            i = img_circles[0,0]
            #draw outer circle
            cv2.circle(copy_RGB, (i[0], i[1]), i[2], (0,255,0), 2)
            #draw center of circle
            cv2.circle(copy_RGB, (i[0], i[1]), 2, (0,0,255), 3)
            print(i[0], i[1], i[2])
            
            if (i[2] < 40):
                start=time.time()
                while time.time() < start+.15:
                    set_for(1)
                    set_for(2)
                    print("full speed ahead")
                shutoff()
            if (i[2] >= 40) :
                start=time.time()
                while time.time() < start+.1:
                    set_for(1)
                    set_for(2)
                    print("SLOW speed ahead")
                shutoff()
            if i[0] > (centerx + 50):
                start=time.time()
                while time.time() < start+.15:
                    set_for(1)
                    print("moving towards centerpt x")
                shutoff()
            if i[0] < (centerx - 50):
                start=time.time()
                while time.time() < start+.15:
                    set_for(2)
                    print("moving towards centerpt x")
                shutoff()
            if i[0] > (centery + 50):
                start=time.time()
                while time.time() < start+.1:
                    set_for(1)
                    print("moving towards centerpt y")
                shutoff()
            if i[0] < (centery - 50):
                start=time.time()
                while time.time() < start+.1:
                    set_for(2)
                    print("moving towards centerpt y")
                shutoff()
            if distance() < 8.5:
                shutoff()
                print(distance())
                print('close enough to object')
                return 1

        cv2.imshow("Copy with Detected Object",copy_RGB)
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
    rawCapture.truncate(0)

def find_green_stuff():
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image_RGB = frame.array
        new_image_RGB = cv2.flip(image_RGB, -1) #for upside down camera
        copy_RGB = new_image_RGB.copy()
        grey = cv2.cvtColor(new_image_RGB, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(new_image_RGB, (15,15), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lower_GREEN = np.array([40, 220, 100])         #can 
        upper_GREEN = np.array([80, 255, 255])         #can
        mask = cv2.inRange(hsv, lower_GREEN, upper_GREEN)

        keypoints = None
        keypoints = detector.detect(mask)

        if keypoints is not None:
            copy_RGB = cv2.drawKeypoints(copy_RGB, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            print('keypoints drawn')
            print(keypoints)
            #print(dir(keypoints))
            for point in keypoints:
                can_x = int(point.pt[0])
                can_y = int(point.pt[1])
                print(point.pt[0], point.pt[1])
                cv2.circle(copy_RGB, (can_x, can_y), 2, (255,0, 0), 2)

                if can_x > (centerx + 50):
                    start=time.time()
                    while time.time() < start+.15:
                        set_for(1)
                        print("moving towards centerpt x")
                    shutoff()
                if can_x < (centerx - 50):
                    start=time.time()
                    while time.time() < start+.15:
                        set_for(2)
                        print("moving towards centerpt x")
                    shutoff()
                if can_x > (centerx - 50) and can_x < (centerx +50) and distance() > 12:
                    start=time.time()
                    while time.time() < start+.6:
                        set_for(1)
                        set_for(2)
                        print("full speed ahead")
                    shutoff()
                if distance() >= 9.7 and distance() <= 12 :
                    start=time.time()
                    while time.time() < start+.1:
                        set_for(1)
                        set_for(2)
                        print(distance())
                        print("SLOW speed ahead")
                    shutoff()
                if distance() < 9.7:
                    shutoff()
                    print(distance())
                    print('close enough to object')
                    return 1
        cv2.imshow("Copy with Detected Object",copy_RGB)
        cv2.imshow("Mask",mask)
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)



def pick_up_ball():
    while True:
      input_top = GPIO.input(pinTOP)
      if input_top == False:
          lift_up()
          print('no button, lifting')
      if input_top == True:
          servo_off()
          print('button hit')
          return 1

def set_down_ball():                  #MIGHT NOT HIT BUTTON IF NOT PERFECTLY ALIGNED WITH CAN
    print('going to set down')
    while True:
      input_bot = GPIO.input(pinBOT)
      print(input_bot)
      start=time.time()
      while input_bot == False:
          print('no button')
          set_down()
          the_time = time.time()
          if (the_time - start) > 0.85:
              print('.5 seconds elapsed, backing up')
              set_back(1)
              set_back(2)
          if (the_time - start) > 1.7:
              print('too much time has passed, turning off')
              servo_off()
              shutoff()
              return 
      if input_bot == True:
          shutoff()
          servo_off()
          print('button hit, turning off')
          return 1

mission_accomplished = 0
while mission_accomplished == 0:
    input_top = GPIO.input(pinTOP)
    input_bot = GPIO.input(pinBOT)
    if input_top ==False and input_bot == True:
        print('going to pick up ball')
        find_yellow_ball()
        rawCapture.truncate(0)
        print('going to pick up')
        pick_up_ball()
    if input_top ==False and input_bot == False:
        print('one of buttons not pressed, try up and down')
        pick_up_ball()
        set_down_ball()
        if input_top ==False and input_bot == False:
            pick_up_ball()
            find_green_stuff()
            set_down_ball()
    if input_top ==True:
        print('going to find the green can')
        find_green_stuff()
        rawCapture.truncate(0)
        print('going to set down')
        set_down_ball()
        if input_bot == False:
            start = time.time()
            while time.time() < start + .5:
                set_back(1)
                set_back(2)
                if input_bot == True:
                    break
                if input_top == True:
                    set_down_ball()
            shutoff()
            servo_off()
            mission_accomplished = 1
        if input_bot == True:
            mission_accomplished = 1

#pick_up_ball()
#set_down_ball()
#find_green_stuff()

cv2.destroyAllWindows()
GPIO.cleanup() # cleanup all GPIO 






