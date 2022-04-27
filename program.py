from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import cv2
import cv2.cv as cv
import numpy as np

GPIO.setmode(GPIO.BOARD)
GPIO_TRIGGER1 = 29      
GPIO_ECHO1 = 31
GPIO_TRIGGER2 = 36
GPIO_ECHO2 = 37
GPIO_TRIGGER3 = 33      
GPIO_ECHO3 = 35
MOTOR1B=18  
MOTOR1E=22
MOTOR2B=21  
MOTOR2E=19
LED_PIN=13  


GPIO.setup(GPIO_TRIGGER1,GPIO.OUT)  
GPIO.setup(GPIO_ECHO1,GPIO.IN)      
GPIO.setup(GPIO_TRIGGER2,GPIO.OUT)  
GPIO.setup(GPIO_ECHO2,GPIO.IN)
GPIO.setup(GPIO_TRIGGER3,GPIO.OUT)  
GPIO.setup(GPIO_ECHO3,GPIO.IN)
GPIO.setup(LED_PIN,GPIO.OUT)
GPIO.output(GPIO_TRIGGER1, False)
GPIO.output(GPIO_TRIGGER2, False)
GPIO.output(GPIO_TRIGGER3, False)


def sonar(GPIO_TRIGGER,GPIO_ECHO):
      start=0
      stop=0
GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  
GPIO.setup(GPIO_ECHO,GPIO.IN)           
GPIO.output(GPIO_TRIGGER, False)
time.sleep(0.01)

      while distance > 5:

GPIO.output(GPIO_TRIGGER, True)
time.sleep(0.00001)
GPIO.output(GPIO_TRIGGER, False)
            begin = time.time()
      while GPIO.input(GPIO_ECHO)==0 and time.time()<begin+0.05:
            start = time.time()

      while GPIO.input(GPIO_ECHO)==1 and time.time()<begin+0.1:
            stop = time.time()

      elapsed = stop-start
      distance = elapsed * 34000
      distance = distance / 2

      print ("Distance : %.1f" % distance)
      return distance

GPIO.setup(MOTOR1B, GPIO.OUT)
GPIO.setup(MOTOR1E, GPIO.OUT)
GPIO.setup(MOTOR2B, GPIO.OUT)
GPIO.setup(MOTOR2E, GPIO.OUT)

def forward():
GPIO.output(MOTOR1B, GPIO.HIGH)
GPIO.output(MOTOR1E, GPIO.LOW)
GPIO.output(MOTOR2B, GPIO.HIGH)
GPIO.output(MOTOR2E, GPIO.LOW)

def reverse():
GPIO.output(MOTOR1B, GPIO.LOW)
GPIO.output(MOTOR1E, GPIO.HIGH)
GPIO.output(MOTOR2B, GPIO.LOW)
GPIO.output(MOTOR2E, GPIO.HIGH)

def rightturn():
GPIO.output(MOTOR1B,GPIO.LOW)
GPIO.output(MOTOR1E,GPIO.HIGH)
GPIO.output(MOTOR2B,GPIO.HIGH)
GPIO.output(MOTOR2E,GPIO.LOW)

def leftturn():
GPIO.output(MOTOR1B,GPIO.HIGH)
GPIO.output(MOTOR1E,GPIO.LOW)
GPIO.output(MOTOR2B,GPIO.LOW)
GPIO.output(MOTOR2E,GPIO.HIGH)

def stop():
GPIO.output(MOTOR1E,GPIO.LOW)
GPIO.output(MOTOR1B,GPIO.LOW)
GPIO.output(MOTOR2E,GPIO.LOW)
GPIO.output(MOTOR2B,GPIO.LOW)

def segment_colour(frame):    
hsv_roi=  cv2.cvtColor(frame, cv2.cv.CV_BGR2HSV)
    mask_1 = cv2.inRange(hsv_roi, np.array([160, 160,10]), np.array([190,255,255]))
ycr_roi=cv2.cvtColor(frame,cv2.cv.CV_BGR2YCrCb)
    mask_2=cv2.inRange(ycr_roi, np.array((0.,165.,0.)), np.array((255.,255.,255.)))

    mask = mask_1 | mask_2
kern_dilate = np.ones((8,8),np.uint8)
kern_erode  =np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,kern_erode)      
    mask=cv2.dilate(mask,kern_dilate)     
    return mask

def find_blob(blob): 
largest_contour=0
cont_index=0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area >largest_contour) :
largest_contour=area

cont_index=idx
            if res>15 and res<18:
cont_index=idx

    r=(0,0,2,2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])

    return r,largest_contour

def target_hist(frame):
hsv_img=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hist=cv2.calcHist([hsv_img],[0],None,[50],[0,255])
    return hist


camera = PiCamera()
camera.resolution = (160, 120)
camera.framerate = 16
rawCapture = PiRGBArray(camera, size=(160, 120))
time.sleep(0.001)


for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

      frame = image.array
      frame=cv2.flip(frame,1)
      global centre_x
      global centre_y
centre_x=0.
centre_y=0.
      hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask_red=segment_colour(frame)      
loct,area=find_blob(mask_red)
x,y,w,h=loct

distanceC = sonar(GPIO_TRIGGER2,GPIO_ECHO2)
distanceR = sonar(GPIO_TRIGGER3,GPIO_ECHO3)
distanceL = sonar(GPIO_TRIGGER1,GPIO_ECHO1)

      if (w*h) < 10:
            found=0
      else:
            found=1
            simg2 = cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
centre_x=x+((w)/2)
centre_y=y+((h)/2)
            cv2.circle(frame,(int(centre_x),int(centre_y)),3,(0,110,255),-1)
centre_x-=80
centre_y=6--centre_y
            print (centre_x,centre_y)
      initial=400
      flag=0
GPIO.output(LED_PIN,GPIO.LOW)          
      if(found==0):
            if flag==0:
rightturn()
time.sleep(0.05)
            else:
leftturn()
time.sleep(0.05)
stop()
time.sleep(0.0125)

elif(found==1):
            if(area<initial):
                  if(distanceC<10):
                        if distanceR>=8:
rightturn()
time.sleep(0.00625)
stop()
time.sleep(0.0125)
forward()
time.sleep(0.00625)
stop()
time.sleep(0.0125)
                              #while found==0:
leftturn()
time.sleep(0.00625)
elifdistanceL>=8:
leftturn()
time.sleep(0.00625)
stop()
time.sleep(0.0125)
forward()
time.sleep(0.00625)
stop()
time.sleep(0.0125)
rightturn()
time.sleep(0.00625)
stop()
time.sleep(0.0125)
                        else:
stop()
time.sleep(0.01)
                  else:
forward()
time.sleep(0.00625)
elif(area>=initial):
                  initial2=6700
                  if(area<initial2):
                        if(distanceC>10):
if(centre_x<=-20 or centre_x>=20):
                                    if(centre_x<0):
                                          flag=0
rightturn()
time.sleep(0.025)
elif(centre_x>0):
          flag=1
leftturn()
time.sleep(0.025)
forward()
time.sleep(0.00003125)
stop()
time.sleep(0.00625)
                        else:
stop()
time.sleep(0.01)
  else:
GPIO.output(LED_PIN,GPIO.HIGH)
time.sleep(0.1)
stop()
time.sleep(0.1)
rawCapture.truncate(0)  

if(cv2.waitKey(1) & 0xff == ord('q')):
            break

GPIO.cleanup() 
