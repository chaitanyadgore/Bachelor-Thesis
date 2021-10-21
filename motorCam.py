#importing required libraries
import numpy as np
import RPi.GPIO as GPIO
import time
from time import sleep
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera


#Setting the pin numbering mode
GPIO.setmode(GPIO.BCM)

GPIO.setwarnings(False)


#Assigning pins
#For the motor drivers
M1A = 02
M1B = 03
M1C = 04
M1D = 14
M1E = 05
M2E = 06
M2A = 17
M2B = 27
M2C = 22
M2D = 23
M3E = 13
M4E = 19

#For the ultrasonic sensors
Trig = 18
Echo = 24

#For the infrared sensor
IR = 21



#setting the pins as I/O pins
#Motor driver 1 IN inputs
GPIO.setup(M1A,GPIO.OUT)
GPIO.setup(M1B,GPIO.OUT)
GPIO.setup(M1C,GPIO.OUT)
GPIO.setup(M1D,GPIO.OUT)

#Motor driver 2 IN inputs
GPIO.setup(M2A,GPIO.OUT)
GPIO.setup(M2B,GPIO.OUT)
GPIO.setup(M2C,GPIO.OUT)
GPIO.setup(M2D,GPIO.OUT)

#Motor driver enable inputs
GPIO.setup(M1E,GPIO.OUT)
GPIO.setup(M2E,GPIO.OUT)
GPIO.setup(M3E,GPIO.OUT)
GPIO.setup(M4E,GPIO.OUT)

#Ultasonic sensor 
GPIO.setup(Trig,GPIO.OUT)
GPIO.setup(Echo,GPIO.IN)

#IR input
GPIO.setup(IR,GPIO.IN)




#CAMERA
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
 
    # show the frame
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,200)
    ret, th1 = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    cv2.imshow("Original", image)
    cv2.imshow("Edges", edges)
    cv2.imshow("Threshold", th1)

    key = cv2.waitKey(1)
 
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
 
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break




#ULTRASONIC SENSOR
def distance():
    # set Trigger to HIGH
    GPIO.output(TRIG, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
 
    if __name__ == '__main__':
        try:
            while True:
                dist = distance()
                print ("Measured Distance = %.1f cm" % dist)
                time.sleep(1)
 
            # Reset by pressing CTRL + C
            except KeyboardInterrupt:
            print("Measurement stopped by User")
            

    while True:
        IR = GPIO.input(39)
        if IR == 0:
            print "Obstacle detected"
            time.sleep(1)
            break

    #RUNNING THE MOTORS
    if distance > 15 and IR == 1:
        print "No obstacle detected"
        #Enabling the motor drivers
        GPIO.output(M1E,HIGH)
        GPIO.output(M2E,HIGH)
        GPIO.output(M3E,HIGH)
        GPIO.output(M4E,HIGH)

        #Input to motor driver 1 IN pins
        GPIO.output(M1A,HIGH)
        GPIO.output(M1B,LOW)
        GPIO.output(M1C,HIGH)
        GPIO.output(M1D,LOW)

        #Input to motor driver 2 IN pins
        GPIO.output(M2A,HIGH)
        GPIO.output(M2B,LOW)
        GPIO.output(M2C,HIGH)
        GPIO.output(M2D,LOW)

    else:
        print "Obstacle detected. Stopping the motors"
        #Disabling the motor drivers
        GPIO.output(M1E,LOW)
        GPIO.output(M2E,LOW)
        GPIO.output(M3E,LOW)
        GPIO.output(M4E,LOW)
 
GPIO.cleanup()          



