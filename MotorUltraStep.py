#importing the necessary packages
import RPi.GPIO as GPIO
import numpy as np
import time
from time import sleep
#import cv2
#from picamera.array import PiRGBArray
#from picamera import PiCamera




#setting the pin mode
GPIO.setmode(GPIO.BCM)

GPIO.setwarnings(False)




#Assigning the pin numbers
#To motor driver 1 IN pins
M1A = 02
M1B = 03
M1C = 04
M1D = 14

#To motor driver 2 IN pins
M2A = 17
M2B = 27
M2C = 22
M2D = 23

#To motor driver 3 IN pins
M3A = 19
M3B = 16
M3C = 20
M3D = 21

#To motor driver enable pins
M1E = 05
M2E = 06
M3E = 13

#For the ultrasonic sensor
Trig = 10
Echo = 9

#For the Infrared sensor
IR = 26

#For the stepper motor
enable_pin1 = 18
enable_pin2 = 03
coil_A_1_pin = 04
coil_A_2_pin = 17
coil_B_1_pin = 23
coil_B_2_pin = 24




#Setting the pins as I/O pins
#Motor driver 1 IN pins
GPIO.setup(M1A, GPIO.OUT)
GPIO.setup(M1B, GPIO.OUT)
GPIO.setup(M1C, GPIO.OUT)
GPIO.setup(M1D, GPIO.OUT)

#Motor driver 2 IN pins
GPIO.setup(M2A, GPIO.OUT)
GPIO.setup(M2B, GPIO.OUT)
GPIO.setup(M2C, GPIO.OUT)
GPIO.setup(M2D, GPIO.OUT)

#Motor driver 3 IN pins
GPIO.setup(M3A, GPIO.OUT)
GPIO.setup(M3B, GPIO.OUT)
GPIO.setup(M3C, GPIO.OUT)
GPIO.setup(M3D, GPIO.OUT)

#Enable inputs of motor drivers
GPIO.setup(M1E, GPIO.OUT)
GPIO.setup(M2E, GPIO.OUT)
GPIO.setup(M3E, GPIO.OUT)

#Ultrasonic sensor pins
GPIO.setup(Trig, GPIO.OUT)
GPIO.setup(Echo, GPIO.IN)

#IR input
GPIO.setup(IR, GPIO.IN)

#Stepper motor inputs
GPIO.setup(enable_pin1, GPIO.OUT)
GPIO.setup(enable_pin2, GPIO.OUT)
GPIO.setup(coil_A_1_pin, GPIO.OUT)
GPIO.setup(coil_A_2_pin, GPIO.OUT)
GPIO.setup(coil_B_1_pin, GPIO.OUT)
GPIO.setup(coil_B_2_pin, GPIO.OUT)




#Stepper Motor Functions
def forward(delay, steps):  
  for i in range(0, steps):
    setStep(1, 0, 1, 0)
    time.sleep(delay)
    setStep(0, 1, 1, 0)
    time.sleep(delay)
    setStep(0, 1, 0, 1)
    time.sleep(delay)
    setStep(1, 0, 0, 1)
    time.sleep(delay)
 
def backwards(delay, steps):  
  for i in range(0, steps):
    setStep(1, 0, 0, 1)
    time.sleep(delay)
    setStep(0, 1, 0, 1)
    time.sleep(delay)
    setStep(0, 1, 1, 0)
    time.sleep(delay)
    setStep(1, 0, 1, 0)
    time.sleep(delay)
 
  
def setStep(w1, w2, w3, w4):
  GPIO.output(coil_A_1_pin, w1)
  GPIO.output(coil_A_2_pin, w2)
  GPIO.output(coil_B_1_pin, w3)
  GPIO.output(coil_B_2_pin, w4)




#Measuring distance using ultrasonic sensor
while True:
    GPIO.output(Trig, False)                        #Sending a LOW pulse to the trigger pin
    print "Waiting for the sensor to settle"
    time.sleep(0.1)                                 #Delay of 0.5 second

    GPIO.output(Trig, True)                         #Sending a HIGH pulse to the trigger pin
    time.sleep(0.00001)                             #Delay of 1 micro sec
    GPIO.output(Trig, False)                        #Sending a LOW pulse to the trigger pin

    while GPIO.input(Echo) == 0:                    #Checks whether the Echo pin is LOW
        pulse_start = time.time()                   #Saves the last known time of LOW pulse        

    while GPIO.input(Echo) == 1:                    #Checks whether the Echo pin is HIGH
        pulse_end = time.time()                     #Saves the last known time of HIGH pulse

    pulse_duration = pulse_start - pulse_end        #pulse duration

    distance = (pulse_duration * 34300)/2           #Multiply pulse duration by 34300 (speed of sound) and divide by 2 to get the distance in cm
    distance = round(distance, 2)

    InRed = GPIO.input(IR)                          #Check the IR input pin



    #Controlling of the motors using the sensors
    if distance > 15 and InRed == 0:
        print "No obstacle detected."
        print "Distance:", distance - 0.5,"cm"      #print the distance with a 0.5cm calibration

        GPIO.output(M1E, GPIO.HIGH)
        GPIO.output(M2E, GPIO.HIGH)
        GPIO.output(M3E, GPIO.LOW)

        GPIO.output(M1A, GPIO.HIGH)
        GPIO.output(M1B, GPIO.LOW)
        GPIO.output(M1E, GPIO.HIGH)
        GPIO.output(M1E, GPIO.LOW)

        GPIO.output(M1E, GPIO.HIGH)
        GPIO.output(M1E, GPIO.LOW)
        GPIO.output(M1E, GPIO.HIGH)
        GPIO.output(M1E, GPIO.LOW)

    else:
        print "Obstacle detected."

        GPIO.output(enable_pin1, GPIO.HIGH)
        GPIO.output(enable_pin2, GPIO.HIGH)

        GPIO.output(M1E, GPIO.LOW)
        GPIO.output(M2E, GPIO.LOW)
        GPIO.output(M3E, GPIO.HIGH)

        GPIO.output(M3A, GPIO.HIGH)
        GPIO.output(M3B, GPIO.LOW)
        GPIO.output(M3C, GPIO.HIGH)
        GPIO.output(M3D, GPIO.LOW)

        delay = 500
        steps = 13
        forward(int(delay) / 1000.0, int(steps))
        steps = 26
        backwards(int(delay) / 1000.0, int(steps))
        steps = 13
        forward(int(delay) / 1000.0, int(steps))

    sleep(0.5) 
