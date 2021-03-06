# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import imutils
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

Framefirst = None
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	image = imutils.resize(image, width=500)

        #Converting to grayscale
	gray =cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	
        

	#Thresholding
        #fmd = cv2.absdiff(Framefirst, gray)
	th1 = cv2.threshold(gray, 165, 255, cv2.THRESH_BINARY)[1]
	thresh = cv2.dilate(th1, None, iterations=2)
	thresh = cv2.erode(thresh, None, iterations=2)


	#_,cnts, _ = cv2.findContours(th1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = not np.any(thresh)

        if result is True:
                print "No object detected."
        else:
                print "Object detected."

	# show the frame
	cv2.imshow("Frame0", image)
	cv2.imshow("Frame", th1)
	cv2.imshow("Frame1", thresh)

	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
