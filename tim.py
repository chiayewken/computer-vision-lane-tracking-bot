import numpy as np 
import cv2
import serial
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import argparse
import imutils
from threading import *
from rplidar import RPLidar

class PiVideoStream:
	def __init__(self, resolution=(192, 112), framerate=32):
		# initialize the camera and stream
		self.camera = PiCamera()
		self.camera.resolution = resolution
		self.camera.rotation = 180
		self.camera.framerate = framerate
		self.rawCapture = PiRGBArray(self.camera, size=resolution)
		self.stream = self.camera.capture_continuous(self.rawCapture,format="bgr", use_video_port=True)
		# initialize the frame and the variable used to indicate
		# if the thread should be stopped
		self.frame = None
		self.stopped = False

	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		for f in self.stream:
			# grab the frame from the stream and clear the stream in
			# preparation for the next frame
			self.frame = f.array
			self.rawCapture.truncate(0)

			# if the thread indicator variable is set, stop the thread
			# and resource camera resources
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.camera.close()
				return

	def read(self):
		return self.frame

	def stop(self):
		self.stopped = True

cv2.setUseOptimized(True)
resheight = 0
vs = PiVideoStream().start()
arduino = serial.Serial("/dev/ttyACM1", 9600, timeout=.1)
time.sleep(0.5)

def plotline(rho,theta,img,B,G,R):
	a = np.cos(theta)
	b = np.sin(theta)
	x0 = a*rho
	y0 = b*rho
	x1 = int(x0 + 1000*(-b))
	y1 = int(y0 + 1000*(a))
	x2 = int(x0 - 1000*(-b))
	y2 = int(y0 - 1000*(a))
	cv2.line(img,(x1,y1),(x2,y2),(B,G,R),2)

def process(frame,left,right):
	# convert to grayscale and detect edges
	height,width,channels = frame.shape # get frame dimensions after downscale
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	edges = cv2.Canny(gray,100,150,apertureSize = 3)
	cv2.imshow("edges", edges)

	# detect lines
	threshold = 40 # hough line transform tolerance, less is more tolerant
	lines = cv2.HoughLines(edges,1,np.pi/180,threshold)
	try:
		for line in lines:
			for rho,theta in line:
				plotline(rho,theta,frame,0,0,255)
	except:
		print("NOLINESSSSS")
        
        if (get_front_distance() > 510):
                arduino.write("<" + str(left) + "," + str(right) + ">")
        else:
                arduino.write("<0,0>")

	cv2.imshow("frame", frame)

left = 25
right = 25

while True:
	start = time.time()
	frame = vs.read()
	process(frame,left,right)
	end = time.time()
	fps = 1/(end-start)
	print(str(fps))

	if cv2.waitKey(1) & 0xFF == ord("q"):
		break

vs.stop()
cap.release()
cv2.destroyAllWindows()
