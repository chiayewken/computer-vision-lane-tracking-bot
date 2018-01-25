import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

upperbody_cascade = cv2.CascadeClassifier('haarcascade_upperbody.xml')
arduino = serial.Serial("/dev/ttyACM1", 9600, timeout=.1)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    height, width, channels = frame.shape # laptop cam is 480x640
    # plot bisecting lines horizontally and vertically
    cv2.line(frame,(0,height/2),(width,height/2),(255,0,0),2)
    cv2.line(frame,(width/2,0),(width/2,height),(255,0,0),2)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    min = int(height * 0.2)
    uppers = upperbody_cascade.detectMultiScale(gray, 1.1, 1, minSize=(min,min))
    count = 0
    for (x,y,w,h) in uppers:
        count += 1
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
        # plot centre of detection square
        a = x+(w/2)
        b = y+(h/2)
        cv2.rectangle(frame,(a-5,b-5),(a+5,b+5),(255,0,255),5)

        if count == 1:
            if a > width / 2: print("turn left")
            else: print("turn right")
            if b > height / 2: print("forward")
            else: print("backward")

    cv2.imshow("frame2", frame)
    #print("%s detected" % count)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
