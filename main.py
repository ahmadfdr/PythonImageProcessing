import numpy as np
import cv2

capture = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_COMPLEX
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)        ##Set camera resolution
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
Kernal = np.ones((3, 3), np.uint8)

#dist = 0
#focal = 450
#pixels = 30
#width = 4
#font = cv2.FONT_HERSHEY_SIMPLEX
#org = (0,20)
#color = (0, 0, 255)


#cnt = 0

low_H = [0, 0]
low_S = [0, 0]
low_V = [0, 0]

high_H = [255, 255]
high_S = [255, 255]
high_V = [255, 255]

low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

window_detection_name = 'Trackbar'

def on_low_H_thresh_trackbar(va1):
    global low_H
    global high_H
    low_H[0] = va1
    low_H[0] = min(high_H[0]-1, low_H[0])
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H[0])
def on_high_H_thresh_trackbar(va1):
    global low_H
    global high_H
    high_H[0] = va1
    high_H[0] = max(high_H[0], low_H[0]+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H[0])
def on_low_S_thresh_trackbar(va1):
    global low_S
    global high_S
    low_S[0] = va1
    low_S[0] = min(high_S[0]-1, low_S[0])
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S[0])
def on_high_S_thresh_trackbar(va1):
    global low_S
    global high_S
    high_S[0] = va1
    high_S[0] = max(high_S[0], low_S[0]+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S[0])
def on_low_V_thresh_trackbar(va1):
    global low_V
    global high_V
    low_V[0] = va1
    low_V[0] = min(high_V[0]-1, low_V[0])
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V[0])
def on_high_V_thresh_trackbar(va1):
    global low_V
    global high_V
    high_V[0] = va1
    high_V[0] = max(high_V[0], low_V[0]+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V[0])

cv2.namedWindow(window_detection_name)

def Trackbar():
    cv2.createTrackbar(low_H_name, window_detection_name, low_H[0], 255, on_low_H_thresh_trackbar)
    cv2.createTrackbar(high_H_name, window_detection_name, high_H[0], 255, on_high_H_thresh_trackbar)
    cv2.createTrackbar(low_S_name, window_detection_name, low_S[0], 255, on_low_S_thresh_trackbar)
    cv2.createTrackbar(high_S_name, window_detection_name, high_S[0], 255, on_high_S_thresh_trackbar)
    cv2.createTrackbar(low_V_name, window_detection_name, low_V[0], 255, on_low_V_thresh_trackbar)
    cv2.createTrackbar(high_V_name, window_detection_name, high_V[0], 255, on_high_V_thresh_trackbar)

while(True):
    ret, gambar = capture.read()
    HSV = cv2.cvtColor(gambar, cv2.COLOR_BGR2HSV)
    Trackbar()
    Dunialain = cv2.inRange(HSV, (low_H[0], low_S[0], low_V[0]), (high_H[0], high_S[0], high_V[0]))
    cnts = cv2.findContours(Dunialain.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    #for cnt in cnts:
    if len(cnts) > 0:

           # rect = cv2.minAreaRect(cnt)
           # box = cv2.boxPoints(rect)
           # box = np.int0(box)
           # cv2.drawContours(gambar,[box], -1,(255,0,0),3)

           # pixels = rect[1][0]
           # print(pixels)
           # dist = (width*focal)/pixels
           # gambar = cv2.putText(gambar, 'Distance :', org, font, 1, color, 2, cv2.LINE_AA)

        c = max(cnts, key = cv2.contourArea)
        cnt = cnts[0]
       # area = cv2.contourArea(cnt)

        distance = 2 * (10 ** (-7)) * (c ** 2) - (0.0067 * c) + 83.487
        M = cv2.moments(c)
       # x = int(M['m10'] / M['m00'])
      #  y = int(M['m01'] / M['m00'])
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        S = 'Distance Of Object: ' + str(distance)
        cv2.putText(gambar, S, (5, 50), font, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.drawContours(gambar, cnt, -1, (0, 255, 0), 3)
        if radius > 10:
            cv2.circle(gambar, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(gambar, (int(x), int(y)), 3, (0, 0, 255), -1)
            cv2.putText(gambar, 'centeroid', (int(x)+10, int(y)), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4,(0, 0, 255), 1)
    cv2.imshow('Camera', gambar)
    cv2.imshow('Threshold', Dunialain)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
capture.release()
cv2.destroyAllWindows()