#!/usr/bin/env python
# -*- coding: utf-8 -*-
def pantograf(frame,counter):
    
    import cv2
    import numpy as np
      
    image = frame
    
    #Detection of red zones in input image  
    bgr = [100, 80, 215]
    thresh = 40
    
    minBGR = np.array([bgr[0] - thresh-20, bgr[1] - thresh-30, bgr[2] - thresh-20])
    maxBGR = np.array([bgr[0] + thresh+40, bgr[1] + thresh, bgr[2] + thresh])
    
    maskBGR = cv2.inRange(image,minBGR,maxBGR)
    resultBGR = cv2.bitwise_and(image, image, mask = maskBGR)
    
    #Morphological Operations
    # Grayscale
    gray = cv2.cvtColor(resultBGR, cv2.COLOR_BGR2GRAY)
    ret,th = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    th = th[125:325, 100:500]
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15))
    dilated = cv2.dilate(th,kernel,iterations = 1)
    
    dilated = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15)))
    eroded = cv2.erode(dilated,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15)),iterations=1)

    #Finding of contours
    contours, hierarchy = cv2.findContours(eroded, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    colored = cv2.drawContours(frame[125:325, 100:500],contours,-1,(0,255,255),3)
    cv2.imshow("asd",colored)
    
    #Labeling of biggest contour and classification of the situation
    #Also sending the command to STM32 is happenning here
    areas=[cv2.contourArea(i) for i in contours]
    area=max(areas)
    global arealist
    arealist.append(area)
    global lenval,lenval2,flag,defected,ser
#     print(area,counter)
    if area>500:
        if lenval==0:
            print("starting point:",counter)
            lenval=counter
            lenval2=0
            flag=1
    elif area<300:
        if lenval2==0 and arealist[counter-1]>=200 and flag==1:
            lenval2=counter
            print(" detected length:",lenval2-lenval)
            if lenval2-lenval<50:
                print(" HATALI PARÇA","\nend of defect:",counter,"\n")
                lenval2=0
                defected=1
            else:
                print("ending point:",counter,"\n\n*************")
                lenval=0
                flag=0
                if defected==1:
                    ser.write(b'1')
                    print("command sent(f) ")
                elif defected==0:
                    ser.write(b'0')
                    print("command sent(o) ")
                defected=0

import cv2
import serial
ser = serial.Serial("/dev/ttyS0", baudrate = 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
arealist=[]
num=0
lenval=0
lenval2=0
flag=0
defected=0
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

# Loop untill the end of the video
while (cap.isOpened()):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        break   
    
    pantograf(frame,num)
    num=num+1
#     cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
cap.release()
cv2.destroyAllWindows()

# from matplotlib import pyplot as plt
# plt.figure(0), plt.plot(arealist), plt.show()