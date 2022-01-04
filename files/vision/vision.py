#!/usr/bin/python3

import cv2
import numpy as np

# connect to a sensor (0: webcam)
cap=cv2.VideoCapture(0)

# capture an image
ret, frame=cap.read()

# Select ROI
r = cv2.selectROI(frame)

# Crop image
imCrop = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]

# Display cropped image
cv2.imshow("Image", imCrop)
cv2.waitKey(0)
