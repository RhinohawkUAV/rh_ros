#!/usr/bin/env python

import cv2
import numpy as np


# Display window
image_window = 'Image'
cv2.namedWindow(image_window, cv2.WINDOW_NORMAL)

# Color Mask window
mask_window = 'Color Mask'
cv2.namedWindow(mask_window, cv2.WINDOW_NORMAL)

#Picker
control_window = "Picker"
window = cv2.namedWindow(control_window)


##################
# Color filtering
#################

# Low cut off

hue_low = 0
def set_hue_low(new_value):
    global hue_low
    hue_low = new_value
cv2.createTrackbar('Hue_Low', control_window, 0, 255, set_hue_low)

saturation_low = 0
def set_saturation_low(new_value):
    global saturation_low
    saturation_low = new_value
cv2.createTrackbar('Saturation_Low', control_window, 0, 255, set_saturation_low)

value_low = 0
def set_value_low(new_value):
    global value_low
    value_low = new_value
cv2.createTrackbar('Value_Low', control_window, 0, 255, set_value_low)


# High cut off

hue_high = 0
def set_hue_high(new_value):
    global hue_high
    hue_high = new_value
cv2.createTrackbar('Hue_High', control_window, 0, 255, set_hue_high)

saturation_high = 0
def set_saturation_high(new_value):
    global saturation_high
    saturation_high = new_value
cv2.createTrackbar('Saturation_High', control_window, 0, 255, set_saturation_high)

value_high = 0
def set_value_high(new_value):
    global value_high
    value_high = new_value
cv2.createTrackbar('Value_High', control_window, 0, 255, set_value_high)


################
# Blob detector
###############

# params = cv2.SimpleBlobDetector_Params()
# params.maxThreshold = 255
# params.minThreshold = 0
# params.filterByArea = True
# params.filterByCircularity = True

# def set_min_area(new_value):
#     params.minArea = new_value
# cv2.createTrackbar('Min_Area', control_window, 0, 2000, set_min_area)

# def set_max_area(new_value):
#     params.maxArea = new_value
# cv2.createTrackbar('Max_Area', control_window, 0, 10000, set_max_area)

# def set_circularity(new_value):
#     params.minCircularity = new_value
# cv2.createTrackbar('Circularity', control_window, 0, 1, set_circularity)


# Run video
capture = cv2.VideoCapture(0);
if not capture.isOpened():
    print "Cannot open video 0"
    quit();

detector = cv2.SimpleBlobDetector()

while True:
    success, image = capture.read()
    if not success:
        print "Cannot capture"
        break

    # Build mask for filter by color
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (hue_low, saturation_low, value_low), (hue_high, saturation_high, value_high))

    contours, hierarchy = cv2.findContours(np.copy(mask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 3)

    cv2.imshow(image_window, image)
    cv2.imshow(mask_window, mask)
    cv2.waitKey(1)
