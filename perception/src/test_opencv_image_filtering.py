#!/usr/bin/env python
'''
OpenCV image enhancements results
By Alexandre Martins
Axis Communications - Lund LTH - WASP

List of used sources:
- Equalize: http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_histograms/py_histogram_equalization/py_histogram_equalization.html
- Median blurring: https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
- Opening and dilating image: http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
- Thresholding: http://docs.opencv.org/trunk/d7/d4d/tutorial_py_thresholding.html

List of unused but seem interesting sources:
- Blob detection: https://www.learnopencv.com/blob-detection-using-opencv-python-c/
- Haar classifier: http://coding-robin.de/2013/07/22/train-your-own-opencv-haar-classifier.html
'''
import cv2
import numpy as np

###
# HIDE: Move around to hide displays
#
show = False #HIDE !
###

#Original image
img = cv2.imread('opencv_enhance_test.jpg')
b, g, r = cv2.split(img)
if show: cv2.imshow('Original', img)


#
# Transformations
#

# Equalize (light correction)
# http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_histograms/py_histogram_equalization/py_histogram_equalization.html
clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8,8))
cr = clahe.apply(r)
cg = clahe.apply(g)
cb = clahe.apply(b)
c_img = cv2.merge((cb, cg, cr))
if show: cv2.imshow('CLAHE equalized', c_img)

# Median blur (noise reduction)
# https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
b_img = cv2.medianBlur(img, 3)
bc_img = cv2.medianBlur(c_img, 3)
if show: cv2.imshow('Median blurred img', b_img)
if show: cv2.imshow('Median blurred equalized img', bc_img)

# Open image (noise reduction)
# http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
kernel = np.ones((3,3),np.uint8)
ob_img = cv2.morphologyEx(b_img, cv2.MORPH_OPEN, kernel)
obc_img = cv2.morphologyEx(bc_img, cv2.MORPH_OPEN, kernel)
if show: cv2.imshow('Opened median blurred img', ob_img)
if show: cv2.imshow('Opened median blurred equalized img', obc_img)

# Dilate image (joint object parts)
# http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
kernel = np.ones((5,5),np.uint8)
dob_img = cv2.dilate(ob_img, kernel, iterations=1)
dobc_img = cv2.dilate(obc_img, kernel, iterations=1)
if show: cv2.imshow('Dilated opened median blurred img', dob_img)
if show: cv2.imshow('Dilated opened median blurred equalized img', obc_img)

'''
# Threshold
# http://docs.opencv.org/trunk/d7/d4d/tutorial_py_thresholding.html
dob_b, dob_g, dob_r = cv2.split(dob_img)
dobc_b, dobc_g, dobc_r = cv2.split(dobc_img)

tdob_r = cv2.adaptiveThreshold(dob_r, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 3, 2)
tdobc_r = cv2.adaptiveThreshold(dobc_r, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 3, 2)
tdob_img = cv2.merge((b, g, tdob_r))
tdobc_img = cv2.merge((cb, cg, tdobc_r))
if show: cv2.imshow('Thresholded original', tdob_img)
if show: cv2.imshow('Thresholded CLAHE equalized', tdobc_img)

#Show only red data
if show: cv2.imshow('Red', r)
if show: cv2.imshow('Equalized red', cr)
if show: cv2.imshow('Thresholded red', tdob_r)
if show: cv2.imshow('Thresholded equalized red', tdobc_r)
'''

###
# SHOW: Move around to show displays
#
show = True #SHOW !
###

#
# Convert to hsv
#
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
c_img_hsv = cv2.cvtColor(c_img, cv2.COLOR_BGR2HSV)
b_img_hsv = cv2.cvtColor(b_img, cv2.COLOR_BGR2HSV)
bc_img_hsv = cv2.cvtColor(bc_img, cv2.COLOR_BGR2HSV)
ob_img_hsv = cv2.cvtColor(ob_img, cv2.COLOR_BGR2HSV)
obc_img_hsv = cv2.cvtColor(obc_img, cv2.COLOR_BGR2HSV)
dob_img_hsv = cv2.cvtColor(dob_img, cv2.COLOR_BGR2HSV)
dobc_img_hsv = cv2.cvtColor(dobc_img, cv2.COLOR_BGR2HSV)

#Threshold
lower_red_upper = np.array([0, 120, 65])
upper_red_upper = np.array([7, 220,220])
lower_red_lower = np.array([140, 90, 80])
upper_red_lower = np.array([190, 240,230])

#
# Apply thresholds
#
#Original
redMask_upper = cv2.inRange(img_hsv, lower_red_upper, upper_red_upper)
redMask_lower = cv2.inRange(img_hsv, lower_red_lower, upper_red_lower)
redMask = cv2.bitwise_or(redMask_upper, redMask_lower)
if show: cv2.imshow('Original filtered', redMask)

#Equalized
c_redMask_upper = cv2.inRange(c_img_hsv, lower_red_upper, upper_red_upper)
c_redMask_lower = cv2.inRange(c_img_hsv, lower_red_lower, upper_red_lower)
c_redMask = cv2.bitwise_or(c_redMask_upper, c_redMask_lower)
if show: cv2.imshow('Equalized filtered', c_redMask)

#Blurred original
b_redMask_upper = cv2.inRange(b_img_hsv, lower_red_upper, upper_red_upper)
b_redMask_lower = cv2.inRange(b_img_hsv, lower_red_lower, upper_red_lower)
b_redMask = cv2.bitwise_or(b_redMask_upper, b_redMask_lower)
if show: cv2.imshow('Original blurred filtered', b_redMask)

#Blurred equalized
bc_redMask_upper = cv2.inRange(bc_img_hsv, lower_red_upper, upper_red_upper)
bc_redMask_lower = cv2.inRange(bc_img_hsv, lower_red_lower, upper_red_lower)
bc_redMask = cv2.bitwise_or(bc_redMask_upper, bc_redMask_lower)
if show: cv2.imshow('Equalized blurred filtered', bc_redMask)

#Opened & dilated original
dob_redMask_upper = cv2.inRange(dob_img_hsv, lower_red_upper, upper_red_upper)
dob_redMask_lower = cv2.inRange(dob_img_hsv, lower_red_lower, upper_red_lower)
dob_redMask = cv2.bitwise_or(dob_redMask_upper, dob_redMask_lower)
if show: cv2.imshow('Original dilated opened blurred filtered', dob_redMask)

#Open 6 dilated equalized
dobc_redMask_upper = cv2.inRange(dobc_img_hsv, lower_red_upper, upper_red_upper)
dobc_redMask_lower = cv2.inRange(dobc_img_hsv, lower_red_lower, upper_red_lower)
dobc_redMask = cv2.bitwise_or(dobc_redMask_upper, dobc_redMask_lower)
if show: cv2.imshow('Equalized dilated opened blurred filtered', dobc_redMask)

cv2.waitKey(0)
cv2.destroyAllWindows()

