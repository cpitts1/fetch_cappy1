# Cappy Pitts
# This program detects circles in images

import cv2
import numpy
import sys

# Code is adapted from the OpenCV docs on HoughCircles

# Read in image and make copy
# If the number of arguments is incorrect specify how the user should give the
# correct arguments

if len(sys.argv) < 4:
    print 'usage:', sys.argv[0], \
            'filename dp minDist'

    sys.exit(1)

filein = sys.argv[1]
src = cv2.imread(filein)
srcp = src.copy()

# Get height and width
h = srcp.shape[0]
w = srcp.shape[1]

# Create empty image
color_dst = numpy.empty((h,w), 'uint8')

# Convert picture to gray scale
cv2.cvtColor(srcp, cv2.COLOR_RGB2GRAY, color_dst)

# Use HoughCircles to find circles in the given picture
dp = float(sys.argv[2])
minDist = float(sys.argv[3])

circles = cv2.HoughCircles(color_dst, cv2.cv.CV_HOUGH_GRADIENT, dp, minDist)

# If no circles are detected then exit the program
if circles == None:
    print "No circles found using these parameters"
    sys.exit()

print circles
circles = numpy.uint16(numpy.around(circles))

for i in circles[0,:]:
    # draw the outer circle
    cv2.circle(srcp,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(srcp,(i[0],i[1]),2,(0,0,255),3)

# Display picture with circles over it
cv2.imshow('detected circles', srcp)
cv2.waitKey()


