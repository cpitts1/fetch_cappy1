# Cappy Pitts
# This program detects lines in images

import cv2
import numpy
import sys

# Code is adapted from the OpenCV docs on HoughLines and Canny

# Use this function to draw the lines-converts the output of hough lines from
# polar coordinates to line segments
def polarToSegment(rho, theta):
    a = numpy.cos(theta)
    b = numpy.sin(theta)
    tangent = numpy.array([-b,a])
    pt0 = rho*numpy.array([a,b])
    pt1 = tuple(map(int, pt0 - 1000*tangent))
    pt2 = tuple(map(int, pt0 + 1000*tangent))
    return pt1, pt2

# Read in image and make copy
if len(sys.argv) < 2:
    print 'usage:', sys.argv[0], \
            'filename'

    sys.exit(1)

filein = sys.argv[1]
src = cv2.imread(filein)
srcp = src.copy()

# Get height and width
h = srcp.shape[0]
w = srcp.shape[1]

# Make empty image
color_dst = numpy.empty((h,w), 'uint8')

# Use canny edge detector for line detection
lowThresh = 85
ratio = 2.8
kernel_size = 3
edges = cv2.Canny(srcp, lowThresh, lowThresh*ratio, kernel_size)

# Convert picture to gray scale
cv2.cvtColor(srcp, cv2.COLOR_RGB2GRAY, color_dst)

# Use cv2 HoughLines method to detect lines and draw them using the
# polarToSegment method created above

lines = cv2.HoughLines(edges, .9, 3.14/120, 100)
lines = lines[0]

for i in range(len(lines)):
    pt1, pt2 = polarToSegment(lines[i][0],lines[i][1])
    cv2.line(src, pt1, pt2,(255,0,0))

# Display picture with lines over it
cv2.imshow('lines', src)
cv2.waitKey()


