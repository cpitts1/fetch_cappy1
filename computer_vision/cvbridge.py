#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel, StereoCameraModel
from tf import TransformListener

class image_converter:

  def __init__(self):
    
    # Publish an image topic and show view of image
    self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
    cv2.namedWindow("Image window", 1)
    
    # Use CvBridge to convert from OpenCv to ros
    self.bridge = CvBridge()

    # Set up data member to get camera info
    self.cam_model = PinholeCameraModel()

    # Subscribe to the head_camera/rgb/image_raw camera topic
    self.image_sub = rospy.Subscriber("head_camera/rgb/image_raw", Image, self.get_coordinates)

    # Subscribe to the head_camera/depth_registered/camera_info
    #self.image_sub = rospy.Subscriber("head_camera/depth_registered/image_raw", Image, self.callback)

    # Set up transform listener
    self.tf_listener = TransformListener()
  
    self.cam_info_pub = rospy.Publisher("image_topic_3", CameraInfo, queue_size=10)
    self.cam_info_sub = rospy.Subscriber("/head_camera/depth_registered/camera_info", CameraInfo, self.set_camera_info)

  def get_coordinates(self, data):
    
    # not sure whether data.data is a python string or not?
    #cv_image = numpy.fromstring(data.data, dtype='f32').reshape((480,640)) 

    try:
      # Show edited image
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    except CvBridgeError, e:
      print e

    # Get height and width of image
    h = cv_image.shape[0]
    w = cv_image.shape[1]

    # Create empty image
    color_dst = numpy.empty((h,w), 'uint8')

    # Convert picture to grayscale
    cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY, color_dst)

    # Find circles given the camera image
    dp = 2
    minDist = 70

    circles = cv2.HoughCircles(color_dst, cv2.cv.CV_HOUGH_GRADIENT, dp, minDist)

    # If no circles are detected then exit the program
    if circles == None:
        print "No circles found using these parameters"
        sys.exit()
    
    circles = numpy.uint16(numpy.around(circles))
    
    # Draw the center of the circle closest to the top
    ycoord = []
    for i in circles[0,:]:
	ycoord.append(i[1])

    top_circ_y_coor = min(ycoord)
    x_coor = 0    
    y_coor = 0

    for i in circles[0,:]:
	if i[1] == top_circ_y_coor:
            # draw the center of the circle
            cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)
            # draw outer circle
            cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
	    x_coor = i[0]
            y_coor = i[1]

    self.center = (x_coor, y_coor)

    # Show edited image
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      # Show edited image
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e

  # Give self.cam_model information about Gretchen's camera
  def set_camera_info(self, data):
    self.cam_model.fromCameraInfo(data)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
