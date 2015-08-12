#!/usr/bin/env python
import roslib
import rospy
import sys
import cv2
import numpy
import math

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from tf import TransformListener

################################################################################

# This class subscribes to a topic one time while all of the other topics are 
# publishing and subscribing an infinite number of times
class get_single_helper:

    def __init__(self, topic, msg_type, timeout=rospy.Duration(30)):
        self.topic = topic
        self.sub = rospy.Subscriber(topic, msg_type, self.callback)
        self.data = None
        self.timeout = timeout
        
    def callback(self, data):
        self.data = data

    def run(self):

        start = rospy.Time.now()
        rospy.loginfo('waiting for ' + self.topic + '...')

        while (rospy.Time.now() - start) < self.timeout:
            if self.data is not None:
                rospy.loginfo('got it!')
                break
            rospy.sleep(rospy.Duration(0.5))

        if self.data is None:
            msg = 'did not get data in time for topic ' + self.topic
            rospy.logerror(msg)
            raise RuntimeError(msg)
            
        return self.data

def get_single(topic, msg_type, timeout=rospy.Duration(30)):
    
    h = get_single_helper(topic, msg_type, timeout)
    return h.run()

################################################################################

class ros_cv_testing_node:

    def __init__(self):
        
	# Get information for this particular camera
        camera_info = get_single('head_camera/depth_registered/camera_info', CameraInfo)
        print camera_info
	
	# Set information for the camera 
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(camera_info)
	
	# Subscribe to the camera's image topic and depth image topic
        self.rgb_image_sub = rospy.Subscriber("head_camera/rgb/image_raw", 
                                              Image, self.on_rgb)
        self.depth_image_sub = rospy.Subscriber("head_camera/depth_registered/image_raw", 
                                                Image, self.on_depth)
	
	# Publish where the button is in the base link frame
        self.point_pub = rospy.Publisher('button_point', PointStamped, queue_size=10)
	
	# Set up connection to CvBridge
        self.bridge = CvBridge()
        
	# Open up viewing windows
	cv2.namedWindow('depth')
        cv2.namedWindow('rgb')
        #cv2.namedWindow('combined')
	
	# Set up the class variables
        self.rgb_image = None
        self.rgb_image_time = None
        self.depth_image = None
        self.center = None
        self.return_point = PointStamped()
        self.tf_listener = TransformListener() 
        
    def on_rgb(self, image):
	# Convert image to something that cv2 can work with
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        except e:
            print e
            return

        self.rgb_image_time = image.header.stamp

        # Get height and width of image
        h = self.rgb_image.shape[0]
        w = self.rgb_image.shape[1]

        # Create empty image
        color_dst = numpy.empty((h,w), 'uint8')

        # Convert picture to grayscale
        cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2GRAY, color_dst)

        # Find circles given the camera image
        dp = 1.2
        minDist = 50

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

	# Find the top circle in the frame
        top_circ_y_coor = min(ycoord)
        x_coor = 0    
        y_coor = 0

        for i in circles[0,:]:
            if i[1] == top_circ_y_coor:
                # draw the center of the circle
                cv2.circle(self.rgb_image,(i[0],i[1]),2,(0,0,255),3)
                # draw outer circle
                cv2.circle(self.rgb_image,(i[0],i[1]),i[2],(0,255,0),2)
                x_coor = i[0]
                y_coor = i[1]

	# Set the coordinates for the center of the circle
        self.center = (x_coor, y_coor)

        cv2.imshow('rgb', self.rgb_image)
        cv2.waitKey(1)
    
    def on_depth(self, image):
        
        # Use numpy so that cv2 can use image
        self.depth_image = numpy.fromstring(image.data, dtype='float32').reshape((480,640))
        nmask = numpy.isnan(self.depth_image)

	#Find the minimum and the maximum of the depth image
        Dmin = self.depth_image[~nmask].min()
        Dmax = self.depth_image[~nmask].max()

        # If a circle has been found find its depth and apply that to the ray
        if self.center!=None:
            ray = self.cam_model.projectPixelTo3dRay(self.center)
            depth = self.depth_image[self.center[1]][self.center[0]]
            # If the depth is not a number exit
            if math.isnan(depth):
                print "Incomplete information on depth at this point"
                return
            # Otherwise mulitply the depth by the unit vector of the ray
            else:
                print "depth", depth
                cam_coor = [depth*ray[0],depth*ray[1],depth*ray[2]]
                #print "camera frame coordinate:", cam_coor  
        else:
            return

        # Rescale to [0,1]
        cv2.imshow('depth', self.depth_image / 2.0)
        cv2.waitKey(1)

	# Only use depth if the RGB image is running
        if self.rgb_image_time is None:
            rospy.loginfo('not using depth cause no RGB yet')
            return
            
        time_since_rgb = image.header.stamp - self.rgb_image_time

	# Only use depth if it is close in time to the RGB image
        if time_since_rgb > rospy.Duration(0.5):
            rospy.loginfo('not using depth because time since RGB is ' + str(time_since_rgb))
            return

        # Find transform from camera frame to world frame
        self.tf_listener.waitForTransform(self.cam_model.tfFrame(), "base_link",
                                          image.header.stamp, rospy.Duration(1.0))
       
	# Set up the point to be published               
        self.return_point.header.stamp = image.header.stamp
        self.return_point.header.frame_id = self.cam_model.tfFrame()
        self.return_point.point.x = cam_coor[0]
        self.return_point.point.y = cam_coor[1]
        self.return_point.point.z = cam_coor[2]
        
	# Transform point to the baselink frame
        self.return_point = self.tf_listener.transformPoint("base_link", self.return_point)
        print "world frame coordinate:", self.return_point.point.x, \
            self.return_point.point.y,self.return_point.point.z, "\n"       

        # Can uncomment to combine the images
        '''
	D = numpy.minimum(self.depth_image, 2.0) / 2.0
        display = (self.rgb_image * D[:,:,None]).astype('uint8')
        cv2.imshow('combined', display)
        cv2.waitKey(1)
	'''

        # Publish point of button
	self.point_pub.publish(self.return_point)

    
if __name__ == "__main__":
    # Initialize node
    rospy.init_node('ros_cv_testing', anonymous=True)
    node = ros_cv_testing_node()
    rospy.spin()
