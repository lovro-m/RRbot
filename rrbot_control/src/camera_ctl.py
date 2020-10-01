#!/usr/bin/env python


# Python libs
import sys, time

# numpy and scipy
import numpy as np
import math
#from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Float64, Bool

# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=True

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        #self.image_pub = rospy.Publisher("/output/image_raw/compressed",
        #    CompressedImage)
        # self.bridge = CvBridge()

        #Initialize values
        self.camera_command = Bool()
        self.camera_command.data = False

        # Subscriber Initialization
        self.subscriber = rospy.Subscriber("/rrbot/camera1/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 1)    #CompressedImage is type of msg, like Float64
        if VERBOSE :
            print("subscribed to /rrbot/camera1/image_raw/compressed")
        rospy.Subscriber('/rrbot/camera1/command', Bool, self.camera_command_cb)

        # Publisher Initialization
        self.pub_centre = rospy.Publisher('/rrbot/camera1/centre', Point, queue_size=1)
        
        self.pub_extracted_destination = rospy.Publisher('/rrbot/camera1/extracted_destination', Point, queue_size=1)
        self.pub_feedback = rospy.Publisher('/rrbot/camera1/feedback', Bool, queue_size=1)


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.frombuffer(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        

        original = image_np
        image = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
        lower = np.array([22, 200, 200], dtype="uint8")     #lower bound for detection
        upper = np.array([45, 255, 255], dtype="uint8")     #upper bound for detection
        mask = cv2.inRange(image, lower, upper)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        #bounds of the sorted zone
        bound_x = 300
        bound_y = 600
        
        #bounds of contour dimensions
        bound_w = 50
        bound_h = 50

        #border case if centre not found
        cX = None
        cY = None

        for c in cnts:

            x,y,w,h = cv2.boundingRect(c)

            #find only in unsorted zone
            if x >= bound_x or y >= bound_y:
                #find only big contours
                if w >= bound_w and h >= bound_h:
                    cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)
                    #compute centre of contour

                    cX = (x + (x + w))/2.0
                    cY = (y + (y + h))/2.0
                    
                    """ this sometimes spits out float division by zero
                    M = cv2.moments(c)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    """
                    cv2.circle(original, (math.floor(cX), math.floor(cY)), 7, (255, 255, 255), -1)

        cv2.rectangle(original, (150, 150), (bound_x, bound_y), (144,238,144), 2)     #sorted zone
        cv2.circle(original, (400, 400), 7, (255, 255, 0), -1)      #centre of image


        cv2.imshow('mask', mask)
        cv2.imshow('original', original)
        cv2.waitKey(2)

        #check if centre was found
        if cX is not None and cY is not None:

            #Check if camera is needed, given from task_ctl.py
            if self.camera_command.data:
                # Publish centre of cylinder
                centre = Point()
                centre.x = cX
                centre.y = cY    
                centre.z = 0                     # z value is arbitrary

                self.pub_centre.publish(centre)     #publish for debugging

                #testing
                #self.centre = Point()
                #self.centre.x = 572
                #self.centre.y = 400   
                #self.centre.z = 0  

                #calculate end effector positioning
                image_centre = Point()
                image_centre.x = 400
                image_centre.y = 400   
                image_centre.z = 0                     # z value is arbitrary
                
                # 172 pixels is 1 meter give or take
                x = (centre.x - image_centre.x) / 172.0
                y = ((centre.y - image_centre.y) / 172.0) * (-1) #image Y value goes from top of image to bottom
                                                                    #robot Y coordinate goes from bottom to top
                z = 0           # z value is arbitrary

                ros_point = Point()
                ros_point.x = x
                ros_point.y = y
                ros_point.z = z

                self.pub_extracted_destination.publish(ros_point) 

                #Send feedback
                camera_feedback_msg = Bool()
                camera_feedback_msg.data = True
                
                self.pub_feedback.publish(camera_feedback_msg)
            else:
                print("Camera not needed yet.")
        else:

            #Send feedback
            camera_feedback_msg = Bool()
            camera_feedback_msg.data = False
            
            self.pub_feedback.publish(camera_feedback_msg) 

            print("No centre found.")
            

        """
        #if the colour of the cylinder changes or we need circle detection we use this
        gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                               param1=100, param2=30,
                               minRadius=1, maxRadius=100)
        output = image_np.copy()

        # ensure at least some circles were found
        if circles is not None:
            print("circle found")
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            # show the output image
        #cv2.imshow("output", np.hstack([image_np, output]))
        #cv2.waitKey(2)

        #cv2.imshow("gray", gray)
        #cv2.waitKey(2)

        #cv2.imshow('cv_img', image_np)
        #cv2.waitKey(2)
        """

        
        """
        # if we need to publish the compressed image we use this
        #### Create CompressedIamge ####
        #msg = CompressedImage()
        #msg.header.stamp = rospy.Time.now()
        #msg.format = "jpeg"
        #msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        #self.image_pub.publish(msg)
        
        #self.subscriber.unregister()
        """

    def camera_command_cb(self, msg):
        self.camera_command = msg

def main(args):
    '''Initializes and cleanup ros node'''
    
    rospy.init_node('image_feature')
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)