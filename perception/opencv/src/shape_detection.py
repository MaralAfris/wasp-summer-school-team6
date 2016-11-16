#!/usr/bin/env python
'''
This source will act as support to finish your project and does not follow best
coding practices.
'''
#Import Python Packages, ROS messages
from __future__ import print_function
from __future__ import division
import roslib
import sys
import rospy
import copy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
#import the custom message we created to store objects
#from wasp_custom_msgs.msg import object_loc
import tf
from math import hypot

import argparse

#marten:
#What to do:
#move the focal lenght etc in the initialization node, because it is different for the drone and TB.
#find out how to localize objects.
#reprint Medic Box. It is too hard to classify!
#reprint different kinds of persons and try to classify them. Should we have multiple colors? ...
#... The program is probably able to classify many different kinds of persons.
#Tune the thresholds for the Medic Box. 
#Probably send the pictures and the initial localisation to a GUI. The human can then accept..
#... the objects and determine how big the objects are (40 % of picture width), which is ...
#... used for localization.


#Define Constants



#This function finds the lengths of all the sides and estimates the longest.
def Longest_Length(approxcontour):
    #add the first element in the end to complete the loop
    approxcontour = np.concatenate((approxcontour,[approxcontour[0]]))
    #The below lines find the length between two adjacent points
    #and append them in  an array
    ptdiff = lambda (p1,p2): (p1[0]-p2[0], p1[1]-p2[1])
    diffs = map(ptdiff, zip(approxcontour,approxcontour[1:]))
    dist = []
    for d in diffs:
        dist.append(hypot(*d))
    #find maximum of lenghts found
    LongestSide = max(dist)
    return LongestSide

#This is the main class for object detection, it has some initializations about nodes
#Call back functions etc
class object_detection:
    def __init__(self, args):
        #Create Rospy Publisher and subscriber
        #marten self.object_location_pub = rospy.Publisher('/object_location', object_loc, queue_size =1)
        #original images is huge and creates lot of latency, therefore subscribe to compressed image

        #Get image from turtlebot/drones...
        source = Image
        self.modeIsDrone = False
        if args.mode == 'drone': #Drone
            topic = '/bebop/image_raw/'
            self.modeIsDrone = True
            #Focal Length of the Asus Prime sensor camera
            self.focal_leng = 570.34222
            #This may change during the competetion, need to be calibrated
            self.square_side_lenth = 0.115 #in mts
        else: #Turtlebot
            topic = '/camera/rgb/image_raw/'
            #Focal Length of the Asus Prime sensor camera
            self.focal_leng = 570.34222
            #This may change during the competetion, need to be calibrated
            self.square_side_lenth = 0.115 #in mts

        if args.source is not None and args.source == 'compressed':
            topic += 'compressed/'
            source = CompressedImage

        print('Get image from of %s from %s' %(args.mode, topic))
        self.image_sub = rospy.Subscriber(topic, source, self.callback)
        #Cv Bridge is used to convert images from ROS messages to numpy array for openCV and vice versa
        self.bridge = CvBridge()
        #Obejct to transform listener which will be used to transform the points from one coordinate system to other.
        self.tl = tf.TransformListener()
        self.grayRedBoy = cv2.imread('MedBox.png',0)#MB funkade med forsta bilden
        self.grayGreenBoy = cv2.imread('BlackPerson.png',0)
        
        self.body_cascade = cv2.CascadeClassifier('haarcascade_fullbody.xml')
        self.jumpOver = 1
    #Callback function for subscribed image
    def callback(self,data):
      self.jumpOver=self.jumpOver+1
      self.jumpOver=self.jumpOver%20
      if self.jumpOver==1:  
        #The below two functions conver the compressed image to opencv Image
        #'''
        np_arr = np.fromstring(data.data, np.uint8)
        #The following is no longer named CV_LOAD_IMAGE_COLOR but CV_LOAD_COLOR. Works by defining it instead
        cv2.CV_LOAD_IMAGE_COLOR = 1
        #marten cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        img_for_presentation = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        img_original = cv2.copyMakeBorder(img_for_presentation,0,0,0,0,cv2.BORDER_REPLICATE)
        #'''
        #cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #Create copy of captured image
        #marten img_cpy = cv_image.copy()
        #Color to HSV and Gray Scale conversion
        hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)
        #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		#img = cv_image
        gray = cv2.cvtColor(img_original, cv2.COLOR_BGR2GRAY)
        bodies = self.body_cascade.detectMultiScale(gray,1.3,5)
        for (x,y,w,h) in bodies:
			cv2.rectangle(img_for_presentation, (x,y), (x+w, y+h), (255,0,0), 2)
			
        #Thresholds

        # worked with one video for TB, but not the other:
        # 0-7 70-220 70-250
        # 190-255 20-255 20-255
        if(self.modeIsDrone):
            lower_red_upper = np.array([0, 70, 70])#drone didn't use
            upper_red_upper = np.array([7, 220,200])#drone didn't use
            lower_red_lower = np.array([140, 40, 80])#drone 140,40,80
            upper_red_lower = np.array([190, 200,230])#drone 190,200,230
            lower_blue = np.array([117,75,50])#drone 117,75,50
            upper_blue = np.array([127,150,170])#drone 127,150,170
            lower_green = np.array([28,62,60])#drone 28,62,60
            upper_green = np.array([48,170,170])#drone 48,170,170
        else :
            lower_red_upper = np.array([0, 70, 70])    #TB 0, 70, 70
            upper_red_upper = np.array([7, 220, 200])
            #   TB 7, 220 200
            lower_red_lower = np.array([140, 30, 30])#TB 140, 30, 30
            upper_red_lower = np.array([255, 230,150])#TB 255, 230,150
            lower_blue = np.array([117,75,50])#TB 117,75,50
            upper_blue = np.array([127,150,170])#TB 127,150,170
            lower_green = np.array([28,62,60])#TB 28,62,60
            upper_green = np.array([48,170,170])#TB 48,170,170
            
        ## Threshold the HSV image to get only single color portions
        #greenMask = cv2.inRange(hsv, lower_green, upper_green)
        #green_cv_image = cv2.bitwise_and(cv_image, cv_image, mask=greenMask)
        ##self.grayGreenBoy = cv2.imread('/home/marten/Pictures/GreenBoy.png',0)
        ##greenBoy_gray = cv2.cvtColor(greenBoy, cv2.COLOR_BGR2GRAY)
        #cv_image_gray = cv2.cvtColor(green_cv_image, cv2.COLOR_BGR2GRAY)
        ##gauss_cv_image_gray = cv2.adaptiveThreshold(cv_image_gray, 255, cv2. 
        #wGreenBoy, hGreenBoy = self.grayGreenBoy.shape[::-1]       
        #greenBoyMatchingResult=cv2.matchTemplate(cv_image_gray, self.grayGreenBoy, cv2.TM_CCOEFF_NORMED)
        #threshold = 0.5
        #locGreenBoy = np.where(greenBoyMatchingResult >=threshold)
        #for pt in zip (*locGreenBoy[::-1]):
			#print('found greenBoy!!')
			#cv2.rectangle(cv_image, pt, (pt[0]+wGreenBoy, pt[1]+hGreenBoy), (0,0,255), 2)
			
		## Threshold the HSV image to get only single color portions
        #blueMask = cv2.inRange(hsv, lower_blue, upper_blue)
        #blue_cv_image = cv2.bitwise_and(cv_image, cv_image, mask=blueMask)
        #cv_image_gray = cv2.cvtColor(blue_cv_image, cv2.COLOR_BGR2GRAY)
        #wBlueBoy, hBlueBoy = self.grayGreenBoy.shape[::-1]       
        #blueBoyMatchingResult=cv2.matchTemplate(cv_image_gray, self.grayGreenBoy, cv2.TM_CCOEFF_NORMED)
        #threshold = 0.5
        #locBlueBoy = np.where(blueBoyMatchingResult >=threshold)
        #for pt in zip (*locBlueBoy[::-1]):
			#print('found blueBoy!!')
			#cv2.rectangle(cv_image, pt, (pt[0]+wBlueBoy, pt[1]+hBlueBoy), (255,0,255), 2)	
			
	    # Threshold the HSV image to get only single color portions
        redMask_upper = cv2.inRange(hsv, lower_red_upper, upper_red_upper)
        redMask_lower = cv2.inRange(hsv, lower_red_lower, upper_red_lower)
        redMask = cv2.bitwise_or(redMask_upper, redMask_lower)
        
        red_cv_image = cv2.bitwise_and(img_original, img_original, mask=redMask)
        cv_image_gray = cv2.cvtColor(red_cv_image, cv2.COLOR_BGR2GRAY)
        wRedBoy, hRedBoy = self.grayRedBoy.shape[::-1]       
        redBoyMatchingResult=cv2.matchTemplate(cv_image_gray, self.grayRedBoy, cv2.TM_CCOEFF_NORMED)
        threshold = 0.3#0.4 for drone
        locRedBoy = np.where(redBoyMatchingResult >=threshold)
        for pt in zip (*locRedBoy[::-1]):
			print('found Medical Kit!!')
			cv2.rectangle(img_for_presentation, pt, (pt[0]+wRedBoy, pt[1]+hRedBoy), (0,255,255), 2)

        #Find contours(borders) for the shapes in the image
        #NOTE if you get following error:
        # contours, hierarchy = cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # ValueError: need more than two values to unpack
        # change following line to:
        # contours, hierarchy = cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #marten: Have added "_, " to get it working with new cv2:
        #_, contours, hierarchy = cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #_, contours, hierarchy = cv2.findContours(greenMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        ##Pass through each contour and check if it has required properties to classify into required object
        #for x in range (len(contours)):
            #contourarea = cv2.contourArea(contours[x]) #get area of contour
            #if contourarea > 600: #Discard contours with a small area as this may just be noise
                ##The below 2 functions help you to approximate the contour to a nearest polygon
                #arclength = cv2.arcLength(contours[x], True)
                #approxcontour = cv2.approxPolyDP(contours[x], 0.02 * arclength, True)
                ##Find the coordinates of the polygon with respect to he camera frame in pixels
                #rect_cordi = cv2.minAreaRect(contours[x])
                #obj_x = int(rect_cordi[0][0])
                #obj_y = int(rect_cordi[0][1])

                ##Check for Square
                #if len(approxcontour) == 4:
                    ##print ('Length ', len(approxcontour))
                    #cv2.drawContours(cv_image,[approxcontour],0,(0,255,255),2)
                    #approxcontour = approxcontour.reshape((4,2))
                    #LongestSide = Longest_Length(approxcontour)
                    #Distance = (self.focal_leng*self.square_side_lenth)/LongestSide #focal length x Actual Border width / size of Border in pixels

                ##Move to next Contour
                #else :
                    #continue

                ##Calculate Cordinates wrt to Camera, convert to Map
                ##Coordinates and publish message for storing
                ##319.5, 239.5 = image centre
                #obj_cam_x = ((obj_x - 319.5)*Distance)/self.focal_leng
                #obj_cam_y = ((obj_y - 239.5)*Distance)/self.focal_leng

                ##convert the x,y in camera frame to a geometric stamped point
                #P = PointStamped()
                #P.header.stamp = rospy.Time.now() - rospy.Time(23)
                ##print ('time: ', data.header.stamp)
                #P.header.frame_id = 'camera_rgb_optical_frame'
                #P.point.x = obj_cam_x
                #P.point.y = obj_cam_y
                #P.point.z = Distance

                #Transform Point into map coordinates
                #trans_pt = self.tl.transformPoint('/map', P)

                #fill in the publisher object to publish
                #marten obj_info_pub = object_loc()
                #marten obj_info_pub.ID = 27 #ID need to be changed
                #obj_info_pub.point.x = trans_pt.point.x
                #obj_info_pub.point.y = trans_pt.point.y
                #obj_info_pub.point.z = trans_pt.point.z

                #publish the message
                #marten self.object_location_pub.publish(obj_info_pub)

        #Display the captured image
        #cv2.imshow("Image",cv_image)
        #cv2.imshow("HSV", hsv)
        #cv2.imshow("res",res)
        #cv2.imshow("Green screen",green_cv_image)
       
        #cv2.imshow("Blue screen",blue_cv_image)
        cv2.imshow("img",img_for_presentation)
        cv2.imshow("Red screen",red_cv_image)
        cv2.waitKey(1)


#Check validity of the mode argument provided
def check_mode(v):
    if v != 'turtlebot' and v != 'drone':
        raise argparse.ArgumentTypeError('mode:%s is neither turtlebot nor drone' % v)
    return v

#Check validity of the source argument provided
def check_source(v):
    if v != 'raw' and v != 'compressed':
        raise argparse.ArgumentTypeError('source:%s is neither raw nor compressed' % v)
    return v

#Main function for the node
def main(args):
    #Get options
    parser = argparse.ArgumentParser(description='List possible arguments for the node')
    parser.add_argument('mode', nargs='?', type=check_mode, default='turtlebot',
                        help='mode to set use (turtlebot or drone)')
    parser.add_argument('-s', '--source', nargs='?', type=check_source, default=None,
                        help='force the source of image to use (raw or compressed)')
    parser.add_argument('-d', '--deactivate', nargs='+', default=None,
                        help='force the deactivation of certain detections (human or medic)')

    args = parser.parse_args()

    #Add some cleverness
    if args.source is None:
        args.source = 'compressed'

    #Start doing stuff
    print('Received:\n %s' % args) #For testing

    print('Starting OpenCV object detection !')
    rospy.init_node('object_detection', anonymous = False)
    print('ROS node initialized !')
    ic = object_detection(args)
    print('Object detection class created !')
    try:
        print('Spin !')
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down object_detection Node')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
