#!/usr/bin/env python
#'''
#This source will act as support to finish your project and does not follow best
#coding practices.
#'''
#Import Python Packages, ROS messages
from __future__ import print_function
from __future__ import division
import sys
#import roslib
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
#import the custom message we created to store objects
from wasp_custom_msgs.msg import object_loc
import tf

import argparse

class obj_map_item:
    def __init__(self, obj_type_id=None, map_x=0, map_y=0, map_z=0):
        self.t = obj_type_id
        self.x = map_x
        self.y = map_y
        self.z = map_z

#This is the main class for object detection, it has some initializations about nodes
#Call back functions etc
class object_detection:
    def __init__(self, args):
        #Create Rospy Publisher and subscriber
        self.object_location_pub = rospy.Publisher('/object_location', object_loc, queue_size =1)

        #Get image from turtlebot/drones...
        source = Image
        self.modeIsDrone = False
        if args.mode == 'drone': #Drone
            topic = '/bebop/image_raw/'
            self.modeIsDrone = True
        else: #Turtlebot
            topic = '/camera/rgb/image_raw/'

        if args.source is not None and args.source == 'compressed':
            print('Using compressed image!')
            topic += 'compressed/'
            source = CompressedImage

        print('Get image from of %s from %s' %(args.mode, topic))
        self.image_sub = rospy.Subscriber(topic, source, self.callback)
        #Cv Bridge is used to convert images from ROS messages to numpy array for openCV and vice versa
        self.bridge = CvBridge()
        #Object to transform listener which will be used to transform the points from one coordinate system to other.
        self.tl = tf.TransformListener()
        self.grayMedBoxSmall = cv2.imread('MedBox.png',0)#MB funkade med forsta bilden
        self.grayMedBoxLarge = cv2.imread('MedBox100px.png',0)#MB funkade med forsta bilden
        self.grayGreenBoy = cv2.imread('BlackPerson.png',0)

        self.body_cascade = cv2.CascadeClassifier('haarcascade_fullbody.xml')
        self.known_obj_map_list = [] #List containing detected objects
        self.obj_map_margin = 10 #Margin to consider for avoiding repetitive objects
        self.jumpOver = 1

    #Callback function for subscribed image
    def callback(self,data):
        self.jumpOver=self.jumpOver+1
        self.jumpOver=self.jumpOver%20
        if self.jumpOver==1:
            np_arr = np.fromstring(data.data, np.uint8)
            #The following is no longer named CV_LOAD_IMAGE_COLOR but CV_LOAD_COLOR. Works by defining it instead
            cv2.CV_LOAD_IMAGE_COLOR = 1
            img_for_presentation = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            img_original = cv2.copyMakeBorder(img_for_presentation,0,0,0,0,cv2.BORDER_REPLICATE)
            #cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            #Create copy of captured image
            #img_cpy = cv_image.copy()
            #Color to HSV and Gray Scale conversion
            hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)

            #img = cv_image
            gray = cv2.cvtColor(img_original, cv2.COLOR_BGR2GRAY)
            bodies = self.body_cascade.detectMultiScale(gray,1.3,5)
            for (x,y,w,h) in bodies:
                #print('Found a person!')
                self.calc_coord(w, h, w, h, 'person')
                cv2.rectangle(img_for_presentation, (x,y), (x+w, y+h), (255,0,0), 2)

            #Thresholds
            # worked with one video for TB, but not the other:
            # 0-7 70-220 70-250
            # 190-255 20-255 20-255
            if(self.modeIsDrone):
                lower_red_upper = np.array([0, 120, 65])#drone didn't use
                #upper_red_upper = np.array([0, 100, 80])#drone didn't use
                upper_red_upper = np.array([7, 220,220])#drone didn't use
                lower_red_lower = np.array([140, 90, 80])#drone 140,40,80
                #upper_red_lower = np.array([140, 90,80])#drone 190,200,230
                upper_red_lower = np.array([190, 240,230])#drone 190,200,230
            else :
                lower_red_upper = np.array([0, 120, 90])    #TB 0, 70, 70
                #upper_red_upper = np.array([0, 120, 90])  #   TB 7, 220 200
                upper_red_upper = np.array([7, 255, 255])  #   TB 7, 220 200
                lower_red_lower = np.array([140, 90, 60])#TB 140, 30, 30
                #upper_red_lower = np.array([140, 30, 30])#TB 255, 230,150
                upper_red_lower = np.array([255, 240,255])#TB 255, 230,150

            # Threshold the HSV image to get only single color portions
            redMask_upper = cv2.inRange(hsv, lower_red_upper, upper_red_upper)
            redMask_lower = cv2.inRange(hsv, lower_red_lower, upper_red_lower)
            redMask = cv2.bitwise_or(redMask_upper, redMask_lower)

            red_cv_image = cv2.bitwise_and(img_original, img_original, mask=redMask)
            cv_image_gray = cv2.cvtColor(red_cv_image, cv2.COLOR_BGR2GRAY)
            wMedBoxSmall, hMedBoxSmall = self.grayMedBoxSmall.shape[::-1]
            wMedBoxLarge, hMedBoxLarge = self.grayMedBoxLarge.shape[::-1]
            medBoxSmallMatchingResult=cv2.matchTemplate(cv_image_gray, self.grayMedBoxSmall, cv2.TM_CCOEFF_NORMED)
            medBoxLargeMatchingResult=cv2.matchTemplate(cv_image_gray, self.grayMedBoxLarge, cv2.TM_CCOEFF_NORMED)
            if(self.modeIsDrone):
                thresholdMedBoxSmall = 0.43
                thresholdMedBoxLarge = 0.25
            else :
                thresholdMedBoxSmall = 0.25
                thresholdMedBoxLarge = 0.25
            locMedBoxSmall = np.where(medBoxSmallMatchingResult >=thresholdMedBoxSmall)
            locMedBoxLarge = np.where(medBoxLargeMatchingResult >=thresholdMedBoxLarge)
            for pt in zip (*locMedBoxSmall[::-1]):
                #print('found Medical Kit far away!!')
                self.calc_coord(pt[0], pt[1], wMedBoxSmall, hMedBoxSmall, 'medkit_far')
                cv2.rectangle(img_for_presentation, pt, (pt[0]+wMedBoxSmall, pt[1]+hMedBoxSmall), (0,255,255), 2)
            for pt in zip (*locMedBoxLarge[::-1]):
                #print('found Medical Kit near!!')
                self.calc_coord(pt[0], pt[1], wMedBoxLarge, hMedBoxLarge, 'medkit_near')
                cv2.rectangle(img_for_presentation, pt, (pt[0]+wMedBoxLarge, pt[1]+hMedBoxSmall), (50,200,200), 2)
            #Display the captured image

            #cv2.imshow("Red screen",red_cv_image)
            #cv2.imshow("Blue screen",blue_cv_image)
            #cv2.imshow("Green screen",green_cv_image)
            cv2.imshow("img",img_for_presentation)
            cv2.imshow("red",red_cv_image)
            cv2.waitKey(1)

    #Calculate coordinates according to picture size and stuff
    def calc_coord(self, x, y, w, h, obj):
        #print('Received: x:%d y:%d w:%d h:%d obj:%s' %(x, y, w, h, obj))
        #Image center
        ctr_x = 239.5
        ctr_y = 319.5

        #Set focal length
        if self.modeIsDrone:
            focal_leng = 570.34222
        else:
            focal_leng = 570.34222

        #Set properties per object detection type
        if obj == 'medkit_near':
            obj_orig_w = 17.5 #cm
            obj_orig_h = 17.5 #cm
            #obj_orig_d = 1 #cm
            obj_type_id = 1
        if obj == 'medkit_far':
            obj_orig_w = 17.5 #cm
            obj_orig_h = 17.5 #cm
            #obj_orig_d = 1 #cm
            obj_type_id = 1
        elif obj == 'person':
            obj_orig_w = 10 #cm
            obj_orig_h = 26 #cm
            #obj_orig_d = 1 #cm
            obj_type_id = 2
        else:
            return None

        #Calculate distance of object from the camera
        obj_dist_x = (obj_orig_w * focal_leng) / w
        obj_dist_y = (obj_orig_h * focal_leng) / h
        dist = min(obj_dist_x, obj_dist_y) #The minimal value should be the correct one

        #Calculate position of object from the camera
        obj_mid_x = x + w/2
        obj_mid_y = y + h/2
        obj_cam_x = ((obj_mid_x - ctr_x)*dist) / focal_leng
        obj_cam_y = ((obj_mid_y - ctr_y)*dist) / focal_leng
        #print('Drone:%d dist:%dcm (x:%d,y:%d), cam:x:%d,y:%d' %(self.modeIsDrone,
        #      dist, obj_dist_x, obj_dist_y, obj_cam_x, obj_cam_y))

        #convert the x,y in camera frame to a geometric stamped point
        P = PointStamped()
        P.header.stamp = rospy.Time(0)
        P.header.frame_id = 'camera_rgb_optical_frame'
        P.point.x = obj_cam_x
        P.point.y = obj_cam_y
        P.point.z = dist

        #Transform Point into map coordinates
        trans_pt = self.tl.transformPoint('/map', P)
        if not self.obj_exists(trans_pt, obj_type_id):
            self.publish_obj(trans_pt, obj_type_id)

    def obj_exists(self, map_pt, obj_type):
        '''
        Verifies if the object at map_pt is already in the map.
        If so return True, otherwise return False and add to the list of known obj
        '''
        margin = 10
        margin_big = 20
        obj_found = False

        #Get new map point
        nx = int(map_pt.point.x)
        ny = int(map_pt.point.y)
        nz = int(map_pt.point.z)

        for o in self.known_obj_map_list:
            if o.t != obj_type:
                #Object type differs
                continue

            #Check if coordinates are the same, if so we found it !
            ox = range(int(o.x - margin), int(o.x + margin))
            oy = range(int(o.y - margin), int(o.y + margin))
            oz = range(int(o.z - margin_big), int(o.z + margin_big))
            if (nx in ox) and (ny in oy) and (nz in oz):
                obj_found = True
                break

        #If object was not found, add it to the list :)
        if not obj_found:
            new_obj = obj_map_item(obj_type_id=obj_type, map_x=nx,
                                   map_y=ny, map_z=nz)
            self.known_obj_map_list.append(new_obj)
            print('Added t:%d x:%d y:%d z:%d to list (now %d elements)' %(new_obj.t,
                  new_obj.x, new_obj.y, new_obj.z, len(self.known_obj_map_list)))

        return obj_found

    def publish_obj(self, map_pt, obj_type):
        '''
        Publishes the object coordinates according to the map
        '''
        #Fill in the publisher object to publish
        obj_info_pub = object_loc()
        obj_info_pub.ID = obj_type
        obj_info_pub.point.x = map_pt.point.x
        obj_info_pub.point.y = map_pt.point.y
        obj_info_pub.point.z = map_pt.point.z

        #Publish the message
        self.object_location_pub.publish(obj_info_pub)
        print('Published obj ID:%d at x:%d y:%d z:%d' %(obj_type, map_pt.point.x,
                                                        map_pt.point.y, map_pt.point.z))


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
    object_detection(args)
    print('Object detection class created !')
    try:
        print('Spin !')
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down object_detection Node')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
