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
import os
import inspect

class obj_map_item:
    def __init__(self, obj_type_id=None, map_x=0, map_y=0, map_z=0):
        self.t = obj_type_id
        self.x = map_x
        self.y = map_y
        self.z = map_z

class obj_item:
    def __init__(self, x=0, y=0, w=0, h=0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

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
        #Get the python code path to retrieve masks and write stuff
        self.pathScript = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        self.pathScript = self.pathScript + '/'
        self.objCoordsLog = self.pathScript + 'obj_detected.log'
        #Delete possible previous objects list file (if only one instance planned)
        try:
            os.remove(self.objCoordsLog)
        except OSError:
            pass
        #Write file header
        with open(self.objCoordsLog, 'w') as l:
            l.write('%s\t%s\t\t%s\t%s\t%s\n' %('Source', 'Object', 'mapX', 'mapY', 'mapZ'))
        #Load masks
        self.grayMedBox = cv2.imread(self.pathScript+'MedBox.png',0)#MB funkade med forsta bilden
        self.grayGreenBoy = cv2.imread(self.pathScript+'BlackPerson.png',0)

        self.body_cascade = cv2.CascadeClassifier(self.pathScript+'haarcascade_fullbody.xml')
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
            gray = cv2.cvtColor(img_original, cv2.COLOR_BGR2GRAY)

            #Do some filtering magic
            #Dilate and open image (join parts and reduce noise)
            kernel = np.ones((2,2), np.uint8)
            img_trans = cv2.morphologyEx(img_original, cv2.MORPH_OPEN, kernel)
            img_trans = cv2.dilate(img_original, kernel, iterations=1)
            #Median blur
            #img_trans = cv2.medianBlur(img_trans, 3)
            #Equalize
            b, g, r, = cv2.split(img_trans)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            cb = clahe.apply(b)
            cg = clahe.apply(g)
            cr = clahe.apply(r)
            #Threshold red (to enhance detection)
            cr = cv2.compare(cr, np.uint8([90]), cv2.CMP_GE)
            #Merge final image
            img_enhanced = cv2.merge((cb, cg, cr))

            #Color to HSV
            hsv = cv2.cvtColor(img_enhanced, cv2.COLOR_BGR2HSV)

            #Detect people (Haar cascade)
            bodies = self.body_cascade.detectMultiScale(gray,1.3,5)
            for (x,y,w,h) in bodies:
                #print('Found a person!')
                self.calc_coord(w, h, w, h, 'person')
                cv2.rectangle(img_for_presentation, (x,y), (x+w, y+h), (255,0,0), 2)

            #HSV Thresholds
            if(self.modeIsDrone):
                lower_red_upper = np.array([0, 120, 65])#drone didn't use
                upper_red_upper = np.array([7, 220,220])#drone didn't use
                lower_red_lower = np.array([140, 90, 80])#drone 140,40,80
                upper_red_lower = np.array([190, 240,230])#drone 190,200,230
            else :
                lower_red_upper = np.array([0, 100, 100])    #TB 0, 70, 70
                upper_red_upper = np.array([10, 255, 255])  #   TB 7, 220 200
                lower_red_lower = np.array([160, 100, 100])#TB 140, 30, 30
                upper_red_lower = np.array([179, 255, 255])#TB 255, 230,150

            # Threshold the HSV image to get only single color portions
            redMask_upper = cv2.inRange(hsv, lower_red_upper, upper_red_upper)
            redMask_lower = cv2.inRange(hsv, lower_red_lower, upper_red_lower)
            #redMask = cv2.addWeighted(redMask_lower, 0.5, redMask_upper, 1.0, 0.0)
            redMask = cv2.bitwise_or(redMask_upper, redMask_lower)
            redMask = cv2.GaussianBlur(redMask, (3,3), 1)

            if(self.modeIsDrone):
                thresholdMedBox = 0.5
            else :
                thresholdMedBox = 0.5

            wMedBox, hMedBox = self.grayMedBox.shape[::-1]
            wMedBox4 = wMedBox * 4
            hMedBox4 = hMedBox * 4

            foundMedBoxes = []
            #Scale image to the pattern we look for
            for scale in np.linspace(0.2, 1.0, 10)[::-1]:
                r_img = cv2.resize(redMask, (0, 0), fx=scale, fy=scale)
                r = redMask.shape[1] / float(r_img.shape[1])
                #Stop if smaller than 4x the shape to detect
                if r_img.shape[0] < wMedBox4 or r_img.shape[1] < hMedBox4:
                    break
                result = cv2.matchTemplate(r_img, self.grayMedBox, cv2.TM_CCOEFF)
                result = np.where(result >= thresholdMedBox)
                for match in zip(*result[::-1]):
                    v = obj_item()
                    v.x = int(match[0] * r)
                    v.y = int(match[1] * r)
                    v.w = int((match[0] + wMedBox) * r)
                    v.h = int((match[1] + hMedBox) * r)
                    foundMedBoxes.append(v)

            for pt in foundMedBoxes:
                self.calc_coord(v.x, v.y, v.w, v.h, 'medkit_far')
                cv2.rectangle(img_for_presentation, (v.x, v.y), (v.w, v.h), (0,255,255), 2)

            #Display the captured image
            cv2.imshow("img",img_for_presentation)
            #cv2.imshow("img enhanced",img_enhanced)
            #cv2.imshow("Red mask", redMask)
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
        #trans_pt = self.tl.transformPoint('/map', P)
        trans_pt = P #TEST: DELETE THIS STUFF, and F*CK the mapping sh*t
        if not self.obj_exists(trans_pt, obj_type_id):
            self.publish_obj(trans_pt, obj_type_id, obj, log=True)

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

    def publish_obj(self, map_pt, obj_type, obj_type_txt, log=False):
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

        #Write to logfile
        if log:
            src = 'Turtle'
            if self.modeIsDrone:
                src = 'Drone'
            with open(self.objCoordsLog, 'a') as l:
                l.write('%s\t%s\t\t%d\t%d\t%d\n' %(src,
                                                 obj_type_txt,
                                                 obj_info_pub.point.x,
                                                 obj_info_pub.point.y,
                                                 obj_info_pub.point.z))


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
