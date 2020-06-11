
#!/usr/bin/python

# Extract images from a bag file.

#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import numpy as np
# Reading bag filename from command line or roslaunch parameter.
import os
import sys


class ImageCreator():


	def __init__(self):
		self.bridge = CvBridge()
       	with rosbag.Bag('/home/wzy/catkin_ws/src/ugv2uav/rosbag_images/three.bag', 'r') as bag:  
            for topic,msg,t in bag.read_messages():
                if topic == "/ugv1/camera/rgb/image_raw_L/compressed": 
					print 'received image of type: "%s"' % msg.format
					np_arr = np.fromstring(msg.data,np.uint8)
					cv_image = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
					img_file = "/home/wzy/catkin_ws/src/ugv2uav/rosbag_images/three"
					timestr = "%.6f" %  msg.header.stamp.to_sec()
                        
					image_name = timestr+ ".png"
					cv2.imwrite(img_file +"/"+ image_name, cv_image)

if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
