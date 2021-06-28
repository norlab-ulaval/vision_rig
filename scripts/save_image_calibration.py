#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Point cloud message
from sensor_msgs.msg import PointCloud2
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# Sauvegarder le temps
import time

# Instantiate CvBridge
bridge = CvBridge()

# Command line for saving pointcloud: rosrun pcl_ros pointcloud_to_pcd input:=/save_pcd
class PointClouds:

    def __init__(self):
        self.pub = rospy.Publisher("save_pcd", PointCloud2, queue_size=1)

    def callback(self, msg):
        print("Received an Point cloud!")
        rospy.sleep(2)
        self.pub.publish(msg)

class Images:

    def __init__(self):
        self.number_images = 0
    
    def callback(self, msg):
        print("Received an image!")
        self.number_images += 1
        try:
        # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
	        cv2.imwrite('/home/xavier/Desktop/camera_image_{}.jpeg'.format(time.time()), cv2_img)


#def image_callback(msg):
#    print("Received an image!")
#    global number_image
#    #if number == 0:
#        # Simulation semble planter au debut et il n'y a rien dans les images
#        #rospy.sleep(20)
#    number_image += 1
#    try:
#        # Convert your ROS Image message to OpenCV2
#        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
#    except CvBridgeError, e:
#        print(e)
#    else:
#        # Save your OpenCV2 image as a jpeg
#        cv2.imwrite('/home/olivierg/Desktop/tests/camera_image_{}.jpeg'.format(number_image), cv2_img)

#def pointcloud_callback(msg):
#    # rosrun pcl_ros pointcloud_to_pcd input:=/save_pcd
#    print("Received an Point cloud!")
#    print(msg)
#    #global number_pointcloud
#    #number_pointcloud += 1
#    pub = rospy.Publisher("save_pcd",PointCloud2, queue_size=1)
#    pub.publish(msg)


def main():
    rospy.init_node('image_pcl_listener')
    # Define your image and lidar topic
    image_topic = "/Camera2/pylon_camera_node/image_color_rect"
    lidar_topic = "/lslidar_point_cloud"
    save_pcl = PointClouds()
    save_image = Images()
    # Set up your subscriber and define its callback
    #rospy.Subscriber(image_topic, Image, image_callback)
    #rospy.Subscriber(lidar_topic, PointCloud2, pointcloud_callback)
    msg_image = rospy.wait_for_message(image_topic, Image, timeout=None)
    save_image.callback(msg_image)
    msg_pcl = rospy.wait_for_message(lidar_topic, PointCloud2, timeout=None)
    save_pcl.callback(msg_pcl)
    # Spin until ctrl + c
    #rospy.spin()
    rospy.sleep(0.1)

if __name__ == '__main__':
    main()

