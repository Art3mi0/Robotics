#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")
# define a lower range
lower_yellow = np.array([100, 100, 1])
# define an upper range
upper_yellow = np.array([255, 255, 100])


# get the image message
def get_image(ros_img):
	global rgb_img
	global img_received
	# convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	# raise flag
	img_received = True

	
if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('ball_2d', anonymous = True)
	# define a subscriber to ream images
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('/ball_2d', Image, queue_size = 1)
	
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			# flip the image up			
			alt_img = cv2.inRange(rgb_img, lower_yellow, upper_yellow)
			# convert it to ros msg and publish it
			img_msg = CvBridge().cv2_to_imgmsg(alt_img, encoding="mono8")
			# publish the image
			img_pub.publish(img_msg)
		# pause until the next iteration			
		rate.sleep()



