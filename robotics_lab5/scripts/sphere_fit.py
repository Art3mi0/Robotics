#!/usr/bin/env python3
import rospy
import numpy as np
from robot_vision_lectures.msg import SphereParams
from robot_vision_lectures.msg import XYZarray

# Initialize messages
sphere_params_msg = SphereParams()
xyzarray_msg = []
A = []
B = []

def array_callback(data):
	global xyzarray_msg
	global A
	global B
	A = []
	A.append(data.points)

def param_callback(data):
	global sphere_params_msg
	sphere_params_msg.xc = data.xc
	sphere_params_msg.yc = data.yc
	sphere_params_msg.zc = data.zc
	sphere_params_msg.radius = data.radius

if __name__ == '__main__':
	# Intialize the node
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to read ball detection output
	rospy.Subscriber("/xyz_cropped_ball", XYZarray, array_callback) 
	# define a publisher to publish estimated sphere parameters
	#param_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 10)
	test_pub = rospy.Publisher('/XYZarray', XYZarray, queue_size = 10)
	loop_rate = rospy.Rate(10)
	estimate = SphereParams()
	
	while not rospy.is_shutdown():
		#test_pub.publish(xyzarray_msg)
		print(A)

		loop_rate.sleep()
