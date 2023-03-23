#!/usr/bin/env python3
import rospy
import numpy as np
from robot_vision_lectures.msg import SphereParams
from robot_vision_lectures.msg import XYZarray

# Initialize messages
sphere_params_msg = SphereParams()
xyzarray_msg = XYZarray()

def params_callback(data):
	global sphere_params_msg
	sphere_params_msg.points = data.points

def array_callback(data):
	global xyzarray_msg
	xyzarray_msg.xc = data.xc
	xyzarray_msg.yc = data.yc
	xyzarray_msg.zc = data.zc
	xyzarray_msg.radius = data.radius

if __name__ == '__main__':
	# Intialize the node
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to read ball detection output
	rospy.Subscriber("/ball_2D", SphereParams, params_callback) 
	# define a publisher to publish estimated sphere parameters
	param_pub = rospy.Publisher('/sphere_params', XYZarray, queue_size = 10)
	loop_rate = rospy.Rate(10)
	estimate = XYZarray()
