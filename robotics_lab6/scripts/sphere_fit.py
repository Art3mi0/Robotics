#!/usr/bin/env python3
import rospy
import numpy as np
from robot_vision_lectures.msg import SphereParams
from robot_vision_lectures.msg import XYZarray

# Initialize message
sphere_params_msg = SphereParams()
check = False

# Initialize array
P = []

# Method for estimating coordinates
def array_callback(data):
	global P
	global check
	A = []
	B = []
	# Appending points with modifications to appropriate arrays
	for i in data.points:
		A.append([i.x * 2, i.y * 2, i.z * 2, 1])
		B.append([i.x ** 2 + i.y ** 2 + i.z ** 2])
	# Converting into numpy arrays
	A = np.array(A)
	B = np.array(B)
	
	# Math for calculating estimate using a pseudo-inverse
	# results in 4x1 matrix
	ATA = np.matmul(A.T, A)
	ATB = np.matmul(A.T, B)
	P = np.matmul(np.linalg.inv(ATA), ATB)
	check = True
	
# Method for creating a SphereParams message
def param_callback(data):
	global sphere_params_msg
	sphere_params_msg.xc = data.xc
	sphere_params_msg.yc = data.yc
	sphere_params_msg.zc = data.zc
	sphere_params_msg.radius = data.radius

def coordinate_filter(coordinate, out):
	fil_in = coordinate
	fil_out = out
	fil_gain = 0.7
	fil_out = fil_gain*fil_in + (1 - fil_gain)*fil_out
	return fil_out
	
def radius_filter(radius, out):
	fil_in = radius
	fil_out = out
	fil_gain = 0.8
	fil_out = fil_gain*fil_in + (1 - fil_gain)*fil_out
	return fil_out

if __name__ == '__main__':
	# Intialize the node
	rospy.init_node('sphere_fit', anonymous = True)
	# define a subscriber to read ball detection output
	rospy.Subscriber("/xyz_cropped_ball", XYZarray, array_callback) 
	# define a publisher to publish estimated sphere parameters
	param_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 10)
	loop_rate = rospy.Rate(10)
	estimate = SphereParams()
	global fil_out
	xfil_out = 3
	yfil_out = 3
	zfil_out = 3
	rfil_out = 3
	
	while not rospy.is_shutdown():
		# Using boolean check to help prevent crash and debug
		if check:
			# Initialize variables for clarity
			x = P[0][0]
			y = P[1][0]
			z = P[2][0]
			p = P[3][0]
			
			# Calculates radius
			radius = (p + (x **2) + (y **2) + (z **2)) **(1/2)
			
			xfil_out = coordinate_filter(x, xfil_out)
			yfil_out = coordinate_filter(x, yfil_out)
			zfil_out = coordinate_filter(x, zfil_out)
			rfil_out = radius_filter(radius, rfil_out)
			# Assign message parameters
			sphere_params_msg.xc = xfil_out
			sphere_params_msg.yc = yfil_out
			sphere_params_msg.zc = zfil_out
			sphere_params_msg.radius = rfil_out
			# Publishes to topic that will display in rviz
			param_pub.publish(sphere_params_msg)

		loop_rate.sleep()
