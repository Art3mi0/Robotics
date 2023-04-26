#!/usr/bin/env python3
import rospy
import numpy as np
from robot_vision_lectures.msg import SphereParams
from robot_vision_lectures.msg import XYZarray
from std_msgs.msg import Bool

# Initialize message
sphere_params_msg = SphereParams()
pause_resume = Bool()
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

# Method for applying filter to xyz coordiantes
def coordinate_filter(coordinate, out):
	fil_in = coordinate
	fil_out = out
	fil_gain = 0.03
	fil_out = fil_gain*fil_in + (1 - fil_gain)*fil_out
	return fil_out

# Method for applying filter to radius	
def radius_filter(radius, out):
	fil_in = radius
	fil_out = out
	fil_gain = 0.01
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
	# Initial guesses of first reading
	# Values used from a reading from lab 5 script
	xfil_out = -0.014
	yfil_out = -0.016
	zfil_out = 0.47
	rfil_out = 0.045
	
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
			
			# Apply filter and store variables within loop
			xfil_out = coordinate_filter(x, xfil_out)
			yfil_out = coordinate_filter(y, yfil_out)
			zfil_out = coordinate_filter(z, zfil_out)
			rfil_out = radius_filter(radius, rfil_out)
			# Assign filtered message parameters
			estimate.xc = xfil_out
			estimate.yc = yfil_out
			estimate.zc = zfil_out
			estimate.radius = rfil_out
			# Publishes to topic that will display in rviz
			param_pub.publish(estimate)

		loop_rate.sleep()
