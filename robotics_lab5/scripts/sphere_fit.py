#!/usr/bin/env python3
import rospy
import numpy as np
from robot_vision_lectures.msg import SphereParams
from robot_vision_lectures.msg import XYZarray

# Initialize message
sphere_params_msg = SphereParams()

# Initialize array
P = []

# Method for estimating coordinates
def array_callback(data):
	global P
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
	
# Method for creating a SphereParams message
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
	param_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 10)
	loop_rate = rospy.Rate(10)
	estimate = SphereParams()
	
	while not rospy.is_shutdown():
		# Try except is used because in the beginning the program gets
		# empty lists
		try:
			# Initialize variables for clarity
			x = P[0][0]
			y = P[1][0]
			z = P[2][0]
			p = P[3][0]
			
			# Assign message parameters
			sphere_params_msg.xc = x
			sphere_params_msg.yc = y
			sphere_params_msg.zc = z
			# Calculates radius
			sphere_params_msg.radius = (p + (x **2) + (y **2) + (z **2)) **(1/2)
			# Publishes to topic that will display in rviz
			param_pub.publish(sphere_params_msg)
		except:
			# I am assuming the program takes a second before recieving 
			# data, and that is why it uses empty arrays at the start
			print("Program Initializing...")

		loop_rate.sleep()
