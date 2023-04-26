#!/usr/bin/env python3

import rospy
import math
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs

# import the messages
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from robot_vision_lectures.msg import SphereParams

def createPlan(rStart, pt_in_base):
	# define a plan variable
	plan = Plan()
	plan_point1 = Twist()
	# define a point close to the initial position
	plan_point1.linear.x = rStart.linear.x
	plan_point1.linear.y = rStart.linear.y
	plan_point1.linear.z = rStart.linear.z
	plan_point1.angular.x = rStart.angular.x
	plan_point1.angular.y = rStart.angular.y
	plan_point1.angular.z = rStart.angular.z
	# add this point to the plan
	plan.points.append(plan_point1)
	
	plan_point2 = Twist()
	# Move the claw above the ball
	plan_point2.linear.x = pt_in_base.point.x
	plan_point2.linear.y = pt_in_base.point.y
	plan_point2.linear.z = rStart.linear.z
	plan_point2.angular.x = rStart.angular.x
	plan_point2.angular.y = rStart.angular.y
	plan_point2.angular.z = rStart.angular.z
	# add this point to the plan
	plan.points.append(plan_point2)
	
	plan_point3 = Twist()
	# Move the claw towards the radius of the ball
	plan_point3.linear.x = pt_in_base.point.x
	plan_point3.linear.y = pt_in_base.point.y
	plan_point3.linear.z = pt_in_base.point.z
	plan_point3.angular.x = rStart.angular.x
	plan_point3.angular.y = rStart.angular.y
	plan_point3.angular.z = rStart.angular.z
	# add this point to the plan
	plan.points.append(plan_point3)
	
	plan_point4 = Twist()
	# Move the claw above where the ball was
	plan_point4.linear.x = pt_in_base.point.x
	plan_point4.linear.y = pt_in_base.point.y
	plan_point4.linear.z = rStart.linear.z
	plan_point4.angular.x = rStart.angular.x
	plan_point4.angular.y = rStart.angular.y
	plan_point4.angular.z = rStart.angular.z
	# add this point to the plan
	plan.points.append(plan_point4)
	
	plan_point5 = Twist()
	# Move the claw where the ball started
	plan_point5.linear.x = rStart.linear.x
	plan_point5.linear.y = rStart.linear.y
	plan_point5.linear.z = rStart.linear.z
	plan_point5.angular.x = rStart.angular.x
	plan_point5.angular.y = rStart.angular.y
	plan_point5.angular.z = rStart.angular.z
	# add this point to the plan
	plan.points.append(plan_point5)
	
	plan_point6 = Twist()
	# Move the claw downwards to let go of ball
	plan_point6.linear.x = rStart.linear.x
	plan_point6.linear.y = rStart.linear.y
	plan_point6.linear.z = pt_in_base.point.z
	plan_point6.angular.x = rStart.angular.x
	plan_point6.angular.y = rStart.angular.y
	plan_point6.angular.z = rStart.angular.z
	# add this point to the plan
	plan.points.append(plan_point6)
	print("The Plan:")
	print(plan_point1)
	print(plan_point2)
	print(plan_point3)
	print(plan_point4)
	print(plan_point5)
	print(plan_point6)
	return plan

# Initialize messages for processing data from subscribed nodes
ball_params = SphereParams()
robot_params = Twist()
ball_move = Bool()
count = 0

# Method for storing data from sphere_params
def ball_callback(data):
	global ball_params
	global count
	ball_params.xc = data.xc
	ball_params.yc = data.yc
	ball_params.zc = data.zc
	ball_params.radius = data.radius
	gotData = True
	count += 1

# method for storing data from ur5e_toolpose
def robot_callback(data):
	global robot_params
	robot_params.linear.x = data.linear.x
	robot_params.linear.y = data.linear.y
	robot_params.linear.z = data.linear.z
	robot_params.angular.x = data.angular.x
	robot_params.angular.y = data.angular.y
	robot_params.angular.z = data.angular.z

def move_callback(data):
	global ball_move
	ball_move = data

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# define a subscriber to read estimated sphere coordinates
	param_pub = rospy.Subscriber('/sphere_params', SphereParams, ball_callback)
	pause_pub = rospy.Subscriber('/movement_start', Bool, move_callback)
	# define a subscriber to read robot parameters
	param_pub = rospy.Subscriber('/ur5e/toolpose', Twist, robot_callback)
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	q_rot = Quaternion()
	
	# Flags for displaying when frames are found and when the plan is created
	planComplete = False
	framesCheck = True	
	
	while not rospy.is_shutdown():
		# try getting the most update transformation between the camera frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
			if framesCheck:
				print("Frames available")
				framesCheck = False
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available!!!')
			loop_rate.sleep()
			continue
		# extract the xyz coordinates
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# extract the quaternion and converto RPY
		q_rot = trans.transform.rotation
		roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w])		
		# after data has been collected 24 times, the next collected data
		# will be used for the plan
		if count == 25:
			# define a point in the camera frame
			pt_in_tool = tf2_geometry_msgs.PointStamped()
			pt_in_tool.header.frame_id = 'camera_color_optical_frame'
			pt_in_tool.header.stamp = rospy.get_rostime()
			pt_in_tool.point.x= ball_params.xc
			pt_in_tool.point.y= ball_params.yc
			pt_in_tool.point.z= ball_params.zc
			# convert the 3D point to the base frame coordinates
			pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
			# Send the current robot parameters and ball coordiantes to the planner
			plan = createPlan(robot_params, pt_in_base)
			planComplete = True
			print("Plan is created. Expect robot movement after using rqtgui")
		# publish the plan
		if planComplete and ball_move.data:
			plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
