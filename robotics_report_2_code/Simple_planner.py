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
from robot_vision_lectures.msg import SphereParams

#TODO:
# -add 3 or 4 more plan points. Need to go over target then move. If it
#  it goes straight to the position, it would push the ball away.
# -subscribe to topic that displays current robot position and use that 
#  in planning

def createPlan(pt_in_base):
	# define a plan variable
	plan = Plan()
	plan_point1 = Twist()
	# define a point close to the initial position
	plan_point1.linear.x = -0.6
	plan_point1.linear.y = -0.14
	plan_point1.linear.z = 0.357
	plan_point1.angular.x = 3.07
	plan_point1.angular.y = -0.07
	plan_point1.angular.z = -1.566
	# add this point to the plan
	plan.points.append(plan_point1)
	
	plan_point2 = Twist()
	# Move the claw downwards
	plan_point2.linear.x = pt_in_base.point.x
	plan_point2.linear.y = pt_in_base.point.y
	plan_point2.linear.z = pt_in_base.point.z
	plan_point2.angular.x = 3.07
	plan_point2.angular.y = -0.07
	plan_point2.angular.z = -1.566
	# add this point to the plan
	plan.points.append(plan_point2)

	return plan

ball_params = SphereParams()
count = 0

def ball_callback(data):
	global ball_params
	global count
	ball_params.xc = data.xc
	ball_params.yc = data.yc
	ball_params.zc = data.zc
	ball_params.radius = data.radius
	gotData = True
	count += 1

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# define a subscriber to read estimated sphere coordinates
	param_pub = rospy.Subscriber('/sphere_params', SphereParams, ball_callback)
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	q_rot = Quaternion()
	
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
			# Send the ball coordiantes to the planner
			plan = createPlan(pt_in_base)
			planComplete = True
			print("Plan is created. Expect robot movement")
		# publish the plan
		if planComplete:
			plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
