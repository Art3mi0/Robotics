#!/usr/bin/env python3

# import ROS for developing the node
import rospy
# import geometry_msgs/Twist for control commands
from geometry_msgs.msg import Twist
# we are going to read turtlesim/Pose messages
from turtlesim.msg import Pose
#importing the new message from our package
from robotics_lab1.msg import Turtlecontrol
# for radians to degrees conversions
import math

def pose_callback(data):
	global pos_msg
	pos_msg.x = data.x 

def control_callback(data):
	global control_msg
	control_msg.kp = data.kp
	control_msg.xd = data.xd

if __name__ == '__main__':
	# add a subscriber to it to read the position information
	rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	# add a subscriber to it to read the desired position information
	rospy.Subscriber('/turtle1/control_params', Turtlecontrol, control_callback)
	# declare a publisher to publish in the velocity command topic
	cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
	# initialize the node
	rospy.init_node('control_node', anonymous = True)
	
	loop_rate = rospy.Rate(10)
	vel_cmd = Twist()

	while not rospy.is_shutdown():
		vel_cmd.linear.x = (control_msg.kp * (control_msg.xd - pos_msg.x))
		cmd_pub.publisher(vel_cmd)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
	
	
	
