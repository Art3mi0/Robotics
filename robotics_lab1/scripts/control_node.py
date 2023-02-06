#!/usr/bin/env python3

# import ROS for developing the node
import rospy
# import geometry_msgs/Twist for movement commands
from geometry_msgs.msg import Twist
# importing turtlesim.msg pose for current turtle position data
from turtlesim.msg import Pose
#importing the custom message for givng turtle location
from robotics_lab1.msg import Turtlecontrol

# Initiating messages
pos_msg = Pose()
control_msg = Turtlecontrol()

def pose_callback(data):
	global pos_msg
	pos_msg.x = data.x 

def control_callback(data):
	global control_msg
	control_msg.kp = data.kp
	control_msg.xd = data.xd

if __name__ == '__main__':
	# add a subscriber to read the current position information
	rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	# add a subscriber to read the desired position information
	rospy.Subscriber('/turtle1/control_params', Turtlecontrol, control_callback)
	# declare a publisher to publish in the velocity command topic
	cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
	# initialize the node
	rospy.init_node('control_node', anonymous = True)
	
	# setting 10hz frequency for loop 
	loop_rate = rospy.Rate(10)
	# declarng a twist variable
	vel_cmd = Twist()
	# Setting desired location
	control_msg.kp = 1
	control_msg.xd = 8

	while not rospy.is_shutdown():
		# Performing formula to move towards desired location
		vel_cmd.linear.x = (control_msg.kp * (control_msg.xd - pos_msg.x))
		# Publish command
		cmd_pub.publish(vel_cmd)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
	
	
	
