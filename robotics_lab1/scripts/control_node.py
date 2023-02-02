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

if __name__ == '__main__':
	# declare a publisher to publish in the velocity command topic
	cmd_pub = rospy.Publisher('/turtle1/control_params', Twist, queue_size = 10)
	# initialize the node
	rospy.init_node('control_node', anonymous = True)
	# add a subscriber to it to read the position information
	rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	
	loop_rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
	
	
	
