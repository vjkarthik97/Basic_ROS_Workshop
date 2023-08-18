#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
import time
#Run this command on a parallel tab in the terminal before running this node
#roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
def Open_Loop_Controller():

	#Initializing the Node
	sendCommand = rospy.Publisher('/cmd_vel',Twist,queue_size=5)
	rospy.init_node('Basic_Robot_Control')
	rate = rospy.Rate(10) #10Hz
	velocity_msg = Twist()
	while not rospy.is_shutdown():

		velocity_msg.linear.x = 0.1
		velocity_msg.angular.z = 0.1
		sendCommand.publish(velocity_msg) #Publishing the linear and angular velocity command
		rate.sleep()
if __name__ == '__main__':
	try:
		time.sleep(2)
		Open_Loop_Controller()
	except rospy.ROSInterruptException:
		pass  
