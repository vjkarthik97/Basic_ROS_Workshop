#!/usr/bin/python3

import rospy
from Lissajous_Tracker.msg import Error_data
from geometry_msgs.msg import Twist
import time

position_error = 0
theta_error = 0

def receiveError(data):
	global position_error
	global theta_error

	#Storing the position error that was received
	position_error = data.pos_error
	theta_error = data.theta_error

	#Logging the received position error
	rospy.loginfo("Robot's Position error: {}".format(position_error))
	rospy.loginfo("Robot's Orientation error: {}".format(theta_error))

def sat(x, threshold):
	if x <= -threshold:
		x = -threshold
	if x >= threshold:
		x = threshold
	return x


def talker():
	K1 = 0.25	        #Linear Velocity Gain
	K2 = 0.75	#Angular Velocity Gain

	#Initializing the Node
	sendCommand = rospy.Publisher('/cmd_vel',Twist,queue_size=5)
	rospy.init_node('Control_Data_Sender')
	rospy.Subscriber('Error',Error_data,receiveError)
	rate = rospy.Rate(10) #10Hz

	velocity_msg = Twist()

	while not rospy.is_shutdown():
		# velocity_msg.linear.x = K1*position_error; 
		# velocity_msg.angular.z = -K2*theta_error; 
		velocity_msg.linear.x = sat(K1*position_error,1);
		velocity_msg.angular.z = sat(-K2*theta_error,1);
		sendCommand.publish(velocity_msg) #Publishing the linear and angular velocity command
		rate.sleep()

if __name__ == '__main__':
	try:
		time.sleep(7)
		talker()
	except rospy.ROSInterruptException:
		pass  
