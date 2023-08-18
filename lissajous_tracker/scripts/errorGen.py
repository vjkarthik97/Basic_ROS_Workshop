#!/usr/bin/python3

import rospy
import math
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from lissajous_tracker.msg import Error_data
import time
from waypointGen import waypoints


state_x = 0
state_y = 0

state_yaw = 0
prev_yaw = 0


def RobotPose(data):
	global state_x 
	global state_y  

	global state_yaw
	global prev_yaw

	state_x = data.pose.pose.position.x #To get the robot's present x co-ordinate
	state_y = data.pose.pose.position.y #To get the robot's present y co-ordinate
	
	state_yaw = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2] #To get the robot's present heading angle
	
	#To eliminate the spikes(due to 2*pi angle shift) that might arise in the heading angle of the robot. 
	yaw_array = [prev_yaw,state_yaw]
	yaw_array = np.unwrap(yaw_array)
	state_yaw = yaw_array[1]
	
	prev_yaw = state_yaw


# def pose_error(x_cur, y_cur, yaw_cur x_des, y__des, pre_des_heading, z):
# 	x_des, y_des = waypoints(4,1,2);
# 	e_pos = ((x_des - x_cur)**2 + (y_des - y_cur)**2)**0.5 #Error in Position
		
# 	#To eliminate the spikes(due to 2*pi angle shift) that might arise when we calculate the desired heading angle to be driven to.		
# 	des_heading = math.atan2((y_des-y_cur),(x_des-x_cur))

# 	heading_array = [prev_des_heading,des_heading]
# 	heading_array = np.unwrap(heading_array)
# 	    des_heading = heading_array[1];

# 	e_theta = yaw_cur - des_heading

def programflow():
	
	x, y = waypoints(2,1,1); # waypoints

	#Loop Variables
	z = 0
	vir_time = 0

	#Co-ordinates and Errors
	E_pos   = [None] *50000 	#Array to store the error in robot's position
	E_theta = [None] * 50000 #Array to store the error in robot's heading angle
	graph_x = [None]*50000 	#Array to store the robot's present x co-ordinate
	graph_y = [None]*50000	#Array to store the robot's present y co-ordinate

	prev_des_heading = 0

	#Node Initialization
	rospy.init_node('Log_odom')
	rospy.Subscriber('/odom',Odometry,RobotPose)

	rate = rospy.Rate(10) #10 Hz

	#Setting the initial errors to zero in the created custom error message  
	Error_msg = Error_data(0.0,0.0)


	while not rospy.is_shutdown() and z<=600: 

		E_pos[z] = ((state_x - x[z])**2 + (state_y - y[z])**2)**0.5 #Error in Position
		
		#To eliminate the spikes(due to 2*pi angle shift) that might arise when we calculate the desired heading angle to be driven to.		
		des_heading = math.atan2((y[z]-state_y),(x[z]-state_x))

		heading_array = [prev_des_heading,des_heading]
		heading_array = np.unwrap(heading_array)
		des_heading = heading_array[1]
		E_theta[z] = state_yaw - des_heading #Error in Heading angle

		# E_pos[z], E_theta[z] = pose_error(state_x,state_y,state_yaw,x[z],y[z],prev_des_heading)
		
		Error_msg = Error_data(E_pos[z],E_theta[z]) #Storing the Error message to be published

		#Publishing the Error Message
		ErrorPublisher = rospy.Publisher('Error',Error_data,queue_size=5)
		ErrorPublisher.publish(Error_msg)

		#Logging the Robot's present co-ordinates
		rospy.loginfo("waypoints:{},{}".format(x[z], y[z]))
		rospy.loginfo("robot's pose: {}, {}".format(state_x,state_y))	
		#rospy.loginfo(time.time()-start_time)	

		#Storing the Robot's present co-ordinates which is later used for plotting
		graph_x[vir_time] = state_x
		graph_y[vir_time] = state_y
			
		prev_des_heading = des_heading #Storing the present heading angle to be used in the next iteration of the loop
 
		if E_pos[z] < 0.08:
			z = z+1 #Incrementing the loop variable - waypoint

		vir_time = vir_time+1
	
		rate.sleep()

	#Plotting the output
	plt.figure(0)
	plt.title('Robot motion path')
	plt.xlabel('x')
	plt.ylabel('y')
	plt.grid()
	plt.plot(graph_x,graph_y)

	plt.show()	
	

if __name__ == '__main__':
	try:
		time.sleep(7)
		programflow()
	except rospy.ROSInterruptException:
		pass  
