#!/usr/bin/python3

import rospy
import math
from matplotlib import pyplot as plt

def waypoints(A, a, b):
	x = [None] * 1000
	y = [None] * 1000
	i = 0
	for k in range(220):
		x[k] = A*math.cos(a*i)
		y[k] = A*math.sin(b*i)
		i += 0.01
	return x, y

if __name__ == '__main__':
	p, q = waypoints(2,1,1)
	plt.title('Waypoints')
	plt.xlabel('x')
	plt.ylabel('y')
	plt.grid()
	plt.plot(p,q)
	plt.show()

