#! /usr/bin/env python

import rospy
import time

from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist



def take_action(distance_front):
	msg = Twist()
	linear_x = 0
	angular_z = 0

	state_description = ''

	if distance_front > 0.3:
		state_description = 'Go forward'
		linear_x = 0.5
		angular_z = 0
		print(linear_x)
		print("tamos en go forward")

	elif distance_front <= 0.3:
		state_description = 'Turn right'
		linear_x = 0.0
		angular_z = 1.5
		t0 = rospy.Time.now().secs
		current_angle = 0
		#estimate velocity by iterations
		while (current_angle < 1.570796): #it will turn 90 degrees
			angular_z = 1  #velocidad angular del micromouse
			t1 = time.time()
			delta = t1-t0

			print(current_angle)
			current_angle+=angular_z*(t1-t0)
			msg.angular.z = angular_z
			pub.publish(msg)
			t0 = t1
		print("dio la media vuelta oowowowowo")	
		angular_z = 0	
		msg.angular.z = angular_z 
		#time.sleep(2)
	else:
		state_description = 'Unknown case'
		rospy.loginfo(distance_front)

	rospy.loginfo(state_description)
	msg.linear.x = linear_x
	msg.angular.z = angular_z
	pub.publish(msg)


def clbk_if(msg):
	distance_front = msg.range
	rospy.loginfo(distance_front)

	take_action(distance_front)

def main():

	global pub

	rospy.init_node('reading_ir')

	pub = rospy.Publisher('/cmd_vel',Twist , queue_size = 1)

	sub = rospy.Subscriber('/sensor/ir_front', Range, clbk_if)


	rospy.spin()

if __name__ == '__main__':
	main()
