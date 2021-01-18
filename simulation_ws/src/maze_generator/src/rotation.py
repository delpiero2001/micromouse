#!/usr/bin/env python
import rospy
import numpy as np


from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray  #for the topic possible actions



import math

roll = pitch = yaw = 0.0
current_angle = 0.0
target = 0
z_angular_vel = 0
kp=-1    #proportional controller for angle
kp_pos = 0.05	  #proportional controller for position
a = 0
current_x = current_y=0 

x_position = 0
y_position=0
z_position=0

state_ = 0

R_position = np.matrix([[0],[0],[0]])

#Rotation respect the z axis

def rotation_matrix(target):
	matrix = np.matrix([[np.cos(target), -np.sin(target), 0 ],
						[np.sin(target), np.cos(target), 0],
						[0, 0, 1]])
	return matrix


general_rotation = rotation_matrix(0)

state_dict_ = {
	0:"go forward",
	1:"turn right",
	2:"turn left",
	3:"go_back"
}

def change_state(state):
	global state_, state_dict_
	if state is not state_:
		print 'Action - [%s] - %s' % (state, state_dict_[state])
		state_ = state


def get_rotation (msg):
	global roll, R_position,pitch, yaw, current_angle, a, x_position, y_position,current_x,current_y,general_rotation
	
	I_position = np.matrix([[msg.pose.pose.position.x],[msg.pose.pose.position.y],[msg.pose.pose.position.z]])  #Position respect inertial reference
	
	R_position = general_rotation*I_position
	x_position = R_position[0,0]
	y_position = R_position[1,0]
	z_position = R_position[2,0]
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	#print yaw
	if a==0:
		current_angle=yaw
		current_x = R_position[0,0]
		current_y = R_position[1,0]
		current_z = R_position[2,0]

	a=1

def take_action(msg):

	linear_x = 0
	angular_z = 0
	state_description = ''

	if(msg.data[0]==True):
		state_description = 'case 2 - turn_left'
		change_state(2)		#turn left	
	elif(msg.data[1]==True):
		state_description = 'case 0 - go_forward'
		change_state(0)		#go forward
	elif(msg.data[2]==True):
		state_description = 'case 1 - turn_right'
		change_state(1)		#turn right
	
		
	else:
		change_state(3)
		state_description = 'unknown case'
	print(state_description)	


def go_forward():
	global target,current_x,kp_pos,command,r,x_position,y_psotion, forw
	target = 1.8
	if(forw == 1):
		current_x = x_position
		forw = 0
	while abs((current_x + target)-x_position)>0.1:
		print(current_x)
	 	#print('xpos',x_position, current_x + target)
	 	#print('error_forward',(current_x + target)-x_position)	
	 	print((current_x+target)-x_position)
		x_linear_vel = kp_pos * ((current_x+target)-x_position)

		command.linear.x = x_linear_vel
		pub.publish(command)
		r.sleep()
	#current_x=current_x+target


def turn_right():
	global target,current_angle,kp,command,r,general_rotation
	target = -90
	target_rad = target*math.pi/180
	while abs((current_angle+target_rad)-yaw)>0.05:
		#print('error_right',(current_angle+target_rad)-yaw)	
		z_angular_vel = kp * ((current_angle+target_rad)-yaw)
		command.angular.z = z_angular_vel
		pub.publish(command)
		r.sleep()
	current_angle = current_angle + target_rad

	general_rotation = general_rotation*rotation_matrix(-target_rad)



def turn_left():
	global target,current_angle,kp,command,r,general_rotation
	target = 90
	target_rad = target*math.pi/180
	while abs((current_angle+target_rad)-yaw)>0.05:
		#print('error_left',(current_angle+target_rad)-yaw)	
		z_angular_vel = kp * ((current_angle+target_rad)-yaw)
		command.angular.z = z_angular_vel
		pub.publish(command)
		r.sleep()
	current_angle = current_angle + target_rad
	general_rotation = general_rotation*rotation_matrix(-target_rad)

def main():
	global pub,sub,sub1,command,r,forw

	rospy.init_node('rotate_robot')

	sub = rospy.Subscriber ('/odom', Odometry, get_rotation)  #taking data form odometry
	sub1 = rospy.Subscriber ('/possible_actions', Int16MultiArray, take_action) 
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	r = rospy.Rate(20)
	command =Twist()

	prueba = 1
	while not rospy.is_shutdown():
	    #quat = quaternion_from_euler (roll, pitch,yaw)
	    #print quat
	
		print(round(R_position[0,0],2),round(R_position[1,0],2),round(R_position[2,0],2))
	
		
		forw = 1
		go_forward()
		turn_left()
		forw = 1
		go_forward()
		turn_right()
		forw = 1
		go_forward()
		# turn_left()
		# forw = 1
		# go_forward()
		# forw = 1
		#go_forward()
		# turn_right()
		# 	turn_left()
		# 	prueba = 0

		# if state_ == 0:
		# 	forw =1
		# 	go_forward()
		# elif state_ == 1:
		# 	turn_right()
		# elif state_ == 2:
		# 	turn_left()
		# elif state_ == 3:
		# 	turn_left()
		# 	turn_left()
		# else:
		# 	rospy.logerr('Unknown state!')



		#target_rad = target*math.pi/180
		
		#z_angular_vel = -kp * ((current_angle+target_rad)-yaw)

		##if((current_angle+target_rad)-yaw)<0.01:
		#	#a=0
		#	#enable =1

		#command.angular.z = z_angular_vel
		#pub.publish(command)
	    ##print(current_angle)
		#print("target={} current:{}".format( current_angle+target_rad,yaw))
		r.sleep()


if __name__ == '__main__':
	main()