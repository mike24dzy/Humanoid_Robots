#! /usr/bin/env python

import math
import fetch_api
import rospy

if __name__ == '__main__':
	rospy.init_node('motion_demo')
	base = fetch_api.Base()
	base.go_forward(1.5)
	print('Move Base Forward 1.5 meters')
	head = fetch_api.Head()
	head.pan_tilt(0, math.pi/4)
	head.pan_tilt(0, -math.pi/2)
	print('Head looks down 45 degrees')
	print('Head looks up 90 degrees')
	head.pan_tilt(0, 0)
	head.pan_tilt(math.pi/2, 0)
	head.pan_tilt(0, 0)
	print('Head looks left 90 degrees')
	head.pan_tilt(-math.pi/2, 0)
	print('Head looks right 90 degrees')
	head.pan_tilt(0, 0)
	print('Head looks forward')
	torso = fetch_api.Torso()
	torso.set_height(0.4)
	print('Raise torso to the maximum')
	arm = fetch_api.Arm()
	arm.move_to_joints(fetch_api.ArmJoints.from_list([0,0,0,0,0,0,0]))
	print('Move arm to joint values [0,0,0,0,0,0,0]')
	gripper = fetch_api.Gripper()
	gripper.close()
	print('Close the gripper')
	gripper.open()
	print('Open the gripper')
	
	
	
	
	
	
	
