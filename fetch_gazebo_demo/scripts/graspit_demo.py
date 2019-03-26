#! /usr/bin/env python

import math
from graspit_commander import GraspitCommander
import rospy
import tf
import geometry_msgs.msg
import numpy as np

if __name__ == '__main__':
	GraspitCommander.clearWorld()
	
	Transformation = np.array([[math.cos(67.5),0,math.sin(67.5),0],[0,1,0,0],[-math.sin(67.5),0,math.cos(67.5),0],[0,0,0,1]])

	Rotation = tf.transformations.quaternion_from_matrix(Transformation)

	object_pose = geometry_msgs.msg.Pose()

	object_pose.position.x = 3

	object_pose.position.y = 3

	object_pose.position.z = 0.1

	object_pose.orientation.x = Rotation[0]

	object_pose.orientation.y = Rotation[1]

	object_pose.orientation.z = Rotation[2]

	object_pose.orientation.w = Rotation[3]

	robot_transformation = np.array([[math.cos(67.5),0,math.sin(67.5),0],[0,1,0,0],[-math.sin(67.5),0,math.cos(67.5),0],[0,0,0,1]])

	robot_rotation = tf.transformations.quaternion_from_matrix(robot_transformation)

	robot_pose = geometry_msgs.msg.Pose()

	robot_pose.position.x = -3

	robot_pose.position.y = -3

	robot_pose.position.z = 0.01

	robot_pose.orientation.x = 0

	robot_pose.orientation.y = 0

	robot_pose.orientation.z = 0

	robot_pose.orientation.w = 0




	GraspitCommander.importObstacle("floor")

	GraspitCommander.importGraspableBody("longBox", pose = object_pose)

	GraspitCommander.importRobot("fetch_gripper", pose = robot_pose)

	GraspitCommander.planGrasps(max_steps = 50000)







	




