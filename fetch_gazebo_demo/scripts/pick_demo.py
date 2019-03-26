#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib 
import tf.transformations
import fetch_api
import math
from graspit_commander import GraspitCommander
import numpy as np
from gazebo_msgs.srv import GetModelState
from moveit_python import (MoveGroupInterface, PlanningSceneInterface, PickPlaceInterface)

def Graspit():
	GraspitCommander.clearWorld()

	Rotation = tf.transformations.quaternion_from_euler(0, math.pi/2, 0)

	object_pose = geometry_msgs.msg.Pose()

	object_pose.position.x = 3

	object_pose.position.y = 3

	object_pose.position.z = 0.09

	object_pose.orientation.x = Rotation[0]

	object_pose.orientation.y = Rotation[1]

	object_pose.orientation.z = Rotation[2]

	object_pose.orientation.w = Rotation[3]

	'''robot_transformation = np.array([[math.cos(67.5),0,math.sin(67.5),0],[0,1,0,0],[-math.sin(67.5),0,math.cos(67.5),0],[0,0,0,1]])

	robot_rotation = tf.transformations.quaternion_from_matrix(robot_transformation)

	robot_pose = geometry_msgs.msg.Pose()

	robot_pose.position.x = -3

	robot_pose.position.y = -3

	robot_pose.position.z = 0.01

	robot_pose.orientation.x = 0

	robot_pose.orientation.y = 0

	robot_pose.orientation.z = 0

	robot_pose.orientation.w = 0'''




	GraspitCommander.importObstacle("floor")

	GraspitCommander.importGraspableBody("longBox", pose = object_pose)

	GraspitCommander.importRobot("fetch_gripper")

	plan = GraspitCommander.planGrasps(max_steps = 50000)

	return plan

def PoseTransform(po):
	orient = []
	post = []

	orient.append(po.pose.orientation.x)
	orient.append(po.pose.orientation.y)
	orient.append(po.pose.orientation.z)
	orient.append(po.pose.orientation.w)

	post.append(po.pose.position.x)
	post.append(po.pose.position.y)
	post.append(po.pose.position.z)

	obj_r = tf.transformations.quaternion_matrix(np.array(orient))
	obj_t = tf.transformations.translation_matrix(np.array(post))

	Trs = np.dot(obj_t,obj_r)

	return Trs



if __name__ == '__main__':
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node("pick_demo")

	base = fetch_api.Base()
	torso = fetch_api.Torso()
	arm = fetch_api.Arm()
	gripper = fetch_api.Gripper()

	torso.set_height(0.4)
	arm.move_to_joints(fetch_api.ArmJoints.from_list([0, 0, 0, math.pi/2, 0, -math.pi/2, 0]))
	base.go_forward(0.6)
	gripper.open()
	grasping = Graspit()
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

	for grasp in grasping.grasps:
		grasp.pose.position.x -= 3
		grasp.pose.position.y -= 3
		grasp.pose.position.z -= 0.09
		gripper_to_object = PoseTransform(grasp)

		relative_p = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		obj_p = relative_p("demo_cube", "world")
		fetch_p = relative_p("fetch", "world")
		print("Pose Retrieved")

		object_to_world = PoseTransform(obj_p)
		fetch_to_world = PoseTransform(fetch_p)
		world_to_fetch = np.linalg.inv(fetch_to_world)
		gripper_to_world = np.dot(object_to_world, gripper_to_object)
		gripper_to_fetch = np.dot(world_to_fetch, gripper_to_world)

		translate = tf.transformations.translation_from_matrix(gripper_to_fetch)
		rotate = tf.transformations.quaternion_from_matrix(gripper_to_fetch)
		gripper_from_fetch = geometry_msgs.msg.Pose()
		gripper_from_fetch.position.x = translate[0]
		gripper_from_fetch.position.y = translate[1]
		gripper_from_fetch.position.z = translate[2]
		gripper_from_fetch.orientation.x = rotate[0]
		gripper_from_fetch.orientation.y = rotate[1]
		gripper_from_fetch.orientation.z = rotate[2]
		gripper_from_fetch.orientation.w = rotate[3]

		group = moveit_commander.MoveGroupCommander("arm")

		scene = moveit_commander.PlanningSceneInterface()

		robot = moveit_commander.RobotCommander()

		group.set_pose_target(gripper_from_fetch)

		pl1 = group.plan()

		if(len(pl1.joint_trajectory.points)!=0):
			display_trajectory = moveit_msgs.msg.DisplayTrajectory()

			display_trajectory.trajectory_start = robot.get_current_state()
			display_trajectory.trajectory.append(pl1)
			display_trajectory_publisher.publish(display_trajectory)

			group.execute(pl1, wait=True)

			rospy.sleep(5)

	rospy.sleep(2)
	gripper.close(55)
	arm.move_to_joints(fetch_api.ArmJoints.from_list([0,0,0,0,0,0,0]))
	base.go_forward(-0.6)
	arm.move_to_joints(fetch_api.ArmJoints.from_list([0,math.pi/2,0,-math.pi/2,0,0,0]))

	print("Mission Finished")
		
		

