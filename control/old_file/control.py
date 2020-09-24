# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
import math
import numpy as np
import copy
import serial
import os


arduino_motor = serial.Serial('/dev/ttyACM0', 38400)
# arduino_voltage = serial.Serial('/dev/ttyACM1', 9600, timeout=0.5)



class MoveItIkDemo:
	def __init__(self):
		# 初始化move_group的API
		moveit_commander.roscpp_initialize(sys.argv)

		# 初始化ROS节点
		rospy.init_node('real_jaco_moveit_fixed_grasp')

		# 初始化需要使用move group控制的机械臂中的self.arm group
		self.arm = moveit_commander.MoveGroupCommander('arm')

		# 初始化需要使用move group控制的机械臂中的gripper group
		# self.gripper = moveit_commander.MoveGroupCommander('gripper')

		# 获取终端link的名称
		self.end_effector_link = self.arm.get_end_effector_link()

		# 设置目标位置所使用的参考坐标系
		self.reference_frame = 'world'
		self.arm.set_pose_reference_frame(self.reference_frame)


		# 当运动规划失败后，不允许重新规划
		self.arm.allow_replanning(False)

		# 设置位置(单位：米)和姿态（单位：弧度）的允许误差
		self.arm.set_goal_position_tolerance(0.001)
		self.arm.set_goal_orientation_tolerance(0.001)
		# self.gripper.set_goal_joint_tolerance(0.001)

		# 设置允许的最大速度和加速度
		self.arm.set_max_acceleration_scaling_factor(0.5)
		self.arm.set_max_velocity_scaling_factor(0.5)

		# 初始化场景对象
		scene = PlanningSceneInterface()
		rospy.sleep(1)

		# ws = [-0.3, -0.6, 0.01, 0.2, -0.3, 0.1]
		# self.arm.set_workspace(ws)

		# # 控制机械臂先回到初始化位置，手爪打开
		# self.arm.set_named_target('Home')
		# self.arm.go()
		# # self.gripper.set_named_target('Open')
		# # self.gripper.go()
		# rospy.sleep(1)

		self.line_cont = 10
		self.circle_cont = 40
		self.square_cont = 10
		self.triangle_cont = 10

		self.x_offset = -4
		self.y_offset = -53

		self.target_pose = PoseStamped()
		self.target_pose.header.frame_id = self.reference_frame
		self.target_pose.pose.position.z = 0.03

		self.target_pose.pose.orientation.x = 0.14578
		self.target_pose.pose.orientation.y = 0.98924
		self.target_pose.pose.orientation.z = -0.0085346
		self.target_pose.pose.orientation.w = 0.0084136

	# def init_upright_path_constraints(self, pose):
	#
	# 	self.upright_constraints = Constraints()
	# 	self.upright_constraints.name = "upright"
	# 	orientation_constraint = OrientationConstraint()
	# 	orientation_constraint.header = pose.header
	# 	orientation_constraint.link_name = self.end_effector_link
	# 	orientation_constraint.orientation = pose.pose.orientation
	# 	orientation_constraint.absolute_x_axis_tolerance = 0.2
	# 	orientation_constraint.absolute_y_axis_tolerance = 0.2
	# 	orientation_constraint.absolute_z_axis_tolerance = 0.2
	# 	# orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
	# 	orientation_constraint.weight = 1
	#
	# 	self.upright_constraints.orientation_constraints.append(orientation_constraint)
	#
	# def enable_upright_path_constraints(self):
	# 	self.arm.set_path_constraints(self.upright_constraints)
	#
	# def disable_upright_path_constraints(self):
	# 	self.arm.set_path_constraints(None)

	def move_to(self, point):
		point[0] = (point[0] + self.x_offset) * 0.01
		point[1] = (point[1] + self.y_offset) * 0.01
		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = point[0]
		self.target_pose.pose.position.y = point[1]
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)

	def draw_line(self, xy_center_pos, length=0.05, horizontal=True):
		waypoints = []
		# if xy_end_pos is None:
		# 	xy_end_pos = [-0.2, -0.25]

		xy_center_pos[0] = (xy_center_pos[0] + self.x_offset) * 0.01
		xy_center_pos[1] = (xy_center_pos[1] + self.y_offset) * 0.01

		if horizontal:
			self.target_pose.header.stamp = rospy.Time.now()
			self.target_pose.pose.position.x = xy_center_pos[0] + length
			self.target_pose.pose.position.y = xy_center_pos[1]
			self.arm.set_start_state_to_current_state()
			self.arm.set_pose_target(self.target_pose, self.end_effector_link)
			traj = self.arm.plan()
			self.arm.execute(traj)

			wpose = self.arm.get_current_pose()
			wpose.pose.orientation.x = 0.14578
			wpose.pose.orientation.y = 0.98924
			wpose.pose.orientation.z = -0.00853
			wpose.pose.orientation.w = 0.00841
			wpose.pose.position.z = 0.03

			# self.init_upright_path_constraints(wpose)
			# self.enable_upright_path_constraints()

			for t in range(self.line_cont):
				wpose.pose.position.x = (xy_center_pos[0] + length - 2 * length / self.line_cont * (t + 1))
				wpose.pose.position.y = xy_center_pos[1]
				waypoints.append(copy.deepcopy(wpose.pose))

			(plan, fraction) = self.arm.compute_cartesian_path(
				waypoints,
				0.001,  # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
				0.0
			)

		else:
			self.target_pose.header.stamp = rospy.Time.now()
			self.target_pose.pose.position.x = xy_center_pos[0]
			self.target_pose.pose.position.y = xy_center_pos[1] - length
			self.arm.set_start_state_to_current_state()
			self.arm.set_pose_target(self.target_pose, self.end_effector_link)
			traj = self.arm.plan()
			self.arm.execute(traj)

			wpose = self.arm.get_current_pose()
			wpose.pose.orientation.x = 0.14578
			wpose.pose.orientation.y = 0.98924
			wpose.pose.orientation.z = -0.00853
			wpose.pose.orientation.w = 0.00841
			wpose.pose.position.z = 0.03

			# self.init_upright_path_constraints(wpose)
			# self.enable_upright_path_constraints()

			for t in range(self.line_cont):
				wpose.pose.position.x = xy_center_pos[0]
				wpose.pose.position.y = (xy_center_pos[1] - length + 2 * length / self.line_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))


			(plan, fraction) = self.arm.compute_cartesian_path(
				waypoints,
				0.1,             # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
				0.0
			)

		arduino_motor.write('2')
		self.arm.execute(plan)
		arduino_motor.write('0')
		self.arm.stop()

		# self.disable_upright_path_constraints()

		rospy.sleep(10)

	def draw_cross(self, xy_center_pos, length=0.05):
		waypoints = []
		# if xy_end_pos is None:
		# 	xy_end_pos = [-0.2, -0.25]

		xy_center_pos[0] = (xy_center_pos[0] + self.x_offset) * 0.01
		xy_center_pos[1] = (xy_center_pos[1] + self.y_offset) * 0.01



		point_1 = [xy_center_pos[0] + length, xy_center_pos[1] - length]
		point_2 = [xy_center_pos[0] - length, xy_center_pos[1] + length]
		point_3 = [xy_center_pos[0] - length, xy_center_pos[1] - length]
		point_4 = [xy_center_pos[0] + length, xy_center_pos[1] + length]

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = point_1[0]
		self.target_pose.pose.position.y = point_1[1]
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)

		wpose = self.arm.get_current_pose()
		wpose.pose.orientation.x = 0.14578
		wpose.pose.orientation.y = 0.98924
		wpose.pose.orientation.z = -0.00853
		wpose.pose.orientation.w = 0.00841
		wpose.pose.position.z = 0.03

		# self.init_upright_path_constraints(wpose)
		# self.enable_upright_path_constraints()

		for t in range(self.line_cont):
			wpose.pose.position.x = ((self.line_cont - t) * point_1[0] + t * point_2[0]) / self.line_cont
			wpose.pose.position.y = ((self.line_cont - t) * point_1[1] + t * point_2[1]) / self.line_cont
			waypoints.append(copy.deepcopy(wpose.pose))

		(plan, fraction) = self.arm.compute_cartesian_path(
			waypoints,
			0.01,             # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.0
		)

		arduino_motor.write('2')
		self.arm.execute(plan)
		arduino_motor.write('0')
		self.arm.stop()

		waypoints = []


		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = point_3[0]
		self.target_pose.pose.position.y = point_3[1]
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)

		wpose = self.arm.get_current_pose()
		wpose.pose.orientation.x = 0.14578
		wpose.pose.orientation.y = 0.98924
		wpose.pose.orientation.z = -0.00853
		wpose.pose.orientation.w = 0.00841
		wpose.pose.position.z = 0.03


		for t in range(self.line_cont):
			wpose.pose.position.x = ((self.line_cont - t) * point_3[0] + t * point_4[0]) / self.line_cont
			wpose.pose.position.y = ((self.line_cont - t) * point_3[1] + t * point_4[1]) / self.line_cont
			waypoints.append(copy.deepcopy(wpose.pose))

		(plan, fraction) = self.arm.compute_cartesian_path(
			waypoints,
			0.01,             # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.0
		)

		arduino_motor.write('2')
		self.arm.execute(plan)
		arduino_motor.write('0')
		self.arm.stop()


		# self.draw_line(xy_center_pos=xy_center_pos, length=length, horizontal=True)
		# self.draw_line(xy_center_pos=xy_center_pos, length=length, horizontal=False)

	def draw_diamond(self, xy_center_pos, length=0.05):
		waypoints = []
		# rospy.rostime.wallsleep(0.05)

		xy_center_pos[0] = (xy_center_pos[0] + self.x_offset) * 0.01
		xy_center_pos[1] = (xy_center_pos[1] + self.y_offset) * 0.01

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = xy_center_pos[0]
		self.target_pose.pose.position.y = xy_center_pos[1] - length
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)
		rospy.sleep(2)


		wpose = self.arm.get_current_pose()
		wpose.pose.orientation.x = 0.14578
		wpose.pose.orientation.y = 0.98924
		wpose.pose.orientation.z = -0.00853
		wpose.pose.orientation.w = 0.00841
		wpose.pose.position.z = 0.03

		for _ in range (3):
			for t in range(self.square_cont):
				wpose.pose.position.x = (xy_center_pos[0] + length / self.square_cont * (t + 1))
				wpose.pose.position.y = (xy_center_pos[1] - length + length / self.square_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))
			for t in range(self.square_cont):
				wpose.pose.position.x = (xy_center_pos[0] + length - length / self.square_cont * (t + 1))
				wpose.pose.position.y = (xy_center_pos[1] + length / self.square_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))
			for t in range(self.square_cont):
				wpose.pose.position.x = (xy_center_pos[0] - length / self.square_cont * (t + 1))
				wpose.pose.position.y = (xy_center_pos[1] + length - length / self.square_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))
			for t in range(self.square_cont):
				wpose.pose.position.x = (xy_center_pos[0] - length + length / self.square_cont * (t + 1))
				wpose.pose.position.y = (xy_center_pos[1] - length / self.square_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))

		# self.init_upright_path_constraints(wpose)
		# self.enable_upright_path_constraints()

		(plan, fraction) = self.arm.compute_cartesian_path(
			waypoints,
			# 0.0005,             # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.01,  # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.0
		)

		arduino_motor.write('1')
		self.arm.execute(plan)
		arduino_motor.write('0')
		self.arm.stop()
		rospy.sleep(2)

	def draw_triangle(self, xy_center_pos, length=0.05):
		waypoints = []
		# rospy.rostime.wallsleep(0.05)

		xy_center_pos[0] = (xy_center_pos[0] + self.x_offset) * 0.01
		xy_center_pos[1] = (xy_center_pos[1] + self.y_offset) * 0.01

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = xy_center_pos[0]
		self.target_pose.pose.position.y = xy_center_pos[1] - length * np.sqrt(3) / 2
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)
		rospy.sleep(2)


		wpose = self.arm.get_current_pose()
		wpose.pose.orientation.x = 0.14578
		wpose.pose.orientation.y = 0.98924
		wpose.pose.orientation.z = -0.00853
		wpose.pose.orientation.w = 0.00841
		wpose.pose.position.z = 0.03

		point_1 = [xy_center_pos[0], xy_center_pos[1] - 2 * length * np.sqrt(3) / 3]
		point_2 = [xy_center_pos[0] + length, xy_center_pos[1] + length * np.sqrt(3) / 3]
		point_3 = [xy_center_pos[0] - length, xy_center_pos[1] + length * np.sqrt(3) / 3]

		for _ in range(3):
			for t in range(self.triangle_cont):
				wpose.pose.position.x = (point_1[0] + length / self.triangle_cont * (t + 1))
				wpose.pose.position.y = (point_1[1] + length * np.sqrt(3) / self.triangle_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))

			for t in range(self.triangle_cont):
				wpose.pose.position.x = (point_2[0] - 2 * length / self.triangle_cont * (t + 1))
				wpose.pose.position.y = point_2[1]
				waypoints.append(copy.deepcopy(wpose.pose))

			for t in range(self.triangle_cont):
				wpose.pose.position.x = (point_3[0] + length / self.triangle_cont * (t + 1))
				wpose.pose.position.y = (point_3[1] - length * np.sqrt(3) / self.triangle_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))



		# self.init_upright_path_constraints(wpose)
		# self.enable_upright_path_constraints()

		(plan, fraction) = self.arm.compute_cartesian_path(
			waypoints,
			0.01,  # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.0
		)

		arduino_motor.write('1')
		self.arm.execute(plan)
		arduino_motor.write('0')
		self.arm.stop()
		rospy.sleep(2)


	def draw_circle(self, xy_center_pos, radius = 0.05):
		waypoints = []
		# rospy.rostime.wallsleep(0.05)

		xy_center_pos[0] = (xy_center_pos[0] + self.x_offset) * 0.01
		xy_center_pos[1] = (xy_center_pos[1] + self.y_offset) * 0.01

		# print(xy_center_pos)

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = xy_center_pos[0] + radius
		self.target_pose.pose.position.y = xy_center_pos[1]
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)
		rospy.sleep(2)


		wpose = self.arm.get_current_pose()
		wpose.pose.orientation.x = 0.14578
		wpose.pose.orientation.y = 0.98924
		wpose.pose.orientation.z = -0.00853
		wpose.pose.orientation.w = 0.00841
		wpose.pose.position.z = 0.03

		for _ in range (2):
			for t in range(self.circle_cont):
				wpose.pose.position.x = xy_center_pos[0] + radius * np.cos(2 * np.pi * (t+1) / self.circle_cont)
				wpose.pose.position.y = xy_center_pos[1] + radius * np.sin(2 * np.pi * (t+1) / self.circle_cont)
				waypoints.append(copy.deepcopy(wpose.pose))

		# self.init_upright_path_constraints(wpose)
		# self.enable_upright_path_constraints()

		(plan, fraction) = self.arm.compute_cartesian_path(
			waypoints,
			# 0.0005,             # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.01,  # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.0
		)

		arduino_motor.write('1')
		self.arm.execute(plan)
		arduino_motor.write('0')
		self.arm.stop()
		rospy.sleep(2)

		# self.disable_upright_path_constraints()
		# rospy.sleep(2)


		# moveit_commander.roscpp_shutdown()
		# moveit_commander.os._exit(0)

	def draw_shape(self, shape, xy_center_pos):
		if shape == 0:
			self.draw_line(xy_center_pos=xy_center_pos)

		elif shape == 1:
			self.draw_line(xy_center_pos=xy_center_pos, horizontal=False)

		elif shape == 2:
			self.draw_cross(xy_center_pos=xy_center_pos)

		elif shape == 3:
			self.draw_circle(xy_center_pos=xy_center_pos)

		elif shape == 4:
			self.draw_diamond(xy_center_pos=xy_center_pos)

######################## For testing ########################


if __name__ == "__main__":
	demo = MoveItIkDemo()


	# ###########################################################################
	points_file = open("next.txt", "r")
	points_str = points_file.read()
	print(points_str)
	points_list = points_str.split()
	points_file.close()

	for i in range(5):
		shape_index = points_list[i]
		print(shape_index)
		center = [float(points_list[5 + i]), float(points_list[10 + i])]
		print(center)
		demo.draw_shape(shape_index, center)

	# demo.draw_shape(0, [0, 0])


	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)
