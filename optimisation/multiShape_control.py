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
import random
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
		self.arm.set_max_acceleration_scaling_factor(0.2)
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

		self.line_cont = 100
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
		point[1] = (point[1] + self.x_offset) * 0.01

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = point[0]
		self.target_pose.pose.position.y = point[1]
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)

	def draw_line(self, xy_start_pos, xy_end_pos, vel=0.01, times=2):
		# waypoints = []
		#
		# xy_start_pos[0] = (xy_start_pos[0] + self.x_offset) * 0.01
		# xy_start_pos[1] = (xy_start_pos[1] + self.y_offset) * 0.01
		# xy_end_pos[0] = (xy_end_pos[0] + self.x_offset) * 0.01
		# xy_end_pos[1] = (xy_end_pos[1] + self.y_offset) * 0.01
		#
		# stay_point = [0, -5]
		# stay_point[0] = (stay_point[0] + self.x_offset) * 0.01
		# stay_point[1] = (stay_point[1] + self.y_offset) * 0.01
		#
		# self.target_pose.header.stamp = rospy.Time.now()
		# self.target_pose.pose.position.x = stay_point[0]
		# self.target_pose.pose.position.y = stay_point[1]
		#
		# self.arm.set_start_state_to_current_state()
		# self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		# traj = self.arm.plan()
		# self.arm.execute(traj)
		#
		# self.target_pose.header.stamp = rospy.Time.now()
		# self.target_pose.pose.position.x = xy_start_pos[0]
		# self.target_pose.pose.position.y = xy_start_pos[1]
		#
		# self.arm.set_start_state_to_current_state()
		# self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		# traj = self.arm.plan()
		# self.arm.execute(traj)
		#
		#
		# wpose = self.arm.get_current_pose()
		# wpose.pose.orientation.x = 0.14578
		# wpose.pose.orientation.y = 0.98924
		# wpose.pose.orientation.z = -0.00853
		# wpose.pose.orientation.w = 0.00841
		# wpose.pose.position.z = 0.03
		#
		# # self.init_upright_path_constraints(wpose)
		# # self.enable_upright_path_constraints()
		#
		#
		# # for _ in range(2):
		# for t in range(self.line_cont):
		# 	wpose.pose.position.x = ((self.line_cont - t) * xy_start_pos[0] + t * xy_end_pos[0]) / self.line_cont
		# 	wpose.pose.position.y = ((self.line_cont - t) * xy_start_pos[1] + t * xy_end_pos[1]) / self.line_cont
		# 	waypoints.append(copy.deepcopy(wpose.pose))
		#
		# if times == 2:
		# 	for t in range(self.line_cont):
		# 		wpose.pose.position.x = (t * xy_start_pos[0] + (self.line_cont - t) * xy_end_pos[0]) / self.line_cont
		# 		wpose.pose.position.y = (t * xy_start_pos[1] + (self.line_cont - t) * xy_end_pos[1]) / self.line_cont
		# 		waypoints.append(copy.deepcopy(wpose.pose))
		#
		# (plan, fraction) = self.arm.compute_cartesian_path(
		# 	waypoints,
		# 	vel,             # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
		# 	0.0
		# )
		#
		# arduino_motor.write('2')
		# self.arm.execute(plan)
		# arduino_motor.write('0')
		# self.arm.stop()
		#
		# # self.disable_upright_path_constraints()
		#
		# rospy.sleep(2)

		xy_start_pos[0] = (xy_start_pos[0] + self.x_offset) * 0.01
		xy_start_pos[1] = (xy_start_pos[1] + self.y_offset) * 0.01
		xy_end_pos[0] = (xy_end_pos[0] + self.x_offset) * 0.01
		xy_end_pos[1] = (xy_end_pos[1] + self.y_offset) * 0.01

		stay_point = [0, -5]
		stay_point[0] = (stay_point[0] + self.x_offset) * 0.01
		stay_point[1] = (stay_point[1] + self.y_offset) * 0.01

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = stay_point[0]
		self.target_pose.pose.position.y = stay_point[1]

		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = xy_start_pos[0]
		self.target_pose.pose.position.y = xy_start_pos[1]

		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = xy_end_pos[0]
		self.target_pose.pose.position.y = xy_end_pos[1]

		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()

		arduino_motor.write('2')
		self.arm.execute(traj)
		arduino_motor.write('0')
		self.arm.stop()

		# self.disable_upright_path_constraints()

		rospy.sleep(2)

	def draw_cross(self, xy_center_pos, length=10):
		waypoints = []
		# rospy.rostime.wallsleep(0.05)

		xy_start_pos = [0,0]
		xy_end_pos = [0,0]

		# xy_center_pos[0] = (xy_center_pos[0] + self.x_offset) * 0.01
		# xy_center_pos[1] = (xy_center_pos[1] + self.y_offset) * 0.01

		xy_start_pos[0] = xy_center_pos[0] - length / 2
		xy_start_pos[1] = xy_center_pos[1] - length / 2
		xy_end_pos[0] = xy_center_pos[0] + length / 2
		xy_end_pos[1] = xy_center_pos[1] + length / 2
		
		
		# self.move_to(xy_start_pos)

		self.draw_line(xy_start_pos, xy_end_pos, times = 1)


	
		xy_start_pos[0] = xy_center_pos[0] + length / 2
		xy_start_pos[1] = xy_center_pos[1] - length / 2
		xy_end_pos[0] = xy_center_pos[0] - length / 2
		xy_end_pos[1] = xy_center_pos[1] + length / 2

		# self.move_to(xy_start_pos)

		self.draw_line(xy_start_pos, xy_end_pos, times = 1)



	def draw_square(self, xy_center_pos, length=0.05*1.2, orientation=0):
		waypoints = []
		# rospy.rostime.wallsleep(0.05)

		xy_center_pos[0] = (xy_center_pos[0] + self.x_offset) * 0.01
		xy_center_pos[1] = (xy_center_pos[1] + self.y_offset) * 0.01

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = xy_center_pos[0] + length
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

		for _ in range (3):
			for t in range(self.square_cont):
				wpose.pose.position.x = (xy_center_pos[0] + length - length / self.square_cont * (t + 1))
				wpose.pose.position.y = (xy_center_pos[1] - length / self.square_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))
			for t in range(self.square_cont):
				wpose.pose.position.x = (xy_center_pos[0] - length / self.square_cont * (t + 1))
				wpose.pose.position.y = (xy_center_pos[1] - length + length / self.square_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))
			for t in range(self.square_cont):
				wpose.pose.position.x = (xy_center_pos[0] - length + length / self.square_cont * (t + 1))
				wpose.pose.position.y = (xy_center_pos[1] + length / self.square_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))
			for t in range(self.square_cont):
				wpose.pose.position.x = (xy_center_pos[0] + length / self.square_cont * (t + 1))
				wpose.pose.position.y = (xy_center_pos[1] + length - length / self.square_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))

		# self.init_upright_path_constraints(wpose)
		# self.enable_upright_path_constraints()

		(plan, fraction) = self.arm.compute_cartesian_path(
			waypoints,
			# 0.0005,             # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.007,  # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.0
		)

		arduino_motor.write('1')
		self.arm.execute(plan)
		arduino_motor.write('0')
		self.arm.stop()
		rospy.sleep(2)

	def draw_triangle(self, xy_center_pos, length=0.09*np.sqrt(3)/3, orientation=0):
		waypoints = []
		# rospy.rostime.wallsleep(0.05)

		xy_center_pos[0] = (xy_center_pos[0] + self.x_offset) * 0.01
		xy_center_pos[1] = (xy_center_pos[1] + self.y_offset) * 0.01

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = xy_center_pos[0] + length * np.sqrt(3) / 3
		self.target_pose.pose.position.y = xy_center_pos[1] - length
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)
		# rospy.sleep(2)


		wpose = self.arm.get_current_pose()
		wpose.pose.orientation.x = 0.14578
		wpose.pose.orientation.y = 0.98924
		wpose.pose.orientation.z = -0.00853
		wpose.pose.orientation.w = 0.00841
		wpose.pose.position.z = 0.03

		point_1 = [xy_center_pos[0] + length * np.sqrt(3) / 3, xy_center_pos[1] - length]
		point_2 = [xy_center_pos[0] + length * np.sqrt(3) / 3, xy_center_pos[1] + length]
		point_3 = [xy_center_pos[0] - length * 2 * np.sqrt(3) / 3, xy_center_pos[1]]


		for t in range(self.triangle_cont):
			wpose.pose.position.x = point_1[0]
			wpose.pose.position.y = (point_1[1] + 2 * length / self.triangle_cont * (t + 1))
			waypoints.append(copy.deepcopy(wpose.pose))

		for t in range(self.triangle_cont):
			wpose.pose.position.x = (point_2[0] - length * np.sqrt(3) / self.triangle_cont * (t + 1))
			wpose.pose.position.y = (point_2[1] - length / self.triangle_cont * (t + 1))
			waypoints.append(copy.deepcopy(wpose.pose))

		for t in range(self.triangle_cont):
			wpose.pose.position.x = (point_3[0] + length * np.sqrt(3) / self.triangle_cont * (t + 1))
			wpose.pose.position.y = (point_3[1] - length / self.triangle_cont * (t + 1))
			waypoints.append(copy.deepcopy(wpose.pose))

		for t in range(self.triangle_cont):
			wpose.pose.position.x = (point_1[0] - length * np.sqrt(3) / self.triangle_cont * (t + 1))
			wpose.pose.position.y = (point_1[1] + length / self.triangle_cont * (t + 1))
			waypoints.append(copy.deepcopy(wpose.pose))

		for t in range(self.triangle_cont):
			wpose.pose.position.x = (point_3[0] + length * np.sqrt(3) / self.triangle_cont * (t + 1))
			wpose.pose.position.y = (point_3[1] + length / self.triangle_cont * (t + 1))
			waypoints.append(copy.deepcopy(wpose.pose))

	


		# # self.init_upright_path_constraints(wpose)
		# # self.enable_upright_path_constraints()

		(plan, fraction) = self.arm.compute_cartesian_path(
			waypoints,
			# 0.0005,             # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.01,  # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.0
		)

		arduino_motor.write('2')
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


def test(xy_center_pos):
	######################## For testing ########################

	voltage = xy_center_pos[0] - xy_center_pos[1]**2 - xy_center_pos[2]**2 + xy_center_pos[3] + xy_center_pos[4]**2 - xy_center_pos[5]
	voltage_file = open("../control/voltage.txt", "w")
	# print ("-------------------------")
	# print ("file name: ", voltage_file.name)
	# print ("-------------------------")
	voltage_file.write(str(voltage))
	voltage_file.close()
	print("voltage:", voltage)

######################## For testing ########################


if __name__ == "__main__":
	demo = MoveItIkDemo()

	###########################################################################
	points_file = open("next.txt", "r")
	points_str = points_file.read()
	points_list = points_str.split()
	points_file.close()

	#################################################################
	c1 = random.uniform(4, 8)
	c2 = random.uniform(-2, 2)
	c3 = random.uniform(-8, -4)

	# points_list = ["1.0", "1.0", "1.0", "1.0", "1.0", "6.0", "0.0", "-6.0"]
	# points_list = ["0.0", "0.0", "0.0", "0.0", "0.0", str(c1), str(c2), str(c3)]
	# print(points_list)

	shape1 = points_list[0]
	shape2 = points_list[1]
	shape3 = points_list[2]
	shape4 = points_list[3]
	shape5 = points_list[4]
	
	shape_list = [shape1, shape2, shape3, shape4, shape5]

	# print(shape_list)

	center1 = [14, 0.0]
	center2 = [float(points_list[5]), 0.0]
	center3 = [float(points_list[6]), 0.0]
	center4 = [float(points_list[7]), 0.0]
	center5 = [-14, 0.0]

	# center1 = [float(points_list[5]), float(points_list[6])]
	# center2 = [float(points_list[7]), float(points_list[8])]
	# center3 = [float(points_list[9]), float(points_list[10])]
	# center4 = [float(points_list[11]), float(points_list[12])]
	# center5 = [float(points_list[13]), float(points_list[14])]
	
	center_list = [center1, center2, center3, center4, center5]

	for i in range(len(shape_list)):
		if shape_list[i] == '0.0':
			start = [center_list[i][0] + 4.5, center_list[i][1]]
			end = [center_list[i][0] - 4.5, center_list[i][1]]
			demo.draw_line(start, end, 0.01, times=1)
		elif shape_list[i] == '1.0':
			demo.draw_circle(center_list[i])
		elif shape_list[i] == '2.0':
			demo.draw_cross(center_list[i])
		elif shape_list[i] == '3.0':
			demo.draw_triangle(center_list[i])
		elif shape_list[i] == '4.0':
			demo.draw_square(center_list[i])
		elif shape_list[i] == '5.0':
			demo.draw_square(center_list[i])
		else:
			raise NotImplementedError


	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)
