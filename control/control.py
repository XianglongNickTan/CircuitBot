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

		self.line_cont = 80
		self.circle_cont = 40
		self.square_cont = 10
		self.triangle_cont = 10

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
		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = point[0]
		self.target_pose.pose.position.y = point[1]
		self.arm.set_start_state_to_current_state()
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		traj = self.arm.plan()
		self.arm.execute(traj)

	def draw_line(self, xy_init_pos=None):
		waypoints = []
		# if xy_end_pos is None:
		# 	xy_end_pos = [-0.2, -0.25]

		if xy_init_pos is None:
			xy_init_pos = [0.20, -0.52]


		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = xy_init_pos[0]
		self.target_pose.pose.position.y = xy_init_pos[1]
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
			wpose.pose.position.x = (xy_init_pos[0] - 0.46 / self.line_cont * (t + 1))
			wpose.pose.position.y = xy_init_pos[1]
			waypoints.append(copy.deepcopy(wpose.pose))

		(plan, fraction) = self.arm.compute_cartesian_path(
			waypoints,
			0.005,             # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.0
		)
		arduino_motor.write('2')
		self.arm.execute(plan)
		arduino_motor.write('0')
		self.arm.stop()

		# self.disable_upright_path_constraints()

		rospy.sleep(10)

	def draw_square(self, xy_center_pos, length=0.05, orientation=0):
		waypoints = []
		# rospy.rostime.wallsleep(0.05)

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = xy_center_pos[0] + length
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
				wpose.pose.position.x = (xy_center_pos[0] + length - 2 * length / self.square_cont * (t + 1))
				wpose.pose.position.y = xy_center_pos[1] - length
				waypoints.append(copy.deepcopy(wpose.pose))
			for t in range(self.square_cont):
				wpose.pose.position.x = xy_center_pos[0] - length
				wpose.pose.position.y = (xy_center_pos[1] - length + 2 * length / self.square_cont * (t + 1))
				waypoints.append(copy.deepcopy(wpose.pose))
			for t in range(self.square_cont):
				wpose.pose.position.x = (xy_center_pos[0] - length + 2 * length / self.square_cont * (t + 1))
				wpose.pose.position.y = xy_center_pos[1] + length
				waypoints.append(copy.deepcopy(wpose.pose))
			for t in range(self.square_cont):
				wpose.pose.position.x = xy_center_pos[0] + length
				wpose.pose.position.y = (xy_center_pos[1] + length - 2 * length / self.square_cont * (t + 1))
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

	def draw_triangle(self, xy_center_pos, length=0.05, orientation=0):
		waypoints = []
		# rospy.rostime.wallsleep(0.05)

		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = xy_center_pos[0] + length
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

		point_1 = [xy_center_pos[0], xy_center_pos[1] - length * 2 * np.sqrt(3) / 3]
		point_2 = [xy_center_pos[0] - length, xy_center_pos[1] + length * np.sqrt(3) / 3]
		point_3 = [xy_center_pos[0] + length, xy_center_pos[1] + length * np.sqrt(3) / 3]

		for _ in range (3):
			wpose.pose.position.x = point_1[0]
			wpose.pose.position.y = point_1[1]
			waypoints.append(copy.deepcopy(wpose.pose))
			wpose.pose.position.x = (point_1[0] + point_2[0]) / 2
			wpose.pose.position.y = (point_1[1] + point_2[1]) / 2
			waypoints.append(copy.deepcopy(wpose.pose))
			wpose.pose.position.x = point_2[0]
			wpose.pose.position.y = point_2[1]
			waypoints.append(copy.deepcopy(wpose.pose))
			wpose.pose.position.x = (point_2[0] + point_3[0]) / 2
			wpose.pose.position.y = (point_2[1] + point_3[1]) / 2
			waypoints.append(copy.deepcopy(wpose.pose))
			wpose.pose.position.x = point_3[0]
			wpose.pose.position.y = point_3[1]
			waypoints.append(copy.deepcopy(wpose.pose))
			wpose.pose.position.x = (point_3[0] + point_1[0]) / 2
			wpose.pose.position.y = (point_3[1] + point_1[1]) / 2
			waypoints.append(copy.deepcopy(wpose.pose))
			wpose.pose.position.x = point_1[0]
			wpose.pose.position.y = point_1[1]
			waypoints.append(copy.deepcopy(wpose.pose))


		# self.init_upright_path_constraints(wpose)
		# self.enable_upright_path_constraints()

		(plan, fraction) = self.arm.compute_cartesian_path(
			waypoints,
			# 0.0005,             # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.001,  # SUPER IMPORTANT PARAMETER FOR VELOCITY CONTROL !!!!!
			0.0
		)

		arduino_motor.write('1')
		self.arm.execute(plan)
		arduino_motor.write('0')
		self.arm.stop()
		rospy.sleep(2)


	def draw_circle(self, xy_center_pos, radius = 0.06):
		waypoints = []
		# rospy.rostime.wallsleep(0.05)

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

def test(xy_center_pos):
	######################## For testing ########################

	voltage = xy_center_pos[0] - xy_center_pos[1]**2 - xy_center_pos[2]**2 + xy_center_pos[3] + xy_center_pos[4]**2 - xy_center_pos[5]
	voltage_file = open("voltage.txt", "w")
	# print ("-------------------------")
	# print ("file name: ", voltage_file.name)
	# print ("-------------------------")
	voltage_file.write(str(voltage))
	voltage_file.close()
	print("voltage:", voltage)

######################## For testing ########################


if __name__ == "__main__":
	demo = MoveItIkDemo()
	#
	# demo.draw_triangle([0, -0.52])
	demo.draw_square([-0.15, -0.52])
	demo.draw_circle([])

	# ###########################################################################
	# points_file = open("next.txt", "r")
	# points_str = points_file.read()
	# print(points_str)
	# points_list = points_str.split()
	# points_file.close()
	#
	# #################################################################
	#
	# center1 = [float(points_list[0]), float(points_list[3])]
	# center2 = [float(points_list[1]), float(points_list[4])]
	# center3 = [float(points_list[2]), float(points_list[5])]
	# demo.draw_circle(center1)
	# demo.draw_circle(center2)
	# demo.draw_circle(center3)

	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)
