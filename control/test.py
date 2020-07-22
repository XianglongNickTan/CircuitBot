# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
import math


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

		# 当运动规划失败后，允许重新规划
		self.arm.allow_replanning(True)

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

		# 控制机械臂先回到初始化位置，手爪打开
		self.arm.set_named_target('Home')
		self.arm.go()
		# self.gripper.set_named_target('Open')
		# self.gripper.go()
		rospy.sleep(1)

		# 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
		# 姿态使用四元数描述，基于base_link坐标系

		self.cont = 10
		self.radius = 0.10

		self.target_pose = PoseStamped()
		self.target_pose.pose.position.z = 0.3
		self.target_pose.pose.orientation.x = 0.14578
		self.target_pose.pose.orientation.y = 0.98924
		self.target_pose.pose.orientation.z = -0.0085346
		self.target_pose.pose.orientation.w = 0.0084136


	def draw_circles(self, xy_center_pos):

		for t in range(self.cont):
			self.target_pose.header.frame_id = self.reference_frame
			self.target_pose.header.stamp = rospy.Time.now()
			self.target_pose.pose.position.x = xy_center_pos[0] + self.radius * math.cos( 2 * math.pi * t / self.cont)
			self.target_pose.pose.position.y = xy_center_pos[1] + self.radius * math.sin( 2 * math.pi * t / self.cont)

			# 设置机器臂当前的状态作为运动初始状态
			self.arm.set_start_state_to_current_state()
			# 设置机械臂终端运动的目标位姿
			self.arm.set_pose_target(self.target_pose, self.end_effector_link)
			# 规划运动路径
			traj = self.arm.plan()
			# 按照规划的运动路径控制机械臂运动
			self.arm.execute(traj)
			# rospy.sleep(1)


		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

if __name__ == "__main__":
	demo = MoveItIkDemo()
	point = [-0.3, -0.3]
	demo.draw_circles(point)