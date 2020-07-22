# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
import math
import numpy as np
from ar_track_alvar_msgs.msg  import  AlvarMarkers
import tf2_ros



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
		# self.radius = 0.10

		self.target_pose = PoseStamped()
		self.target_pose.pose.position.z = 0.3
		self.target_pose.pose.orientation.x = 0.14578
		self.target_pose.pose.orientation.y = 0.98924
		self.target_pose.pose.orientation.z = -0.0085346
		self.target_pose.pose.orientation.w = 0.0084136

		# Use ARtag to locate the circuit
		self.tfBuffer1 = tf2_ros.Buffer()
        # self.tfListener1 = tf2_ros.TransformListener(tfBuffer1)
		self.coord_1 = []

		self.artag = None

		if self.artag != None:
			try:
				self.coord_1 = tfBuffer1.lookup_transform('world', self.artag, rospy.Time())
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				pass


	def trans(position):
		return [position[0] + self.coord1.transform.translation.x, position[1] + self.coord1.transform.translation.y]



	def go_to(self, end_pos):
		self.target_pose.header.frame_id = self.reference_frame
		self.target_pose.header.stamp = rospy.Time.now()
		self.target_pose.pose.position.x = end_pos[0]
		self.target_pose.pose.position.y = end_pos[1]

		# 设置机器臂当前的状态作为运动初始状态
		self.arm.set_start_state_to_current_state()
		# 设置机械臂终端运动的目标位姿
		self.arm.set_pose_target(self.target_pose, self.end_effector_link)
		# 规划运动路径
		traj = self.arm.plan()

		return traj




    def draw_line(self, initial_pos, end_pos):
		traj_1 = self.go_to(initial_pos)
		traj_2 = self.go_to(end_pos)

		# 按照规划的运动路径控制机械臂运动
		self.arm.execute(traj_1)
		self.arm.execute(traj_2)
		
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)


	def draw_circles(self, xy_center_pos, radius = 0.10):
		trajs = []
		for t in range(self.cont):
			self.target_pose.header.frame_id = self.reference_frame
			self.target_pose.header.stamp = rospy.Time.now()
			self.target_pose.pose.position.x = xy_center_pos[0] + radius * math.cos( 2 * math.pi * t / self.cont)
			self.target_pose.pose.position.y = xy_center_pos[1] + radius * math.sin( 2 * math.pi * t / self.cont)

			# 设置机器臂当前的状态作为运动初始状态
			self.arm.set_start_state_to_current_state()
			# 设置机械臂终端运动的目标位姿
			self.arm.set_pose_target(self.target_pose, self.end_effector_link)
			# 规划运动路径
			traj = self.arm.plan()
			trajs.append(traj)

		# 按照规划的运动路径控制机械臂运动
		for t in trajs:
			self.arm.execute(t)
			# rospy.sleep(1)


		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

    
    def draw_rectangle(self, xy_center_pos, side = 0.20):
        vertex_1 = [xy_center_pos[0] - side / 2, xy_center_pos[1] - side / 2]
        vertex_2 = [xy_center_pos[0] - side / 2, xy_center_pos[1] + side / 2]
        vertex_3 = [xy_center_pos[0] + side / 2, xy_center_pos[1] + side / 2]
        vertex_4 = [xy_center_pos[0] + side / 2, xy_center_pos[1] - side / 2]

		trajs.append(self.go_to(vertex_1))
		trajs.append(self.go_to(vertex_2))
		trajs.append(self.go_to(vertex_3))
		trajs.append(self.go_to(vertex_4))
		trajs.append(self.go_to(vertex_1))

		# 按照规划的运动路径控制机械臂运动
		for t in trajs:
			self.arm.execute(t)
			# rospy.sleep(1)

		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

	def draw_hexagon(self, xy_center_pos, side = 0.20):
        vertex_1 = [xy_center_pos[0] - side / 2, xy_center_pos[1] - np.sqrt(3) * side / 2]
        vertex_2 = [xy_center_pos[0] + side / 2, xy_center_pos[1] - np.sqrt(3) * side / 2]
        vertex_3 = [xy_center_pos[0] + side, xy_center_pos[1]]
        vertex_4 = [xy_center_pos[0] + side / 2, xy_center_pos[1] + np.sqrt(3) * side / 2]
        vertex_5 = [xy_center_pos[0] - side / 2, xy_center_pos[1] + np.sqrt(3) * side / 2]
        vertex_6 = [xy_center_pos[0] - side, xy_center_pos[1]]


        trajs.append(self.go_to(vertex_1))
		trajs.append(self.go_to(vertex_2))
		trajs.append(self.go_to(vertex_3))
		trajs.append(self.go_to(vertex_4))
		trajs.append(self.go_to(vertex_5))
		trajs.append(self.go_to(vertex_6))
		trajs.append(self.go_to(vertex_1))

		# 按照规划的运动路径控制机械臂运动
		for t in trajs:
			self.arm.execute(t)
			# rospy.sleep(1)

		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)



    def draw_curve(self, key_points):
        """ 
        key_points: A list of points
        This function draws a curve using Bezier method.
        """
        length = len(key_points)
    
        # Find mid points of key points
        mid_points = []
        dist = []
        for i in range(length - 1):
            temper = (np.array(key_points[i]) + np.array(key_points[i + 1]))/2
            mid_points.append(temper)
            dist.append(np.linalg.norm(np.array(key_points[i]) - np.array(key_points[i+1])))
        
        dist = dist/max(dist)

        # Find mid points of mid points
        mid_mid_points = []
        for i in range(length - 2):
            temper = (mid_points[i]+ mid_points[i + 1])/2
            mid_mid_points.append(temper)
        
        # Find the direction of difference of mid_mid points and key points
        t_vector = []
        for i in range(length - 2):
            temper = np.array(key_points[i + 1]) - mid_mid_points[i]
            t_vector.append(temper)
        
        # Find control points
        st_mid_points = [mid_points[0] + t_vector[0]]
        for i in range(length - 2):
            st_mid_points.append(mid_points[i + 0] * dist[i] + mid_mid_points[i] * (1 - dist[i]) + t_vector[i])
            st_mid_points.append(mid_points[i + 1] * dist[i + 1] + mid_mid_points[i] * (1 - dist[i + 1]) + t_vector[i])
        st_mid_points.append(mid_points[length - 2] + t_vector[length - 3])

        length = int(len(st_mid_points)/2+1)

        resolution = 1 * self.cont

		trajs = []

        for i in range(length - 1):
            for j in range(resolution):
                fi = float(j) / float(resolution)
                bezier_12 = np.array(key_points[i]) * (1 - fi) + np.array(st_mid_points[i * 2]) * fi
                bezier_23 = np.array(st_mid_points[i * 2]) * (1 - fi) + np.array(st_mid_points[i * 2 + 1]) * fi
                bezier_34 = np.array(st_mid_points[i * 2 + 1]) * (1 - fi) + np.array(key_points[i + 1]) * fi
                bezier_12_23 = bezier_12 * (1 - fi) + bezier_23 * fi
                bezier_23_34 = bezier_23 * (1 - fi) + bezier_34 * fi
                bezier_point = list(np.array(bezier_12_23)* (1 - fi) + np.array(bezier_23_34) * fi)

                self.target_pose.header.frame_id = self.reference_frame
                self.target_pose.header.stamp = rospy.Time.now()
                self.target_pose.pose.position.x = bezier_point[0]
                self.target_pose.pose.position.y = bezier_point[1]

                # 设置机器臂当前的状态作为运动初始状态
                self.arm.set_start_state_to_current_state()
                # 设置机械臂终端运动的目标位姿
                self.arm.set_pose_target(self.target_pose, self.end_effector_link)
                # 规划运动路径
                traj = self.arm.plan()

				trajs.append(traj)
		# 按照规划的运动路径控制机械臂运动
		for t in trajs:
			self.arm.execute(t)
                # rospy.sleep(1)


		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)



        


            

if __name__ == "__main__":
	demo = MoveItIkDemo()
	point = [-0.3, -0.3]
	demo.draw_circles(point, 0.1)