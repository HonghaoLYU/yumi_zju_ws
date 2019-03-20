#!/usr/bin/env python
# coding=UTF-8
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 全局变量定义以及赋值
a = 0


# # 定义回调函数,订阅接受到的消息传给data
# def callback(data):
#     # 对全局变量a进行赋值
#     global a
#     a = data.data
#     # 转换a为浮点型
#     a = float(a)


class MoveGroupPythonInteface(object):
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        # 初始化 `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface',anonymous=True)

        # 订阅话题
        # rospy.Subscriber('yumiaction', String, callback)

        # 实例化 a `RobotCommander`_ 对象.
        robot = moveit_commander.RobotCommander()

        # 实例化 a `PlanningSceneInterface`_ 对象.
        scene = moveit_commander.PlanningSceneInterface()

        # 实例化 a `MoveGroupCommander`_ 对象.
        group_name = "right_arm"
        arm = moveit_commander.MoveGroupCommander(group_name)
        hand = moveit_commander.MoveGroupCommander("right_hand")

        # 创建 `DisplayTrajectory`_ publisher,稍后用于发布RViz可视化的轨迹
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        # 获取机器人的参考坐标系并输出
        planning_frame = arm.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # 获取当前末端执行器并输出
        eef_link = arm.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # 获取机器人中所有groups的名称并打印:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # 输出当前机器人的全部状态便于调试:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # 各种变量:
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.arm = arm
        self.hand = hand
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        # 设置动作对象变量,此处为arm
        arm = self.arm
        # 获取当前目标点关节状态
        joint_goal = arm.get_current_joint_values()
        if count == 1:
            joint_goal[0] = 0
            joint_goal[1] = -pi/4
            joint_goal[2] = 0
            joint_goal[3] = -pi/2
            joint_goal[4] = 0
            joint_goal[5] = pi/3
            joint_goal[6] = 0
        else:
            joint_goal[0] = 0
            joint_goal[1] = -pi/4
            joint_goal[2] = 0
            joint_goal[3] = -pi/2
            joint_goal[4] = 0
            joint_goal[5] = pi/3
            joint_goal[6] = 0

        # 规划并执行路径动作
        arm.go(joint_goal, wait=True)

        # 调用 stop() 命令，确保动作停止
        arm.stop()

    def go_to_pose_goal(self):
        # 设置动作对象变量,此处为arm
        arm = self.arm

        # 获取当前末端执行器位置姿态
        pose_goal = self.arm.get_current_pose().pose

        print (a)

        # 设置动作对象目标位置姿态
        pose_goal.orientation.x = pose_goal.orientation.x
        pose_goal.orientation.y = pose_goal.orientation.y
        pose_goal.orientation.z = pose_goal.orientation.z
        pose_goal.orientation.w = pose_goal.orientation.w
        pose_goal.position.x = pose_goal.position.x
        pose_goal.position.y = pose_goal.position.y + a
        pose_goal.position.z = pose_goal.position.z
        arm.set_pose_target(pose_goal)
        print "End effector pose %s" % pose_goal

        # euler_from_quaternion(pose_goal.orientation,)

        # 规划和输出动作
        arm.go(wait=True)
        # 确保没有剩余未完成动作在执行
        arm.stop()
        # 动作完成后清除目标信息
        arm.clear_pose_targets()

    def go_to_gripper_goal(self):
        # 设置动作对象变量，此处为gripper
        gripper = self.gripper

        # 获取当前gripper姿态信息
        gripper_goal = self.gripper.get_joint_value_target()
        print "Gripper pose %s" % gripper_goal

        # 设置gripper目标姿态
        gripper.set_joint_value_target([0.02, 0.02])

        # 规划和输出动作
        gripper.go(wait=True)
        # 确保没有剩余未完成动作在执行
        gripper.stop()
        # 动作完成后清除目标信息
        gripper.clear_pose_targets()

    def go_to_gripper_joint_goal(self):
        # 设置动作对象变量,此处为arm
        arm = self.arm
        # 获取当前目标点关节状态
        joint_goal = arm.get_current_joint_values()
        joint_goal[7] = joint_goal[7] + 0.001

        # 规划并执行路径动作
        arm.go(joint_goal, wait=True)

        # 调用 stop() 命令，确保动作停止
        arm.stop()

    def go_to_home_goal(self):
        # 控制机械臂回到初始化位置
        arm = self.arm
        arm.set_named_target('home')
        arm.go()

    def go_to_cal_goal(self):
        # 控制机械臂回到校准位置
        arm = self.arm
        arm.set_named_target('calc')
        arm.go()

    def go_to_ready_goal(self):
        # 控制机械臂回到校准位置
        arm = self.arm
        arm.set_named_target('ready')
        arm.go()

    def go_to_open_goal(self):
        # 控制夹具张开
        hand = self.hand
        hand.set_named_target('open')
        hand.go()


def main():
    # 定义全局变量count,判断程序执行次数
    global count
    count = 0

    # 输入回车,执行初始化程序
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    yumi = MoveGroupPythonInteface()

    # 循环等待,执行动作程序
    while 1:
        count = count + 1

        # 首次程序执行,回到动作原点
        # if count == 1:
        print "============ Press `Enter` to execute a arm movement using a joint state goal for the original state..."
        raw_input()
        yumi.go_to_cal_goal()
        yumi.go_to_open_goal()

        # # 执行arm目标点动作
        # print "============ Press `Enter` to execute a arm movement using a pose goal ..."
        # raw_input()
        # yumi.go_to_gripper_joint_goal()

        # # 执行gripper目标点动作
        # print "============ Press `Enter` to execute a gripper movement using a pose goal ..."
        # raw_input()
        # yumi.go_to_gripper_goal()

if __name__ == '__main__':
    main()
