#!/usr/bin/env python

import depends.joy_converter_control as j
import rospy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray, Pose
from mors.srv import QuadrupedCmd, QuadrupedCmdResponse, JointsCmd
import copy
import numpy as np
import time

import evdev
import pyudev

STANDUP = 1
LAY_DOWN = 2
SITDOWN = 4
GIVE_HAND = 3

X = 0
Y = 1
Z = 2

def get_device_name_by_path(name:str='js0')->str:
    context = pyudev.Context()
    input_num = None
    for device in context.list_devices(): 
        if name in device.sys_path: 
            input_num = device.sys_path.lower().split('/')[-2]
            break
    if input_num != None:
        for device in context.list_devices(): 
            if (input_num in device.sys_path) and ('event' in device.sys_path): 
                event = device.sys_path.split('/')[-1]
                devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
                for device in devices:
                    if event in device.path:
                        return device.name
    return None

# call robot_mode service
def set_mode_client(mode):
    rospy.wait_for_service('robot_mode')
    try:
        set_mode = rospy.ServiceProxy('robot_mode', QuadrupedCmd)
        resp = set_mode(mode)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# call robot_action service
def set_action_client(action):
    rospy.wait_for_service('robot_action')
    try:
        set_action = rospy.ServiceProxy('robot_action', QuadrupedCmd)
        resp = set_action(action)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def set_stride_height_client(height):
    rospy.wait_for_service('stride_height')
    try:
        set_height = rospy.ServiceProxy('stride_height', QuadrupedCmd)
        resp = set_height(height)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def set_joints_kp(kp):
    rospy.wait_for_service("joints_kp")
    try:
        set_kp_srv = rospy.ServiceProxy('joints_kp', JointsCmd)
        resp = set_kp_srv(kp)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def set_joints_kd(kd):
    rospy.wait_for_service("joints_kd")
    try:
        set_kd_srv = rospy.ServiceProxy('joints_kd', JointsCmd)
        resp = set_kd_srv(kd)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# print(get_device_name_by_path('js0'))

if __name__ == '__main__':
    print('INIT')
    rospy.init_node("joy_converter_node")
    
    #load parameters
    freq = rospy.get_param('~general/frequency', 40)

    cmd_vel_topic = rospy.get_param('~topics/cmd_vel', "joy_converter/cmd_vel")
    cmd_pose_topic = rospy.get_param('~topics/cmd_pose', "joy_converter/cmd_pose")
    cmd_ef_pos_topic = rospy.get_param('~topics/cmd_ef_pose', "joy_converter/ef_position/command")
    cmd_joint_pos_topic = rospy.get_param('~topics/cmd_joint_pos', "joy_converter/joint_group_position_controller/command")
    joy_status_topic = rospy.get_param('~topics/status', "joy_converter/status")

    max_speed_x = rospy.get_param('~gait/max_speed_x', 0.3)
    max_speed_y = rospy.get_param('~gait/max_speed_y', 0.2)
    max_speed_z = rospy.get_param('~gait/max_speed_z', 0.8)
    stride_height = rospy.get_param('~gait/stride_height', 0.05)

    max_angle_x = rospy.get_param('~pose/max_angle_x', 0.33)
    max_angle_y = rospy.get_param('~pose/max_angle_y', 0.33)
    max_angle_z = rospy.get_param('~pose/max_angle_z', 0.33)
    max_lin_x = rospy.get_param('~pose/max_lin_x', 0.1)
    max_lin_y = rospy.get_param('~pose/max_lin_y', 0.1)
    max_lin_z = rospy.get_param('~pose/max_lin_z', 0.1)
    min_lin_z = rospy.get_param('~pose/min_lin_z', 0.1)

    ef_max_pos_x = rospy.get_param('~ef/max_pos_x', 0.05)
    ef_max_pos_y = rospy.get_param('~ef/max_pos_y', 0.05)
    ef_max_pos_z = rospy.get_param('~ef/max_pos_z', 0.05)
    ef_min_pos_z = rospy.get_param('~ef/min_pos_z', 0.05)

    max_abad = rospy.get_param('~joints/max_abad', 1.57)
    max_hip = rospy.get_param('~joints/max_hip', 1.57)
    max_knee = rospy.get_param('~joints/max_knee', 1.57)

    walk_mode = rospy.get_param('~modes/walk', 0)
    ef_mode = rospy.get_param('~modes/ef', 1)
    body_mode = rospy.get_param('~modes/body', 2)
    joint_mode = rospy.get_param('~modes/joint', 3)
 
  
    joy_path = rospy.get_param('~general/joystick_path', 'js0')
    name = get_device_name_by_path(joy_path)
    print(f"Joystick name: {name}") 
    joy_config = rospy.get_param('~joystick_config')   

    # publishers
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    cmd_pose_pub = rospy.Publisher(cmd_pose_topic, Twist, queue_size=10)
    cmd_ef_pos_pub = rospy.Publisher(cmd_ef_pos_topic, PoseArray, queue_size=10)
    cmd_joint_pos_pub = rospy.Publisher(cmd_joint_pos_topic, JointTrajectoryPoint, queue_size=10)
    status_pub = rospy.Publisher(joy_status_topic, Bool, queue_size=10)

    # messages
    cmd_vel_msg = Twist()
    cmd_pose_msg = Twist()
    cmd_ef_msg = PoseArray()
    cmd_joint_msg = JointTrajectoryPoint()
    cmd_joint_msg.velocities = [0]*12
    cmd_joint_msg.effort = [0]*12
    cmd_joint_msg.positions = [0]*12

    joy_converter = j.JOY_CONVERTER()
    joy_converter.set_joy_config(joy_config)
    rate = rospy.Rate(freq)

    # services data
    mode = 0
    mode_prev = 0
    action = 0
    action_prev = 0

    # other data
    joy_value = [0]*27
    joy_value_prev = [0]*27
    is_standup = 0
    is_sitting = 0

    cur_leg = 0
    cur_joint = 0

    ef_pos = [[0]*3, [0]*3, [0]*3, [0]*3]
    joint_pos = [0]*12

    # check joy connection variables
    err_cnt = 20
    joy_status = False

    time.sleep(0.5)
    joy_value = joy_converter.get_joy_data()
    LX_init = joy_value[j.LX]

    stride_height_set = False
    print("SET!")
    set_joints_kp([30.0]*12)
    set_joints_kd([0.1]*12)
    set_mode_client(mode)
    set_stride_height_client(stride_height)
    print("Cycle")
    while not rospy.is_shutdown():
        # print(joy_value)
        joy_value = joy_converter.get_joy_data()
        # print(joy_value)
        # change modes
        if joy_value[j.SHARE] == 1 and joy_value_prev[j.SHARE] == 0 and joy_value[j.R1] == 0 and joy_value[j.L1] == 0:
            mode = walk_mode
        elif joy_value[j.SHARE] == 1 and joy_value_prev[j.SHARE] == 0 and joy_value[j.R1] == 1 and joy_value[j.L1] == 0:
            mode = body_mode
        elif joy_value[j.SHARE] == 1 and joy_value_prev[j.SHARE] == 0 and joy_value[j.R1] == 0 and joy_value[j.L1] == 1:
            mode = ef_mode
        elif joy_value[j.SHARE] == 1 and joy_value_prev[j.SHARE] == 0 and joy_value[j.R1] == 1 and joy_value[j.L1] == 1:
            mode = joint_mode
        
        if mode == walk_mode and is_standup == 1:
            cmd_vel_msg.linear.x = joy_value[j.LY]*max_speed_x
            cmd_vel_msg.linear.y = -(joy_value[j.LX]-LX_init)*max_speed_y
            cmd_vel_msg.angular.z = -joy_value[j.RX]*max_speed_z

        elif mode == body_mode and is_standup == 1:
            if joy_value[j.LY] >= 0:
                cmd_pose_msg.linear.z = joy_value[j.LY]*max_lin_z
            else:
                cmd_pose_msg.linear.z = joy_value[j.LY]*min_lin_z

            cmd_pose_msg.angular.z = joy_value[j.RX]*max_angle_z * (1 - np.abs(joy_value[j.LY]))
            cmd_pose_msg.angular.y = joy_value[j.RY]*max_angle_y * (1 - np.abs(joy_value[j.LY]))
            cmd_pose_msg.angular.x = joy_value[j.LX]*max_angle_x * (1 - np.abs(joy_value[j.LY]))
            # print(joy_value[j.LY])
        elif mode == ef_mode and is_standup == 1:
            if joy_value[j.RIGHT] == 1 and joy_value_prev[j.RIGHT] == 0:
                if cur_leg < 3:
                    cur_leg += 1
                else:
                    cur_leg = 0
            if joy_value[j.LEFT] == 1 and joy_value_prev[j.LEFT] == 0:
                if cur_leg > 0:
                    cur_leg -= 1
                else:
                    cur_leg = 3
            
            ef_pos[cur_leg][X] += joy_value[j.LY]/200
            ef_pos[cur_leg][Y] += joy_value[j.LX]/200
            ef_pos[cur_leg][Z] += joy_value[j.RY]/200

            ef_pos[cur_leg][X] = max(min(ef_pos[cur_leg][X], ef_max_pos_x), -ef_max_pos_x)
            ef_pos[cur_leg][Y] = max(min(ef_pos[cur_leg][Y], ef_max_pos_y), -ef_max_pos_y)
            ef_pos[cur_leg][Z] = max(min(ef_pos[cur_leg][Z], ef_max_pos_z), -ef_min_pos_z)

            cmd_ef_msg.poses.clear()
            for i in range(4):
                cmd_one_ef_msg = Pose()
                cmd_one_ef_msg.position.x = ef_pos[i][X]
                cmd_one_ef_msg.position.y = ef_pos[i][Y]
                cmd_one_ef_msg.position.z = ef_pos[i][Z]
                cmd_ef_msg.poses.append(cmd_one_ef_msg)

        elif mode == joint_mode and is_standup == 1:
            if joy_value[j.RIGHT] == 1 and joy_value_prev[j.RIGHT] == 0:
                if cur_joint < 11:
                    cur_joint += 1
                else:
                    cur_joint = 0
            if joy_value[j.LEFT] == 1 and joy_value_prev[j.LEFT] == 0:
                if cur_joint >= 1:
                    cur_joint -= 1
                else:
                    cur_joint = 11
            joint_pos[cur_joint] += joy_value[j.RY]/50
            if cur_joint % 3 == 0:
                joint_pos[cur_joint] = max(min(joint_pos[cur_joint], max_abad), -max_abad)
            elif cur_joint % 3 == 1:
                joint_pos[cur_joint] = max(min(joint_pos[cur_joint], max_hip), -max_hip)  
            elif cur_joint % 3 == 2:
                joint_pos[cur_joint] = max(min(joint_pos[cur_joint], max_knee), -max_knee)  
            
            cmd_joint_msg.positions = joint_pos
        
        # whether should call robot_mode or robot_action services 
        if mode != mode_prev:
            # joy_converter.set_joy_color(mode)
            set_mode_client(mode)

        if joy_value[j.OPTIONS] == 1 and joy_value_prev[j.OPTIONS] == 0 and (is_standup == 0 or is_sitting == 1):
            # set_mode_client(mode)
            if set_action_client(STANDUP) == 1:
                is_standup = 1
                is_sitting = 0
                rospy.loginfo("standup")
        elif joy_value[j.OPTIONS] == 1 and joy_value_prev[j.OPTIONS] == 0 and is_standup == 1:
            if set_action_client(LAY_DOWN) == 1:
                is_standup = 0
                rospy.loginfo("laydown")

        elif joy_value[j.CIRCLE] == 1 and joy_value_prev[j.CIRCLE] == 0 and is_standup == 1:
            set_action_client(SITDOWN)
            is_sitting = 1
            # rospy.loginfo("sitdown")

        elif joy_value[j.CROSS] == 1 and joy_value_prev[j.CROSS] == 0 and is_standup == 1:
            set_action_client(GIVE_HAND)
            rospy.loginfo("give hand")

        joy_status = joy_value[j.STATUS]=="OK"
        # joy_status = True

        joy_value_prev = joy_value[:]
        mode_prev = mode

        cmd_vel_pub.publish(cmd_vel_msg)
        cmd_pose_pub.publish(cmd_pose_msg)
        cmd_ef_pos_pub.publish(cmd_ef_msg)
        cmd_joint_pos_pub.publish(cmd_joint_msg)
        status_pub.publish(joy_status)

        rate.sleep()