#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from mors.srv import QuadrupedCmd, QuadrupedCmdResponse, JointsCmd
import numpy as np
import sys

# SCENARIO = 1

STANDUP = 1
LAY_DOWN = 2
SITDOWN = 4
GIVE_HAND = 3

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

def talker(SCENARIO):
    # ROS stuff
    cmd_vel_topic = "cmd_vel"
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
    rospy.init_node('exp_scenarios', anonymous=True)

    cmd_vel_msg = Twist()

    rate = rospy.Rate(240) 

    # experimental stuff
    t_max = 0.0
    t = 0.0
    inc = 1.0/240.0
    
    
    if SCENARIO == 1:
        t_max = 5.0
    elif SCENARIO == 2:
        t_max = 15.0
    elif SCENARIO == 3:
        t_max = 15.0
    elif SCENARIO == 4:
        t_max = np.inf

    set_joints_kp([30.0]*12)
    set_joints_kd([0.1]*12)
    set_stride_height_client(0.07)
    set_mode_client(0)
    if set_action_client(STANDUP) == 1:
        rospy.loginfo("standup")

    for i in range(800):
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.angular.z = 0.0
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()

    # print(SCENARIO)
    # print(t)
    # print(t_max)
    
    while t < t_max:
        
        if SCENARIO == 1:
            cmd_vel_msg.linear.x = 0.4
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = 0.0
        elif SCENARIO == 2:
            if (t % 1.0) < inc:
                cmd_vel_msg.linear.x += 0.1
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = 0.0
        elif SCENARIO == 3:
            if t < 5.0:
                cmd_vel_msg.linear.x = 0.5
                cmd_vel_msg.linear.y = 0.0
            elif 5.0 <= t < 10.0:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.linear.y = 0.3
            elif 10.0 <= t < 15.0:
                cmd_vel_msg.linear.x = -0.5
                cmd_vel_msg.linear.y = -0.3
            cmd_vel_msg.angular.z = 0.0
        elif SCENARIO == 4:
            cmd_vel_msg.linear.x = 0.55
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = -0.25

        t += inc

        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()
    
    for i in range(480):
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.angular.z = 0.0
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()

    set_stride_height_client(0.0)
    if set_action_client(LAY_DOWN) == 1:
        rospy.loginfo("laydown")
    
    rospy.loginfo("Congratulations! The test scenario is finished.")

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print("Usage: rosrun exp_scenario exp_scenario.py scenarioNumber")
        else:
            talker(int(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass