#!/usr/bin/env python

import rospy
import lcm
from lcm_msgs4 import servo_cmd_msg
from trajectory_msgs.msg import JointTrajectoryPoint
from mors.srv import JointsCmd, JointsCmdRequest, JointsCmdResponse

lc = lcm.LCM()
lcm_msg = servo_cmd_msg()

kp = [0.0]*12
kd = [0.0]*12

def srv_callback_kp(self, req : JointsCmdRequest):
    rospy.loginfo(f"I got new kp={req.cmd}")
    global kp
    kp = req.cmd
    return JointsCmdResponse(1, "get kp")

def srv_callback_kd(self, req : JointsCmdRequest):
    rospy.loginfo(f"I got new kd={req.cmd}")
    global kd
    kd = req.cmd
    return JointsCmdResponse(1, "get kd")

def callback(msg : JointTrajectoryPoint):
    lcm_msg.position = []
    lcm_msg.velocity = []
    lcm_msg.torque = []
    lcm_msg.kp = []
    lcm_msg.kd = []

    for o in joint_order:
        lcm_msg.position.append(msg.positions[o]*joint_directions[o])
        lcm_msg.velocity.append(msg.velocities[o]*joint_directions[o])
        lcm_msg.torque.append(msg.effort[o]*joint_directions[o])
        lcm_msg.kp.append(kp[o])
        lcm_msg.kd.append(kd[o])

    lc.publish(lcm_joint_cmd_topic, lcm_msg.encode())


if __name__ == '__main__':
    rospy.init_node("servo_cmd_ros2lcm")

    frequency = rospy.get_param("~frequency", 500)
    ros_joint_cmd_topic = rospy.get_param("~ros_joint_cmd_topic", "joint_group_position_controller/command")
    lcm_joint_cmd_topic = rospy.get_param("~lcm_joint_cmd_topic", "SERVO_CMD")

    joint_order = rospy.get_param("~joint_order", [3, 5, 4, 9, 11, 10, 0, 2, 1, 6, 8, 7])
    joint_directions = rospy.get_param("~joint_directions", [ 1,  -1,  -1, 1,  1,  1, -1,  -1,  -1, -1,  1,  1])

    rate = rospy.Rate(frequency)

    rospy.Subscriber(ros_joint_cmd_topic, JointTrajectoryPoint, callback, queue_size=1)

    rospy.loginfo("ros2lcm configured")
    rospy.spin()