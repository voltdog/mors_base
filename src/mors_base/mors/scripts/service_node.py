#!/usr/bin/env python

import rospy
from mors.srv import QuadrupedCmd, QuadrupedCmdResponse, QuadrupedCmdRequest
from mors.srv import JointsCmd, JointsCmdRequest, JointsCmdResponse

def callback_mode(req : QuadrupedCmdRequest):
    rospy.loginfo(f"I got mode num: {req.cmd}")
    return QuadrupedCmdResponse(1, "get the mode")

def callback_action(req : QuadrupedCmdRequest):
    rospy.loginfo(f"I got action num: {req.cmd}")
    return QuadrupedCmdResponse(1, "get the action")

def callback_stride_height(req : QuadrupedCmdRequest):
    rospy.loginfo(f"I got stride height: {req.cmd}")
    return QuadrupedCmdResponse(1, "get stride height")

def callback_kp(req : JointsCmdRequest):
    rospy.loginfo(f"I got new kp={req.cmd}")
    return JointsCmdResponse(1, "get kp")

def callback_kd(req : JointsCmdRequest):
    rospy.loginfo(f"I got new kd={req.cmd}")
    return JointsCmdResponse(1, "get kd")

if __name__ == "__main__":
    rospy.init_node('service_server')
    sm = rospy.Service('robot_mode', QuadrupedCmd, callback_mode)
    sa = rospy.Service('robot_action', QuadrupedCmd, callback_action)
    sh = rospy.Service('stride_height', QuadrupedCmd, callback_stride_height)
    skp = rospy.Service('joints_kp', JointsCmd, callback_kp)
    skd = rospy.Service('joints_kd', JointsCmd, callback_kd)
    rospy.loginfo("Ready to respond your requests")
    rospy.spin()

