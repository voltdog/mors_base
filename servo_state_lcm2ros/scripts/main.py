#!/usr/bin/env python

import rospy
# from robo_msgs.msg import robot_state
from threading import Thread
import lcm
from servo_state_lcm2ros.lcm_receiver import LCMReceiver
from sensor_msgs.msg import JointState


if __name__ == '__main__':

    leg_data = {}
    leg_data["pos"] = [0] * 12
    leg_data["vel"] = [0] * 12
    leg_data["torq"] = [0] * 12
    leg_data["name"] = [""] * 12

    # ros things
    rospy.init_node("servo_state_lcm2ros")

    frequency = rospy.get_param("~frequency", 300)
    ros_joint_state_topic = rospy.get_param("~ros_joint_state_topic", "joint_state")
    lcm_joint_state_topic = rospy.get_param("~lcm_joint_state_topic", "SERVO_STATE")

    joint_order = rospy.get_param("~joint_order", [3, 5, 4, 9, 11, 10, 0, 2, 1, 6, 8, 7])
    joint_directions = rospy.get_param("~joint_directions", [ 1,  -1,  -1, 1,  1,  1, -1,  -1,  -1, -1,  1,  1])

    rate = rospy.Rate(frequency)

    pub = rospy.Publisher(ros_joint_state_topic, JointState, queue_size=10)
    state_msg = JointState()

    # lcm things
    lcm_receiver = LCMReceiver(channel=lcm_joint_state_topic)

    rospy.loginfo("lcm2ros configured")

    while not rospy.is_shutdown():
        state_msg.name = []
        state_msg.position = []
        state_msg.velocity = []
        state_msg.effort = []

        leg_data["pos"], leg_data["vel"], leg_data["torq"], leg_data["name"] = lcm_receiver.get_ros_msg()

        for o in joint_order:
            state_msg.name.append(leg_data["name"][o].decode('utf-8'))
            state_msg.position.append(leg_data["pos"][o]*joint_directions[o])
            state_msg.velocity.append(leg_data["vel"][o]*joint_directions[o])
            state_msg.effort.append(leg_data["torq"][o]*joint_directions[o])

        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = ""

        pub.publish(state_msg)
        rate.sleep()