#!/usr/bin/env python

import rospy
import rosnode
from std_msgs.msg import Bool, UInt8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray, Pose


class CmdCommutator(object):
    def __init__(self) -> None:
        self.cur_device = 0
        
        rospy.init_node("cmd_commutator")

        freq = rospy.get_param('~frequency', 500)

        # read config file
        output_cmd_vel_topic = rospy.get_param('~output_cmd_vel_topic', "cmd_vel")
        output_cmd_pose_topic = rospy.get_param('~output_cmd_pose_topic', "cmd_pose")
        output_cmd_ef_pose_topic = rospy.get_param('~output_cmd_ef_pose_topic', "ef_position/command")
        output_cmd_joint_pos_topic = rospy.get_param('~output_cmd_joint_pos_topic', "joint_group_position_controller/command")
        output_status_topic = rospy.get_param('~output_status_topic', "cur_device")

        input1_cmd_vel_topic = rospy.get_param('~input1_cmd_vel_topic', "ds4/cmd_vel")
        input1_cmd_pose_topic = rospy.get_param('~input1_cmd_pose_topic', "ds4/cmd_pose")
        input1_cmd_ef_pose_topic = rospy.get_param('~input1_cmd_ef_pose_topic', "ds4/ef_position/command")
        input1_cmd_joint_pos_topic = rospy.get_param('~input1_cmd_joint_pos_topic', "ds4/joint_group_position_controller/command")
        input1_status_topic = rospy.get_param('~input1_status_topic', "ds4/status")

        input2_cmd_vel_topic = rospy.get_param('~input2_cmd_vel_topic', "radiolink/cmd_vel")
        input2_cmd_pose_topic = rospy.get_param('~input2_cmd_pose_topic', "radiolink/cmd_pose")
        input2_cmd_ef_pose_topic = rospy.get_param('~input2_cmd_ef_pose_topic', "radiolink/ef_position/command")
        input2_cmd_joint_pos_topic = rospy.get_param('~input2_cmd_joint_pos_topic', "radiolink/joint_group_position_controller/command")
        input2_status_topic = rospy.get_param('~input2_status_topic', "radiolink/status")

        input3_cmd_vel_topic = rospy.get_param('~input3_cmd_vel_topic', "nav/cmd_vel")
        input3_cmd_pose_topic = rospy.get_param('~input3_cmd_pose_topic', "nav/cmd_pose")
        input3_cmd_ef_pose_topic = rospy.get_param('~input3_cmd_ef_pose_topic', "nav/ef_position/command")
        input3_cmd_joint_pos_topic = rospy.get_param('~input3_cmd_joint_pos_topic', "nav/joint_group_position_controller/command")
        input3_status_topic = rospy.get_param('~input3_status_topic', "nav/status")

        input4_cmd_vel_topic = rospy.get_param('~input4_cmd_vel_topic', "gui/cmd_vel")
        input4_cmd_pose_topic = rospy.get_param('~input4_cmd_pose_topic', "gui/cmd_pose")
        input4_cmd_ef_pose_topic = rospy.get_param('~input4_cmd_ef_pose_topic', "gui/ef_position/command")
        input4_cmd_joint_pos_topic = rospy.get_param('~input4_cmd_joint_pos_topic', "gui/joint_group_position_controller/command")
        input4_status_topic = rospy.get_param('~input4_status_topic', "gui/status")

        self.rate = rospy.Rate(freq)

        # subscribers
        rospy.Subscriber(input1_cmd_vel_topic, Twist, self.input1_cmd_vel_callback, queue_size=1)
        rospy.Subscriber(input1_cmd_pose_topic, Twist, self.input1_cmd_pose_callback, queue_size=1)
        rospy.Subscriber(input1_cmd_ef_pose_topic, PoseArray, self.input1_cmd_ef_pose_callback, queue_size=1)
        rospy.Subscriber(input1_cmd_joint_pos_topic, JointTrajectoryPoint, self.input1_cmd_joint_pos_callback, queue_size=1)
        rospy.Subscriber(input1_status_topic, Bool, self.input1_status_callback, queue_size=1)

        rospy.Subscriber(input2_cmd_vel_topic, Twist, self.input2_cmd_vel_callback, queue_size=1)
        rospy.Subscriber(input2_cmd_pose_topic, Twist, self.input2_cmd_pose_callback, queue_size=1)
        rospy.Subscriber(input2_cmd_ef_pose_topic, PoseArray, self.input2_cmd_ef_pose_callback, queue_size=1)
        rospy.Subscriber(input2_cmd_joint_pos_topic, JointTrajectoryPoint, self.input2_cmd_joint_pos_callback, queue_size=1)
        rospy.Subscriber(input2_status_topic, Bool, self.input2_status_callback, queue_size=1)

        rospy.Subscriber(input3_cmd_vel_topic, Twist, self.input3_cmd_vel_callback, queue_size=1)
        rospy.Subscriber(input3_cmd_pose_topic, Twist, self.input3_cmd_pose_callback, queue_size=1)
        rospy.Subscriber(input3_cmd_ef_pose_topic, PoseArray, self.input3_cmd_ef_pose_callback, queue_size=1)
        rospy.Subscriber(input3_cmd_joint_pos_topic, JointTrajectoryPoint, self.input3_cmd_joint_pos_callback, queue_size=1)
        rospy.Subscriber(input3_status_topic, Bool, self.input3_status_callback, queue_size=1)

        rospy.Subscriber(input4_cmd_vel_topic, Twist, self.input4_cmd_vel_callback, queue_size=1)
        rospy.Subscriber(input4_cmd_pose_topic, Twist, self.input4_cmd_pose_callback, queue_size=1)
        rospy.Subscriber(input4_cmd_ef_pose_topic, PoseArray, self.input4_cmd_ef_pose_callback, queue_size=1)
        rospy.Subscriber(input4_cmd_joint_pos_topic, JointTrajectoryPoint, self.input4_cmd_joint_pos_callback, queue_size=1)
        rospy.Subscriber(input4_status_topic, Bool, self.input4_status_callback, queue_size=1)

        # publishers
        self.cmd_vel_pub = rospy.Publisher(output_cmd_vel_topic, Twist, queue_size=10)
        self.cmd_pose_pub = rospy.Publisher(output_cmd_pose_topic, Twist, queue_size=10)
        self.cmd_ef_pos_pub = rospy.Publisher(output_cmd_ef_pose_topic, PoseArray, queue_size=10)
        self.cmd_joint_pos_pub = rospy.Publisher(output_cmd_joint_pos_topic, JointTrajectoryPoint, queue_size=10)
        self.status_pub = rospy.Publisher(output_status_topic, UInt8, queue_size=10)

        # messages
        self.cmd_vel_msg = Twist()
        self.cmd_pose_msg = Twist()
        self.cmd_ef_msg = PoseArray()
        self.cmd_joint_msg = JointTrajectoryPoint()


        self.input1_status = False
        self.input2_status = False
        self.input3_status = False
        self.input4_status = False

        rospy.loginfo("cmd_commutator configured")

    def run(self):
        while not rospy.is_shutdown():
            if rosnode.rosnode_ping("/move_base", max_count=2, verbose=False):
                if  self.input1_status == False and self.input2_status == False:
                    self.cur_device = 3
            else:
                if self.input1_status == False and self.input2_status == False and self.input4_status == True:
                    self.cur_device = 4
                    self.input3_status = False

            if self.input1_status == True:
                self.cur_device = 1
            elif self.input2_status == True:
                self.cur_device = 2
            elif self.input3_status == True:
                self.cur_device = 3
            elif self.input4_status == True:
                self.cur_device = 4
            else:
                self.cur_device = 0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                self.cmd_pose_pub.publish(self.cmd_pose_msg)
                self.cmd_ef_pos_pub.publish(self.cmd_ef_msg)
                self.cmd_joint_pos_pub.publish(self.cmd_joint_msg)

            

            self.status_pub.publish(self.cur_device)
            self.rate.sleep()

    def input1_cmd_vel_callback(self, msg : Twist):
        if self.cur_device == 1:
            self.cmd_vel_pub.publish(msg)

    def input1_cmd_pose_callback(self, msg : Twist):
        if self.cur_device == 1:
            self.cmd_pose_pub.publish(msg)

    def input1_cmd_ef_pose_callback(self, msg : PoseArray):
        if self.cur_device == 1:
            self.cmd_ef_pos_pub.publish(msg)

    def input1_cmd_joint_pos_callback(self, msg : JointTrajectoryPoint):
        if self.cur_device == 1:
            self.cmd_joint_pos_pub.publish(msg)

    def input1_status_callback(self, msg : Bool):
        self.input1_status = msg.data

    def input2_cmd_vel_callback(self, msg : Twist):
        if self.cur_device == 2:
            self.cmd_vel_pub.publish(msg)

    def input2_cmd_pose_callback(self, msg : Twist):
        if self.cur_device == 2:
            self.cmd_pose_pub.publish(msg)

    def input2_cmd_ef_pose_callback(self, msg : PoseArray):
        if self.cur_device == 2:
            self.cmd_ef_pos_pub.publish(msg)

    def input2_cmd_joint_pos_callback(self, msg : JointTrajectoryPoint):
        if self.cur_device == 2:
            self.cmd_joint_pos_pub.publish(msg)

    def input2_status_callback(self, msg : Bool):
        self.input2_status = msg.data

    def input3_cmd_vel_callback(self, msg : Twist):
        if self.cur_device == 3:
            self.cmd_vel_pub.publish(msg)

    def input3_cmd_pose_callback(self, msg : Twist):
        if self.cur_device == 3:
            self.cmd_pose_pub.publish(msg)

    def input3_cmd_ef_pose_callback(self, msg : PoseArray):
        if self.cur_device == 3:
            self.cmd_ef_pos_pub.publish(msg)

    def input3_cmd_joint_pos_callback(self, msg : JointTrajectoryPoint):
        if self.cur_device == 3:
            self.cmd_joint_pos_pub.publish(msg)

    def input3_status_callback(self, msg : Bool):
        self.input3_status = msg.data

    def input4_cmd_vel_callback(self, msg : Twist):
        if self.cur_device == 4:
            self.cmd_vel_pub.publish(msg)

    def input4_cmd_pose_callback(self, msg : Twist):
        if self.cur_device == 4:
            self.cmd_pose_pub.publish(msg)

    def input4_cmd_ef_pose_callback(self, msg : PoseArray):
        if self.cur_device == 4:
            self.cmd_ef_pos_pub.publish(msg)

    def input4_cmd_joint_pos_callback(self, msg : JointTrajectoryPoint):
        if self.cur_device == 4:
            self.cmd_joint_pos_pub.publish(msg)

    def input4_status_callback(self, msg : Bool):
        self.input4_status = msg.data



if __name__ == '__main__':
    cmd_com = CmdCommutator()
    cmd_com.run()