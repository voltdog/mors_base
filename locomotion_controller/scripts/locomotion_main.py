#!/usr/bin/env python


import configparser
from distutils.command.config import config
import time
from threading import Thread
from zmp_controller.lcm_data_exchange import *
from zmp_controller.robot_controller import RobotController

from math import *
import numpy as np
from statistics import mean
import rospy
from pyrr import Quaternion
from scipy.spatial.transform import Rotation

from mors.msg import ContactsStamped
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, PoseArray, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool
from mors.srv import QuadrupedCmd, QuadrupedCmdResponse, QuadrupedCmdRequest
from mors.srv import JointsCmd, JointsCmdRequest, JointsCmdResponse

import socket
hostname = socket.gethostname()
if hostname == "mors":
	ISREALROBOT = True
	print("Running on the robot...")
else:
	ISREALROBOT = False
	print("Running on a PC...")
        
X = 0
Y = 1
Z = 2


class Locomotion_Control(object):
    def __init__(self):
        self.cmd_vel = [0]*3
        self.cmd_pose = [0]*6
        self.cmd_ef_pos = [0]*12
        self.cmd_joint_pos = [0]*12
        self.base_orient_q = [0]*12
        self.robot_disabled = False

        rospy.init_node("locomotion_controller")
        self._get_ros_params()
        self.controller = RobotController(freq=self.freq,
                                          locomotion_kp=self.kp, 
                                          locomotion_kd=self.kd,
                                          yaw_kp=self.yaw_kp,
                                          kinematic_scheme=self.kinematic_scheme,
                                          ef_init_x=self.ef_init_x,
                                          ef_init_y=self.ef_init_y,
                                          robot_height=self.robot_height,
                                          stride_frequency=self.stride_frequency,
                                          preview_horizon=self.preview_horizon,
                                          cog_offset_x=self.cog_x_offset,
                                          cog_offset_y=self.cog_y_offset,
                                          cog_offset_z=self.cog_z_offset,)
        # self.controller.set_cmd_joint_pos([0]*12, [0]*12, [0]*12)
        # self.controller.set_kp([0]*12)
        # self.controller.set_kd([0]*12)
        self._init_ros()

        self.lcm_exch = LCMDataExchange(servo_cmd_cnannel=self.lcm_servo_cmd_channel,
                                   servo_state_channel=self.lcm_servo_state_channel)
        

        self._init_lcm()

        print(f"Kinematic scheme: {self.kinematic_scheme}")

        self.pre_lpf_param = np.array([0]*4)

        rospy.loginfo("Locomotion_controller configured")

    def _init_lcm(self):
        self.servo_state_th = Thread(target=self.lcm_exch.servo_state_thread, args=())
        self.servo_state_th.daemon = True
        self.servo_state_th.start()

    def _get_ros_params(self):
        # read config
        self.freq = rospy.get_param('~general/frequency', 240)
        self.contact_flags_enabled = rospy.get_param('~general/contact_flags', False)

        if ISREALROBOT == False:
            self.kp = rospy.get_param('~simulation/kp', [30.0, 30.0, 30.0])
            self.kd = rospy.get_param('~simulation/kd', [0.1, 0.1, 0.1])
            self.yaw_kp = rospy.get_param('~simulation/yaw_kp', 0.4)
            self.kinematic_scheme = rospy.get_param('~simulation/kinematic_scheme', 'x')
            self.robot_height = rospy.get_param('~simulation/robot_height', 0.17)
            self.cog_x_offset = rospy.get_param('~simulation/cog_x_offset', 0.0)
            self.cog_y_offset = rospy.get_param('~simulation/cog_y_offset', 0.0)
            self.cog_z_offset = rospy.get_param('~simulation/cog_z_offset', 0.0)
            self.ef_init_x = rospy.get_param('~simulation/ef_init_x', 0.149)
            self.ef_init_y = rospy.get_param('~simulation/ef_init_y', 0.13)
            self.stride_frequency = rospy.get_param('~simulation/stride_frequency', 2.5)
            self.preview_horizon = rospy.get_param('~simulation/preview_horizon', 1.6)
        else:
            self.kp = rospy.get_param('~hardware/kp', [12.0, 12.0, 12.0])
            self.kd = rospy.get_param('~hardware/kd', [0.3, 0.3, 0.3])
            self.yaw_kp = rospy.get_param('~simulation/yaw_kd', 0.5)
            self.kinematic_scheme = rospy.get_param('~hardware/kinematic_scheme', 'x')
            self.robot_height = rospy.get_param('~hardware/robot_height', 0.17)
            self.cog_x_offset = rospy.get_param('~hardware/cog_x_offset', 0.0)
            self.cog_y_offset = rospy.get_param('~hardware/cog_y_offset', 0.0)
            self.cog_z_offset = rospy.get_param('~hardware/cog_z_offset', 0.0)
            self.ef_init_x = rospy.get_param('~hardware/ef_init_x', 0.149)
            self.ef_init_y = rospy.get_param('~hardware/ef_init_y', 0.13)
            self.stride_frequency = rospy.get_param('~hardware/stride_frequency', 2.5)
            self.preview_horizon = rospy.get_param('~hardware/preview_horizon', 1.6)

        self.ros_servo_cmd_topic = rospy.get_param("~topics/ros_servo_cmd_ropic", "joint_cmd")
        self.lcm_servo_cmd_channel = rospy.get_param("~topics/lcm_servo_cmd_channel", "SERVO_CMD")
        self.lcm_servo_state_channel = rospy.get_param("~topics/lcm_servo_state_channel", "SERVO_STATE")

        self.ros_imu_topic = rospy.get_param("~topics/ros_imu_topic", "imu/data")
        self.ros_cmd_vel_topic = rospy.get_param("~topics/ros_cmd_vel_topic", "cmd_vel")
        self.ros_cmd_pose_topic = rospy.get_param("~topics/ros_cmd_pose_topic", "cmd_pose")
        self.ros_cmd_ef_pose_topic = rospy.get_param("~topics/ros_cmd_ef_pose_topic", "ef_position/command")
        self.ros_cmd_joint_pos_topic = rospy.get_param("~topics/ros_cmd_joint_pos_topic", "joint_group_position_controller/command")
        self.ros_state_contact_flags_topic = rospy.get_param("~topics/ros_state_contact_flags_topic", "foot_contacts")
        self.ros_state_ef_pose_topic = rospy.get_param("~topics/ros_state_ef_pose_topic", "ef_position/states")

    def _init_ros(self):
        self.rate = rospy.Rate(self.freq)

        # create publishers
        if self.contact_flags_enabled:
            self.foot_contact_pub = rospy.Publisher(self.ros_state_contact_flags_topic, ContactsStamped, queue_size=10)
        self.ef_pos_pub = rospy.Publisher(self.ros_state_ef_pose_topic, PoseArray, queue_size=10)
        self.js_ref_pos_pub = rospy.Publisher("lcm/servo_cmd", JointTrajectoryPoint, queue_size=10)

        # create subscribers
        rospy.Subscriber(self.ros_imu_topic, Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber(self.ros_cmd_vel_topic, Twist, self.cmd_vel_callback, queue_size=1)
        rospy.Subscriber(self.ros_cmd_pose_topic, Twist, self.cmd_pose_callback, queue_size=1)
        rospy.Subscriber(self.ros_cmd_ef_pose_topic, PoseArray, self.cmd_ef_callback, queue_size=1)
        rospy.Subscriber(self.ros_cmd_joint_pos_topic, JointTrajectoryPoint, self.cmd_joint_callback, queue_size=1)
        rospy.Subscriber("disable", Bool, self.disable_callback, queue_size=1)

        # create messages
        self.foot_contact_msg = ContactsStamped()
        self.ef_poses_msg = PoseArray()
        self.ref_joint_msg = JointTrajectoryPoint()
        
        # init services
        sm = rospy.Service('robot_mode', QuadrupedCmd, self.srv_callback_mode)
        sa = rospy.Service('robot_action', QuadrupedCmd, self.srv_callback_action)
        sh = rospy.Service('stride_height', QuadrupedCmd, self.srv_callback_stride_height)
        skp = rospy.Service('joints_kp', JointsCmd, self.srv_callback_kp)
        skd = rospy.Service('joints_kd', JointsCmd, self.srv_callback_kd)

    def disable_callback(self, msg : Bool):
        self.robot_disabled = msg.data
        self.controller.set_dis_status(msg.data)
        
    def imu_callback(self, msg : Imu):
        # self.base_orient_q = msg.orientation
        quat_df = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        if quat_df[3] != 0:
            quat = Quaternion(quat_df)
            rot = Rotation.from_quat(quat)
            rot_euler = rot.as_euler('xyz', degrees=False)
            if rot_euler[0] < 0:
                rot_euler[0] += np.pi
            else:
                rot_euler[0] -= np.pi
            rot_euler[0] = rot_euler[0] 

            rot_euler[2] = -rot_euler[2] 
            # print(rot_euler[0])
            self.controller.set_euler(rot_euler)
            # print(rot_euler)
        else:
            self.controller.set_euler([0,0,0])

    
    def cmd_vel_callback(self, msg : Twist):
        self.cmd_vel[X] = msg.linear.x
        self.cmd_vel[Y] = msg.linear.y
        self.cmd_vel[Z] = -msg.angular.z
        self.controller.set_cmd_vel(self.cmd_vel)

    def cmd_pose_callback(self, msg : Twist):
        self.cmd_pose[X] = msg.linear.x - self.cog_x_offset
        self.cmd_pose[Y] = msg.linear.y - self.cog_y_offset
        self.cmd_pose[Z] = msg.linear.z

        self.cmd_pose[X+3] = msg.angular.x
        self.cmd_pose[Y+3] = msg.angular.y - 0.02
        self.cmd_pose[Z+3] = msg.angular.z

        self.controller.set_cmd_pose(self.cmd_pose)

    def cmd_ef_callback(self, msg : PoseArray):
        for i in range(len(msg.poses)):
            self.cmd_ef_pos[3*i+0] = msg.poses[i].position.x
            self.cmd_ef_pos[3*i+1] = msg.poses[i].position.y
            self.cmd_ef_pos[3*i+2] = msg.poses[i].position.z
        self.controller.set_cmd_ef_pos(self.cmd_ef_pos)

    def cmd_joint_callback(self, msg : JointTrajectoryPoint):
        if len(msg.positions) == 12 or len(msg.velocities) == 12 or len(msg.effort) == 12: 
            self.controller.set_cmd_joint_pos(msg.positions, msg.velocities, msg.effort)

    def srv_callback_mode(self, req : QuadrupedCmdRequest):
        rospy.loginfo(f"I got mode num: {req.cmd}")
        self.controller.set_mode_num(int(req.cmd))
        return QuadrupedCmdResponse(1, "get the mode")

    def srv_callback_action(self, req : QuadrupedCmdRequest):
        rospy.loginfo(f"I got action num: {req.cmd}")
        self.controller.action_finished = False
        self.controller.set_action_num(int(req.cmd))
        while True:
            if self.controller.is_action_finished():
                return QuadrupedCmdResponse(1, "get the action")

    def srv_callback_stride_height(self, req : QuadrupedCmdRequest):
        rospy.loginfo(f"I got stride height: {req.cmd}")
        self.controller.set_stride_height(req.cmd)
        return QuadrupedCmdResponse(1, "get stride height")

    def srv_callback_kp(self, req : JointsCmdRequest):
        rospy.loginfo(f"I got new kp={req.cmd}")
        # self.ref_kp = req.cmd
        self.controller.set_kp(req.cmd)
        return JointsCmdResponse(1, "get kp")

    def srv_callback_kd(self, req : JointsCmdRequest):
        rospy.loginfo(f"I got new kd={req.cmd}")
        # self.ref_kd = req.cmd
        self.controller.set_kd(req.cmd)
        return JointsCmdResponse(1, "get kd")

    def pub_cur_ef(self, cur_ef):
        self.ef_poses_msg.poses = []
        self.ef_poses_msg.header.stamp = rospy.Time.now()
        self.ef_poses_msg.header.frame_id = "body"
        for i in range(4):
            self.ef_pos_msg = Pose()
            self.ef_pos_msg.position.x = cur_ef[3*i+0]
            self.ef_pos_msg.position.y = cur_ef[3*i+1]
            self.ef_pos_msg.position.z = cur_ef[3*i+2]
            self.ef_poses_msg.poses.append(self.ef_pos_msg)
        self.ef_pos_pub.publish(self.ef_poses_msg)

        
    def loop(self):
        amplitude = [0.3, 0.7, -1.5]
        speed = 1
        t = 0

        while not rospy.is_shutdown():
            t += (1/self.freq)

            # get servo states (pos, vel, torq)
            self.cur_pos, self.cur_vel, self.cur_torq = self.lcm_exch.get_servo_state()

            # step controller
            self.controller.set_cur_joint_pos(self.cur_pos)
            ref_pos, ref_vel, ref_torq, ref_kp, ref_kd = self.controller.step()

            # send cur EF position
            self.cur_ef = self.controller.get_cur_ef(self.cur_pos)
            self.pub_cur_ef(self.cur_ef)

            # send foot contacts
            self.foot_contact_msg.header.stamp = rospy.Time().now()
            self.foot_contact_msg.contacts = self.controller.get_foot_contacts()
            if self.contact_flags_enabled:
                self.foot_contact_pub.publish(self.foot_contact_msg)
            
            # send lcm servo cmd
            
            self.lcm_exch.send_servo_cmd(ref_pos, ref_vel, ref_torq, ref_kp, ref_kd)

            # send ref joint positions
            self.ref_joint_msg.positions = ref_pos
            self.js_ref_pos_pub.publish(self.ref_joint_msg)

            self.rate.sleep()


def main():
    lc_ctrl = Locomotion_Control()
    lc_ctrl.loop()


if __name__ == '__main__':
    main()