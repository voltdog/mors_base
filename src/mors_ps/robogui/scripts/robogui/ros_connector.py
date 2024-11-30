#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, UInt8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray, Pose
from mors.srv import QuadrupedCmd, QuadrupedCmdResponse, JointsCmd, JointsCmdResponse
from sensor_msgs.msg import JointState, Imu, Temperature
# from robo_msgs.msg import robot_state
# from robo_msgs.msg import ll_command
from robogui.data_containers import StateDataDict

from pyrr import Quaternion
from scipy.spatial.transform import Rotation


class CmdData():
    def __init__(self):
        
        # self.mode = 0
        # self.start = 0
        # modes:
        # --- ikine check | mode = 1
        self.ef_x = [0]*4
        self.ef_y = [0]*4
        self.ef_z = [0]*4
        # --- body pose control | mode = 2
        self.body_x = 0
        self.body_y = 0
        self.body_z = 0
        self.body_wx = 0
        self.body_wy = 0
        self.body_wz = 0
        # --- robot walking mode | mode = 0
        self.robot_dx = 0
        self.robot_dy = 0
        self.robot_dz = 0
        # self.robot_step_freq = 0
        # self.robot_step_lenght = 0
        # self.robot_step_height = 0
        # self.robot_gait_type = 0
        # --- servo check mode | mode = 3
        self.joint_pos = [0]*12
        self.joint_vel = [0]*12
        self.joint_torq = [0]*12
        # self.joint_kp = [0]*12
        # self.joint_kd = [0]*12
        # --- cute action mode | mode = 4
        # self.cute_action = 0

class ROSParams():
    def __init__(self) -> None:
        self.max_speed_x = 0.0
        self.max_speed_y = 0.0
        self.max_speed_z = 0.0

        self.max_angle_x = 0.0
        self.max_angle_y = 0.0
        self.max_angle_z = 0.0

        self.max_lin_x = 0.0
        self.max_lin_y = 0.0
        self.max_lin_z = 0.0
        self.min_lin_z = 0.0

        self.ef_max_pos_x = 0.0
        self.ef_max_pos_y = 0.0
        self.ef_max_pos_z = 0.0
        self.ef_min_pos_z = 0.0

        self.max_abad = 0.0
        self.max_hip = 0.0
        self.max_knee = 0.0
        self.max_joint_vel = 0.0
        self.max_joint_torq = 0.0
        self.max_joint_kp = 0.0
        self.max_joint_kd = 0.0

        self.walk_mode = 0
        self.ef_mode = 0
        self.body_mode = 0
        self.joint_mode = 0


class ROS_Connector():
    def __init__(self):
        self.__kill = False
 
        rospy.init_node("robogui")

        self.gui_status_msg = Bool()
        self.cmd_data = CmdData()
        # self.state_data = StateData()
        self.state_data = StateDataDict()

        #load parameters
        self.prm = ROSParams()
        freq = rospy.get_param('~general/frequency', 40)

        cmd_vel_topic = rospy.get_param('~cmd_topics/cmd_vel', "gui/cmd_vel")
        cmd_pose_topic = rospy.get_param('~cmd_topics/cmd_pose', "gui/cmd_pose")
        cmd_ef_pos_topic = rospy.get_param('~cmd_topics/cmd_ef_pose', "gui/ef_position/command")
        cmd_joint_pos_topic = rospy.get_param('~cmd_topics/cmd_joint_pos', "gui/joint_group_position_controller/command")
        gui_status_topic = rospy.get_param('~cmd_topics/status', "gui/status")

        cur_device_topic = rospy.get_param("~state_topics/cur_device", "/cur_device")
        # joint_state_topic = rospy.get_param("/topics/joint_state", "/topics/joint_state")
        state_imu_data_topic = rospy.get_param("~state_topics/imu_data", "imu/data")
        state_temperature_topic = rospy.get_param("~state_topics/temperature_data", "imu/temp")
        state_joint_topic = rospy.get_param("~state_topics/joint_state", "joint_states")
        state_cmd_vel_topic = rospy.get_param("~state_topics/cmd_vel", "cmd_vel")
        state_cmd_pose_topic = rospy.get_param("~state_topics/cmd_pose", "cmd_pose")
        state_cmd_ef_pose_topic = rospy.get_param("~state_topics/cmd_ef_pose", "ef_position/command")
        state_cmd_joint_pos_topic = rospy.get_param("~state_topics/cmd_joint_pos", "joint_group_position_controller/command")
        state_ef_pose_topic = rospy.get_param("~state_topics/ef_pose", "ef_position/states")
        ll_cmd_joint_pos_topic = rospy.get_param("~state_topics/ll_cmd_joint_pos", "lcm/servo_cmd")

        self.prm.max_speed_x = rospy.get_param('~gait/max_speed_x', 0.3)
        self.prm.max_speed_y = rospy.get_param('~gait/max_speed_y', 0.2)
        self.prm.max_speed_z = rospy.get_param('~gait/max_speed_z', 0.8)

        self.prm.max_angle_x = rospy.get_param('~pose/max_angle_x', 0.33)
        self.prm.max_angle_y = rospy.get_param('~pose/max_angle_y', 0.33)
        self.prm.max_angle_z = rospy.get_param('~pose/max_angle_z', 0.33)
        self.prm.max_lin_x = rospy.get_param('~pose/max_lin_x', 0.1)
        self.prm.max_lin_y = rospy.get_param('~pose/max_lin_y', 0.1)
        self.prm.max_lin_z = rospy.get_param('~pose/max_lin_z', 0.1)
        self.prm.min_lin_z = rospy.get_param('~pose/min_lin_z', 0.1)

        self.prm.ef_max_pos_x = rospy.get_param('~ef/max_pos_x', 0.05)
        self.prm.ef_max_pos_y = rospy.get_param('~ef/max_pos_y', 0.05)
        self.prm.ef_max_pos_z = rospy.get_param('~ef/max_pos_z', 0.05)
        self.prm.ef_min_pos_z = rospy.get_param('~ef/min_pos_z', 0.05)

        self.prm.max_abad = rospy.get_param('~joints/max_abad', 1.57)
        self.prm.max_hip = rospy.get_param('~joints/max_hip', 1.57)
        self.prm.max_knee = rospy.get_param('~joints/max_knee', 1.57)
        self.prm.max_joint_vel = rospy.get_param('~joints/max_vel', 2.0)
        self.prm.max_joint_torq = rospy.get_param('~joints/max_torq', 2.0)
        self.prm.max_joint_kp = rospy.get_param('~joints/max_kp', 20.0)
        self.prm.max_joint_kd = rospy.get_param('~joints/max_kd', 2.0)

        self.prm.walk_mode = rospy.get_param('~modes/walk', 0)
        self.prm.ef_mode = rospy.get_param('~modes/ef', 1)
        self.prm.body_mode = rospy.get_param('~modes/body', 2)
        self.prm.joint_mode = rospy.get_param('~modes/joint', 3)
        
        self.rate = rospy.Rate(freq)
        
        # publishers
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.cmd_pose_pub = rospy.Publisher(cmd_pose_topic, Twist, queue_size=10)
        self.cmd_ef_pos_pub = rospy.Publisher(cmd_ef_pos_topic, PoseArray, queue_size=10)
        self.cmd_joint_pos_pub = rospy.Publisher(cmd_joint_pos_topic, JointTrajectoryPoint, queue_size=10)
        self.status_pub = rospy.Publisher(gui_status_topic, Bool, queue_size=10)

        rospy.Subscriber(cur_device_topic, UInt8, self.cur_device_callback, queue_size=1)
        rospy.Subscriber(state_joint_topic, JointState, self.servo_state_callback, queue_size=1)
        rospy.Subscriber(state_imu_data_topic, Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber(state_temperature_topic, Temperature, self.temperature_callback, queue_size=1)

        rospy.Subscriber(state_cmd_vel_topic, Twist, self.state_cmd_vel_callback, queue_size=1)
        rospy.Subscriber(state_cmd_pose_topic, Twist, self.state_cmd_pose_callback, queue_size=1)
        rospy.Subscriber(state_cmd_ef_pose_topic, PoseArray, self.state_cmd_ef_callback, queue_size=1)
        rospy.Subscriber(state_ef_pose_topic, PoseArray, self.state_ef_callback, queue_size=1)
        rospy.Subscriber(state_cmd_joint_pos_topic, JointTrajectoryPoint, self.state_cmd_joint_callback, queue_size=1)
        rospy.Subscriber(ll_cmd_joint_pos_topic, JointTrajectoryPoint, self.ll_cmd_joint_callback, queue_size=1)

        # messages
        self.cmd_vel_msg = Twist()
        self.cmd_pose_msg = Twist()
        self.cmd_ef_msg = PoseArray()
        cmd_one_ef_msg = Pose()
        for _ in range(4):
            self.cmd_ef_msg.poses.append(cmd_one_ef_msg)
        self.cmd_joint_msg = JointTrajectoryPoint()
        self.cmd_joint_msg.velocities = [0]*12
        self.cmd_joint_msg.effort = [0]*12
        self.cmd_joint_msg.positions = [0]*12

        

        # different data
        self.cur_device_data = 0
        self.foot_signals = ["Foot R1 X Pos", "Foot R1 Y Pos", "Foot R1 Z Pos", 
                        "Foot L1 X Pos", "Foot L1 Y Pos", "Foot L1 Z Pos", 
                        "Foot R2 X Pos", "Foot R2 Y Pos", "Foot R2 Z Pos", 
                        "Foot L2 X Pos", "Foot L2 Y Pos", "Foot L2 Z Pos"]

        rospy.loginfo("robogui configured")

    def set_mode(self, mode):
        rospy.wait_for_service('robot_mode')
        try:
            set_mode_srv = rospy.ServiceProxy('robot_mode', QuadrupedCmd)
            resp = set_mode_srv(mode)
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # call robot_action service
    def set_action(self, action):
        rospy.wait_for_service('robot_action')
        try:
            set_action_srv = rospy.ServiceProxy('robot_action', QuadrupedCmd)
            resp = set_action_srv(action)
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # call stride_height service
    def set_stride_height(self, height):
        rospy.wait_for_service('stride_height')
        try:
            set_stride_height_srv = rospy.ServiceProxy('stride_height', QuadrupedCmd)
            resp = set_stride_height_srv(height)
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # call joints_kpkd service
    def set_joints_kp(self, kp):
        rospy.wait_for_service("joints_kp")
        try:
            set_kp_srv = rospy.ServiceProxy('joints_kp', JointsCmd)
            resp = set_kp_srv(kp)
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def set_joints_kd(self, kd):
        rospy.wait_for_service("joints_kd")
        try:
            set_kd_srv = rospy.ServiceProxy('joints_kd', JointsCmd)
            resp = set_kd_srv(kd)
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def cur_device_callback(self, msg: UInt8):
        self.cur_device_data = msg.data

    def servo_state_callback(self, msg : JointState):
        for i in range(12):
            self.state_data.data["Joint States"][f"Joint Pos {i+1}"] = msg.position[i]
            self.state_data.data["Joint States"][f"Joint Vel {i+1}"] = msg.velocity[i]
            self.state_data.data["Joint States"][f"Joint Torq {i+1}"] = msg.effort[i]

    def imu_callback(self, msg : Imu):
        self.state_data.data["Body States"]["Body Quat X"] = msg.orientation.x
        self.state_data.data["Body States"]["Body Quat Y"] = msg.orientation.y
        self.state_data.data["Body States"]["Body Quat Z"] = msg.orientation.z
        self.state_data.data["Body States"]["Body Quat W"] = msg.orientation.w

        self.state_data.data["Body States"]["Body Ang Vel X"] = msg.angular_velocity.x
        self.state_data.data["Body States"]["Body Ang Vel Y"] = msg.angular_velocity.y
        self.state_data.data["Body States"]["Body Ang Vel Z"] = msg.angular_velocity.z

        self.state_data.data["Body States"]["Body Acc X"] = msg.linear_acceleration.x
        self.state_data.data["Body States"]["Body Acc Y"] = msg.linear_acceleration.y
        self.state_data.data["Body States"]["Body Acc Z"] = msg.linear_acceleration.z

        quat_df = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        quat = Quaternion(quat_df).inverse
        rot = Rotation.from_quat(quat)
        rot_euler = rot.as_euler('xyz', degrees=False)

        self.state_data.data["Body States"]["Body Euler X"] = rot_euler[0]
        self.state_data.data["Body States"]["Body Euler Y"] = rot_euler[1]
        self.state_data.data["Body States"]["Body Euler Z"] = rot_euler[2]

    def temperature_callback(self, msg : Temperature):
        self.state_data.data["Body States"]["Temperature"] = msg.temperature

    def state_cmd_vel_callback(self, msg : Twist):
        self.state_data.data["Desired Values"]["Ref Robot X vel"] = msg.linear.x
        self.state_data.data["Desired Values"]["Ref Robot Y vel"] = msg.linear.y
        self.state_data.data["Desired Values"]["Ref Robot Z turn"] = msg.angular.z

    def state_cmd_pose_callback(self, msg : Twist):
        self.state_data.data["Desired Values"]["Ref Body X Pos"] = msg.linear.x
        self.state_data.data["Desired Values"]["Ref Body Y Pos"] = msg.linear.y
        self.state_data.data["Desired Values"]["Ref Body Z Pos"] = msg.linear.z
        self.state_data.data["Desired Values"]["Ref Body X Angle"] = msg.angular.x
        self.state_data.data["Desired Values"]["Ref Body Y Angle"] = msg.angular.y
        self.state_data.data["Desired Values"]["Ref Body Z Angle"] = msg.angular.z

    def state_cmd_ef_callback(self, msg : PoseArray):
        if len(msg.poses) >= 4:
            self.state_data.data["Desired Values"]["Ref Foot R1 X Pos"] = msg.poses[0].position.x
            self.state_data.data["Desired Values"]["Ref Foot R1 Y Pos"] = msg.poses[0].position.y
            self.state_data.data["Desired Values"]["Ref Foot R1 Z Pos"] = msg.poses[0].position.z

            self.state_data.data["Desired Values"]["Ref Foot L1 X Pos"] = msg.poses[1].position.x
            self.state_data.data["Desired Values"]["Ref Foot L1 Y Pos"] = msg.poses[1].position.y
            self.state_data.data["Desired Values"]["Ref Foot L1 Z Pos"] = msg.poses[1].position.z

            self.state_data.data["Desired Values"]["Ref Foot R2 X Pos"] = msg.poses[2].position.x
            self.state_data.data["Desired Values"]["Ref Foot R2 Y Pos"] = msg.poses[2].position.y
            self.state_data.data["Desired Values"]["Ref Foot R2 Z Pos"] = msg.poses[2].position.z

            self.state_data.data["Desired Values"]["Ref Foot L2 X Pos"] = msg.poses[3].position.x
            self.state_data.data["Desired Values"]["Ref Foot L2 Y Pos"] = msg.poses[3].position.y
            self.state_data.data["Desired Values"]["Ref Foot L2 Z Pos"] = msg.poses[3].position.z

    def state_cmd_joint_callback(self, msg : JointTrajectoryPoint):
        for i in range(12):
            if len(msg.positions) >= 12:
                self.state_data.data["Desired Values"][f"Ref Joint Pos {i+1}"] = msg.positions[i]
            else:
                self.state_data.data["Desired Values"][f"Ref Joint Pos {i+1}"] = 0
            if len(msg.velocities) >= 12:
                self.state_data.data["Desired Values"][f"Ref Joint Vel {i+1}"] = msg.velocities[i]
            else:
                self.state_data.data["Desired Values"][f"Ref Joint Vel {i+1}"] = 0
            if len(msg.effort) >= 12:
                self.state_data.data["Desired Values"][f"Ref Joint Torq {i+1}"] = msg.effort[i]
            else:
                self.state_data.data["Desired Values"][f"Ref Joint Torq {i+1}"] = 0

    def state_joint_topic(self, msg : JointState):
        for i in range(12):
            self.state_data.data["Joint States"][f"Joint Pos {i+1}"] = msg.position[i]
            self.state_data.data["Joint States"][f"Joint Vel {i+1}"] = msg.velocity[i]
            self.state_data.data["Joint States"][f"Joint Torq {i+1}"] = msg.effort[i]

    def state_ef_callback(self, msg : PoseArray):
        if len(msg.poses) >= 4:
            self.state_data.data["Foot Positions"][self.foot_signals[0]] = msg.poses[0].position.x
            self.state_data.data["Foot Positions"][self.foot_signals[1]] = msg.poses[0].position.y
            self.state_data.data["Foot Positions"][self.foot_signals[2]] = msg.poses[0].position.z

            self.state_data.data["Foot Positions"][self.foot_signals[3]] = msg.poses[1].position.x
            self.state_data.data["Foot Positions"][self.foot_signals[4]] = msg.poses[1].position.y
            self.state_data.data["Foot Positions"][self.foot_signals[5]] = msg.poses[1].position.z

            self.state_data.data["Foot Positions"][self.foot_signals[6]] = msg.poses[2].position.x
            self.state_data.data["Foot Positions"][self.foot_signals[7]] = msg.poses[2].position.y
            self.state_data.data["Foot Positions"][self.foot_signals[8]] = msg.poses[2].position.z

            self.state_data.data["Foot Positions"][self.foot_signals[9]] = msg.poses[3].position.x
            self.state_data.data["Foot Positions"][self.foot_signals[10]] = msg.poses[3].position.y
            self.state_data.data["Foot Positions"][self.foot_signals[11]] = msg.poses[3].position.z

    def ll_cmd_joint_callback(self, msg : JointTrajectoryPoint):
        
        for i in range(12):
            if len(msg.positions) >= 12:
                self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Pos {i+1}"] = msg.positions[i]
            else:
                self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Pos {i+1}"] = 0
            if len(msg.velocities) >= 12:
                self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Vel {i+1}"] = msg.velocities[i]
            else:
                self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Vel {i+1}"] = 0
            if len(msg.effort) >= 12:
                self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Torq {i+1}"] = msg.effort[i]
            else:
                self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Torq {i+1}"] = 0

    # def robot_state_callback(self, msg: robot_state):

    #     self.state_data.data["Desired Values"]["Mode"] = msg.mode
    #     self.state_data.data["Desired Values"]["Start"] = msg.start
    #     self.state_data.data["Desired Values"]["Ref Robot X vel"] = msg.ref_robot_dx
    #     self.state_data.data["Desired Values"]["Ref Robot Y vel"] = msg.ref_robot_dy
    #     self.state_data.data["Desired Values"]["Ref Robot Z turn"] = msg.ref_robot_dz
    #     self.state_data.data["Desired Values"]["Ref Robot Stride Freq"] = msg.ref_robot_step_freq
    #     self.state_data.data["Desired Values"]["Ref Robot Stride Length"] = msg.ref_robot_step_lenght
    #     self.state_data.data["Desired Values"]["Ref Foot R1 X Pos"] = msg.ref_ef_x[0]
    #     self.state_data.data["Desired Values"]["Ref Foot L1 X Pos"] = msg.ref_ef_x[1]
    #     self.state_data.data["Desired Values"]["Ref Foot R2 X Pos"] = msg.ref_ef_x[2]
    #     self.state_data.data["Desired Values"]["Ref Foot L2 X Pos"] = msg.ref_ef_x[3]
    #     self.state_data.data["Desired Values"]["Ref Foot R1 Y Pos"] = msg.ref_ef_y[0]
    #     self.state_data.data["Desired Values"]["Ref Foot L1 Y Pos"] = msg.ref_ef_y[1]
    #     self.state_data.data["Desired Values"]["Ref Foot R2 Y Pos"] = msg.ref_ef_y[2]
    #     self.state_data.data["Desired Values"]["Ref Foot L2 Y Pos"] = msg.ref_ef_y[3]
    #     self.state_data.data["Desired Values"]["Ref Foot R1 Z Pos"] = msg.ref_ef_z[0]
    #     self.state_data.data["Desired Values"]["Ref Foot L1 Z Pos"] = msg.ref_ef_z[1]
    #     self.state_data.data["Desired Values"]["Ref Foot R2 Z Pos"] = msg.ref_ef_z[2]
    #     self.state_data.data["Desired Values"]["Ref Foot L2 Z Pos"] = msg.ref_ef_z[3]
    #     self.state_data.data["Desired Values"]["Ref Body X Pos"] = msg.ref_body_x
    #     self.state_data.data["Desired Values"]["Ref Body Y Pos"] = msg.ref_body_y
    #     self.state_data.data["Desired Values"]["Ref Body Z Pos"] = msg.ref_body_z
    #     self.state_data.data["Desired Values"]["Ref Body X Angle"] = msg.ref_body_wx
    #     self.state_data.data["Desired Values"]["Ref Body Y Angle"] = msg.ref_body_wy
    #     self.state_data.data["Desired Values"]["Ref Body Z Angle"] = msg.ref_body_wz
    #     self.state_data.data["Desired Values"]["Cute Action"] = msg.cute_action
    #     for i in range(12):
    #         self.state_data.data["Desired Values"][f"Ref Joint Pos {i+1}"] = msg.ref_joint_pos[i]
    #         self.state_data.data["Desired Values"][f"Ref Joint Vel {i+1}"] = msg.ref_joint_pos[i]
    #         self.state_data.data["Desired Values"][f"Ref Joint Torq {i+1}"] = msg.ref_joint_pos[i]
    #         self.state_data.data["Desired Values"][f"Ref Joint Kp {i+1}"] = msg.ref_joint_pos[i]
    #         self.state_data.data["Desired Values"][f"Ref Joint Kd {i+1}"] = msg.ref_joint_pos[i]

    #         self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Pos {i+1}"] = msg.hl_ref_joint_position[i]
    #         self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Vel {i+1}"] = msg.hl_ref_joint_velocity[i]
    #         self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Torq {i+1}"] = msg.hl_ref_joint_torque[i]
    #         self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Kp {i+1}"] = msg.hl_ref_joint_kp[i]
    #         self.state_data.data["Commands to HL from LL"][f"LL Ref Joint Kd {i+1}"] = msg.hl_ref_joint_kd[i]

    #         self.state_data.data["Joint States"][f"Joint Pos {i+1}"] = msg.cur_joint_angle[i]
    #         self.state_data.data["Joint States"][f"Joint Vel {i+1}"] = msg.cur_joint_vel[i]
    #         self.state_data.data["Joint States"][f"Joint Torq {i+1}"] = msg.cur_joint_torque[i]

    #     self.state_data.data["Body States"]["Body Euler X"] = msg.cur_body_euler[0]
    #     self.state_data.data["Body States"]["Body Euler Y"] = msg.cur_body_euler[1]
    #     self.state_data.data["Body States"]["Body Euler Z"] = msg.cur_body_euler[2]
    #     self.state_data.data["Body States"]["Body Acc X"] = msg.cur_body_acc[0]
    #     self.state_data.data["Body States"]["Body Acc Y"] = msg.cur_body_acc[1]
    #     self.state_data.data["Body States"]["Body Acc Z"] = msg.cur_body_acc[2]

    #     for i in range(4):
    #         self.state_data.data["Foot Positions"][self.foot_signals[3*i]] = msg.cur_ef_x[i]
    #         self.state_data.data["Foot Positions"][self.foot_signals[3*i+1]] = msg.cur_ef_y[i]
    #         self.state_data.data["Foot Positions"][self.foot_signals[3*i+2]] = msg.cur_ef_z[i]

    #     # rospy.loginfo(self.state_data.data["Joint States"][f"Joint Pos 2"])

    def get_data(self):
        # print(self.cur_device_data)
        return self.state_data, self.cur_device_data
        # pass

    def get_params(self):
        return self.prm

    def set_data(self, data: CmdData):
        self.cmd_vel_msg.linear.x = data.robot_dx
        self.cmd_vel_msg.linear.y = data.robot_dy
        self.cmd_vel_msg.angular.z = data.robot_dz

        self.cmd_pose_msg.linear.x = data.body_x
        self.cmd_pose_msg.linear.y = data.body_y
        self.cmd_pose_msg.linear.z = data.body_z
        self.cmd_pose_msg.angular.x = data.body_wx
        self.cmd_pose_msg.angular.y = data.body_wy
        self.cmd_pose_msg.angular.z = data.body_wz

        # self.cmd_ef_msg.poses.clear()
        for i in range(4):
            cmd_one_ef_msg = Pose()
            cmd_one_ef_msg.position.x = data.ef_x[i]
            cmd_one_ef_msg.position.y = data.ef_y[i]
            cmd_one_ef_msg.position.z = data.ef_z[i]
            # self.cmd_ef_msg.poses.append(cmd_one_ef_msg)
            self.cmd_ef_msg.poses[i] = cmd_one_ef_msg

        self.cmd_joint_msg.positions = data.joint_pos[:]
        self.cmd_joint_msg.velocities = data.joint_vel[:]
        self.cmd_joint_msg.effort = data.joint_torq[:]

        # self.cmd_msg.mode = data.mode
        # self.cmd_msg.start = data.start

        # self.cmd_msg.ef_x = data.ef_x[:]
        # self.cmd_msg.ef_y = data.ef_y[:]
        # self.cmd_msg.ef_z = data.ef_z[:]

        # self.cmd_msg.body_x = data.body_x
        # self.cmd_msg.body_y = data.body_y
        # self.cmd_msg.body_z = data.body_z
        # self.cmd_msg.body_wx = data.body_wx
        # self.cmd_msg.body_wy = data.body_wy
        # self.cmd_msg.body_wz = data.body_wz

        # self.cmd_msg.robot_dx = data.robot_dx
        # self.cmd_msg.robot_dy = data.robot_dy
        # self.cmd_msg.robot_dz = data.robot_dz
        # self.cmd_msg.robot_step_freq = data.robot_step_freq
        # self.cmd_msg.robot_step_height = data.robot_step_height
        # self.cmd_msg.robot_step_lenght = data.robot_step_lenght
        # self.cmd_msg.robot_gait_type = data.robot_gait_type

        # self.cmd_msg.joint_pos = data.joint_pos[:]
        # self.cmd_msg.joint_vel = data.joint_vel[:]
        # self.cmd_msg.joint_torq = data.joint_torq[:]
        # self.cmd_msg.joint_kp = data.joint_kp[:]
        # self.cmd_msg.joint_kd = data.joint_kd[:]

        # self.cmd_msg.cute_action = data.cute_action

    def kill(self):
        self.__kill = True

    def loop(self):
        while not rospy.is_shutdown():
            
            if self.__kill == False:
                # rospy.loginfo("Hello")
                self.gui_status_msg.data = True
            
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                self.cmd_pose_pub.publish(self.cmd_pose_msg)
                self.cmd_ef_pos_pub.publish(self.cmd_ef_msg)
                self.cmd_joint_pos_pub.publish(self.cmd_joint_msg)
                self.status_pub.publish(self.gui_status_msg)
                self.rate.sleep()
            else:
                self.gui_status_msg.data = False
                self.status_pub.publish(self.gui_status_msg)
                break

            