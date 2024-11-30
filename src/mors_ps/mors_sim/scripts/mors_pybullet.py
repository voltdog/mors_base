#!/usr/bin/env python

from cgitb import reset
from sys import flags
import pybullet as p
import time
from threading import Thread
import pybullet_data
import lcm
from lcm_msgs2.servo_cmd_msg import servo_cmd_msg
from lcm_msgs2.servo_state_msg import servo_state_msg 
import configparser
from mors_sim.mors_gym_env import MorsBulletEnv
import numpy as np
import rospy
import tf2_ros
from PIL import Image as pil

from whole_body_state_msgs.msg import WholeBodyState
from whole_body_state_msgs.msg import JointState as WBJointState
from whole_body_state_msgs.msg import ContactState as WBContactState
from sensor_msgs.msg import Image, Imu, JointState, PointCloud2, PointField, LaserScan, CameraInfo
from champ_msgs.msg import ContactsStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist, Point
from rosgraph_msgs.msg import Clock

class Hardware_Level_Sim(object):
    def __init__(self):
        rospy.init_node("mors_pybullet")

        self.ref_pos = [0]*12
        self.ref_data = [0]*12
        self.init_ref_pos = [0]*12
        self.ref_vel = [0]*12
        self.ref_torq = [0]*12
        self.kp = [0]*12
        self.kd = [0]*12
        self.joint_dir = [-1]*12
        self.joint_offset = [0]*12
        self.theta_ref = [0]*12

        self.read_config()

        # init ROS publishers
        self.imu_data_pub = rospy.Publisher(self.ros_imu_topic, Imu, queue_size=10) # #
        if self.foot_contacts:
            self.contact_pub = rospy.Publisher(self.ros_contact_flags_topic, ContactsStamped, queue_size=10) # #
        self.odom_pub = rospy.Publisher(self.ros_robot_odom_topic, Odometry, queue_size=10) #

        if self.ros_whole_body_state_enabled:
            self.wbc_state_pub = rospy.Publisher(self.ros_whole_body_state_topic, WholeBodyState, queue_size=10) # #

        if self.camera_enabled:
            self.rgb_image_pub = rospy.Publisher(self.camera_rgb_topic, Image, queue_size=10) # #
            self.rgb_info_pub = rospy.Publisher(self.camera_rgb_info_topic, CameraInfo, queue_size=10) # #
            self.depth_image_pub = rospy.Publisher(self.camera_depth_image_topic, Image, queue_size=10) # #
            self.depth_info_pub = rospy.Publisher(self.camera_depth_info_topic, CameraInfo, queue_size=10) # #
            if self.pointcloud_enabled:
                self.depth_points_pub = rospy.Publisher(self.camera_depth_points_topic, PointCloud2, queue_size=10) #

        if self.lidar_enabled:
            self.lidar_pub = rospy.Publisher(self.lidar_topic, LaserScan, queue_size=10) #

        self.js_pub = rospy.Publisher(self.ros_joint_states_topic, JointState, queue_size=30) # #
        self.vel_pub = rospy.Publisher("robot_velocity", Point, queue_size=10)

        # because of SLAM
        if self.ros_whole_body_state_enabled:
            self.robot_tf = tf2_ros.TransformBroadcaster() #
        self.clock_pub = rospy.Publisher(self.clock_topic, Clock, queue_size=10)

        # init ROS messages
        self.imu_msg = Imu()
        self.js_msg = JointState()
        self.wbs = WholeBodyState()
        self.odom = Odometry()
        self.img_rgb_msg = Image()
        self.img_depth_msg = Image()
        self.contact_msg = ContactsStamped()
        self.lidar_msg = LaserScan()
        self.pointcloud_msg = PointCloud2()
        self.robot_vel_msg = Point()
        self.clock_msg = Clock()
        # self.clock_msg.clock.secs = rospy.Time.now().to_sec()
        self.zero_time = rospy.get_time()

        if self.camera_enabled:
            self.cam_info_msg = CameraInfo()
            self.cam_info_msg.header.stamp = rospy.Time.now()
            self.cam_info_msg.header.frame_id = self.camera_frame
            self.cam_info_msg.height = self.pixel_height
            self.cam_info_msg.width = self.pixel_width
            self.cam_info_msg.distortion_model = "plumb_bob"
            self.cam_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.cam_info_msg.K = [524.2422531097977, 0.0, 320.5, 0.0, 524.2422531097977, 240.5, 0.0, 0.0, 1.0]
            self.cam_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            self.cam_info_msg.P = [524.2422531097977, 0.0, 320.5, -0.0, 0.0, 524.2422531097977, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.cam_info_msg.binning_x = 0
            self.cam_info_msg.binning_y = 0
            self.cam_info_msg.roi.x_offset = 0
            self.cam_info_msg.roi.y_offset = 0
            self.cam_info_msg.roi.height = 0
            self.cam_info_msg.roi.width = 0
            self.cam_info_msg.roi.do_rectify = False

            self.img_rgb_msg.header.frame_id = self.camera_frame
            self.img_rgb_msg.width = self.pixel_width
            self.img_rgb_msg.height = self.pixel_height
            self.img_rgb_msg.encoding = "rgb8"
            self.img_rgb_msg.step = self.pixel_width

            self.img_depth_msg.header.frame_id = self.camera_frame
            self.img_depth_msg.width = self.pixel_width
            self.img_depth_msg.height = self.pixel_height
            self.img_depth_msg.encoding = "32FC1"
            self.img_depth_msg.step = self.pixel_width

        if self.lidar_enabled:
            self.lidar_msg.header.frame_id = self.lidar_frame
            self.lidar_msg.angle_min = self.lidar_angle_min
            self.lidar_msg.angle_max = self.lidar_angle_max
            self.lidar_msg.angle_increment = (np.abs(self.lidar_angle_min) + np.abs(self.lidar_angle_max)) / self.lidar_point_num#self.lidar_angle_increment
            self.lidar_msg.scan_time = 1/self.lidar_freq
            self.lidar_msg.time_increment = self.lidar_time_increment
            self.lidar_msg.range_min = self.lidar_range_min
            self.lidar_msg.range_max = self.lidar_range_max
            self.lidar_msg.intensities = [0]*self.lidar_point_num

        self.imu_msg.orientation_covariance = [2.603e-07, 0.0, 0.0, 0.0, 2.603e-07, 0.0, 0.0, 0.0, 0.0]
        self.imu_msg.angular_velocity_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
        self.imu_msg.linear_acceleration_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]

        # init LCM thread
        self.cmd_th = Thread(target=self.get_cmd, args=())
        self.cmd_th.daemon = True
        self.cmd_th.start()

        # init LCM for states
        self.servo_state_msg = servo_state_msg()
        # self.imu_msg = imu_lcm_data()
        self.lc_servo_state = lcm.LCM()
        
        self.sim_it = 0
        self.init_simulation()
        self.set_start_position()

        if self.camera_enabled:
            cam_th = Thread(target=self.camera_loop, args=())
            cam_th.daemon = True
            cam_th.start()

        self.body_quaternion = [0,0,0,1]
        self.body_lin_pos = [0]*3
        self.body_ang_vel = [0]*3
        self.body_lin_vel = np.array([0]*3, float)
        self.body_lin_vel_prev = np.array([0]*3, float)
        self.body_lin_acc = np.array([0]*3, float)
        self.force_dir = 1

        self.imu_data = [0]*10

        self.leg_data = {}
        self.leg_data["pos"] = [0] * 12
        self.leg_data["vel"] = [0] * 12
        self.leg_data["torq"] = [0] * 12
        self.leg_data["name"] = [""] * 12

        self.leg_data_transform = [ 1,  -1,  -1, 
                                    -1,  -1,  -1,
                                    1,  1,  1,
                                    -1,  1,  1]
        self.leg_data_order = [6, 8, 7, 9, 11, 10, 0, 2, 1, 3, 5, 4]
        # self.leg_data_order = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

    def read_config(self):
        self.sim_freq = rospy.get_param('~frequency', 240)
        self.urdf_root = rospy.get_param("~urdf_root", "./urdf")
        self.world_name = rospy.get_param("~world", "empty")
        self.render = rospy.get_param("~render", True)
        self.on_rack = rospy.get_param("~on_rack", True)
         
        self.camera_enabled = rospy.get_param("~camera", False)
        if self.camera_enabled:
            self.camera_frame = rospy.get_param("~camera_frame", "camera_frame")
            self.camera_freq = rospy.get_param("~camera_freq", 20)
            self.pixel_width = rospy.get_param("~pixel_width", 320)
            self.pixel_height = rospy.get_param("~pixel_height", 240)
            self.camera_rgb_topic = rospy.get_param("~camera_rgb_topic", "/camera/rgb")
            self.camera_rgb_info_topic = rospy.get_param("~camera_rgb_info_topic", "camera/rgb/camera_info")
            self.camera_depth_image_topic = rospy.get_param("~camera_depth_image_topic", "/camera/depth/image_raw")
            self.camera_depth_points_topic = rospy.get_param("~camera_depth_points_topic", "/camera/depth/points")
            self.camera_depth_info_topic = rospy.get_param("~camera_depth_info_topic", "camera/depth/camera_info")
            self.pointcloud_enabled = rospy.get_param("~pointcloud_enabled", False)

        self.lidar_enabled = rospy.get_param("~lidar", False)
        if self.lidar_enabled:
            self.lidar_render = rospy.get_param("~lidar_render", False)
            self.lidar_angle_min = rospy.get_param("~lidar_angle_min", 2.2689)
            self.lidar_angle_max = rospy.get_param("~lidar_angle_max", -2.2689)
            self.lidar_freq = rospy.get_param("~lidar_freq", 20)
            self.lidar_point_num = rospy.get_param("~lidar_point_num", 360)
            self.lidar_range_min = rospy.get_param("~lidar_range_min", 0.25)
            self.lidar_range_max = rospy.get_param("~lidar_range_max", 5)
            self.lidar_time_increment = rospy.get_param("~lidar_time_increment", 0.0)
            self.lidar_topic = rospy.get_param("~lidar_topic", "/scan")
            self.lidar_frame = rospy.get_param("~lidar_frame", "scan_frame")

        self.lateral_friction = rospy.get_param("~lateral_friction", 1.0)
        self.spinning_friction = rospy.get_param("~spinning_friction", 0.0065)

        self.accurate_motor_model_enabled = rospy.get_param("~accurate_motor_model_enabled", False)
        self.simple_motor_model_enabled = rospy.get_param("~simple_motor_model_enabled", True)
        self.torque_control_enabled = rospy.get_param("~torque_control_enabled", False)

        self.external_disturbance_enabled = rospy.get_param("~external_disturbance", False)
        self.external_disturbance_value = rospy.get_param("~external_disturbance_value", 2000)
        self.external_disturbance_duration = rospy.get_param("~external_disturbance_duration", 0.001)
        self.external_disturbance_interval = rospy.get_param("~external_disturbance_interval", 2)

        self.ros_whole_body_state_enabled = rospy.get_param("~ros_whole_body_state", False)
        self.ros_whole_body_state_topic = rospy.get_param("~ros_whole_body_state_topic", "whole_body_state")
        self.ros_imu_topic = rospy.get_param("~ros_imu_topic", "imu/data")

        self.ros_joint_states_topic = rospy.get_param("~ros_joint_states_topic", "joint_states")
        self.ros_robot_odom_topic = rospy.get_param("~ros_robot_odom_topic", "robot_odom")
        self.lcm_servo_cmd_channel = rospy.get_param('~lcm_servo_cmd_channel', "SERVO_CMD")
        self.lcm_servo_state_channel = rospy.get_param('~lcm_servo_state_channel', "SERVO_STATE")
        self.clock_topic = rospy.get_param("~clock_topic", "clock")

        self.foot_contacts = rospy.get_param("~foot_contacts", True)
        if self.foot_contacts:
            self.ros_contact_flags_topic = rospy.get_param("~ros_contact_flags_topic", "contact_flags")

        self.joint_dir = rospy.get_param("~joint_dir", [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        self.joint_offset = rospy.get_param("~joint_offset", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        self.sim_period = 1.0/self.sim_freq

    def set_start_position(self):
        self.init_ref_pos[0] = -0.0
        self.init_ref_pos[1] = -0.4
        self.init_ref_pos[2] = 0.8
        self.init_ref_pos[3] = 0.0
        self.init_ref_pos[4] = 0.4
        self.init_ref_pos[5] = -0.8
        self.init_ref_pos[6] = 0.0
        self.init_ref_pos[7] = -0.4
        self.init_ref_pos[8] = 0.8
        self.init_ref_pos[9] = -0.0
        self.init_ref_pos[10] = 0.4
        self.init_ref_pos[11] = -0.8

    def init_simulation(self):
        self.env = MorsBulletEnv(
                urdf_root=self.urdf_root,
                sim_freq=self.sim_freq,
                world=self.world_name,
                hard_reset=False,
                render=self.render,
                on_rack=self.on_rack,
                self_collision_enabled=False,
                debug_mode=True,
                floating_camera=False,
                step_enabled=False,

                accurate_motor_model_enabled=self.accurate_motor_model_enabled,
                simple_motor_model_enabled=self.simple_motor_model_enabled,
                torque_control_enabled=self.torque_control_enabled,
                motor_kp=30.0,
                motor_kd=0.1,
                motor_velocity_limit=np.inf,
                
                max_timesteps = np.inf,
                action_repeat=1,
                rew_scale = 1,
                distance_limit=float("inf"),
                observation_noise_stdev=0.0,
                normalization = False,

                camera_enabled=self.camera_enabled,
                lidar_enabled=self.lidar_enabled,
                ext_disturbance_enabled=self.external_disturbance_enabled,
                )

        self.env.set_friction(self.lateral_friction, self.spinning_friction)
        self.env.set_ext_forces_params(self.external_disturbance_value, 
                                       self.external_disturbance_duration*self.sim_freq, 
                                       self.external_disturbance_interval*self.sim_freq)
        # self.env.set_joint_max_velocity(40)

        if self.camera_enabled:
            self.env.camera.set_params(self.camera_freq, self.pixel_width, self.pixel_height, self.pointcloud_enabled)

        if self.lidar_enabled:
            self.env.lidar.set_params(render=self.lidar_render, 
                                      angle_min=self.lidar_angle_min, 
                                      angle_max=self.lidar_angle_max, 
                                      point_num=self.lidar_point_num, 
                                      range_min=self.lidar_range_min, 
                                      range_max=self.lidar_range_max)
            self.env.lidar.reset()

        
    
    def camera_loop(self):
        rate_1 = rospy.Rate(self.camera_freq)
        it = 0
        while (1):
            self.env.camera.update()

            if self.env.camera.is_camera_updated():
                self.rgb_img = self.env.camera.get_rgb_image()
                self.depth_img = self.env.camera.get_depth_image()
                self.pointcloud_msg = self.env.camera.get_pointcloud()
                self.__pub_rgb_image(self.rgb_img)
                self.__pub_depth_image(self.depth_img)
                if self.pointcloud_enabled:
                    self.__pub_pointcloud(self.pointcloud_msg)

            rate_1.sleep()

    def loop(self):
        self.sim_it += 1

        self.env.set_kpkd(self.kp[0], self.kd[0])
        # print(self.kp)
        # self.env.set_kpkd(30.0, 0.1)
        # print(self.ref_pos, self.ref_vel, self.ref_torq)
        self.env.set_motor_commands(self.ref_pos, self.ref_vel, self.ref_torq)

        observation, rew, done, _, _ = self.env.step(self.ref_data)

        self.contact_points, self.contact_flags = self.env.get_contact_flags()
        # print(self.contact_flags)
        self.leg_data["pos"], self.leg_data["vel"], self.leg_data["torq"], self.leg_data["name"] = self.env.get_motor_states()

        self.imu_data[7:] = self.env.get_base_ang_vel()
        self.body_lin_pos, self.imu_data[3:7] = self.env.get_base_position_and_orientation()
        self.body_lin_vel = np.array(self.env.get_base_lin_vel(), float)
        self.imu_data[0:3] = (self.body_lin_vel - self.body_lin_vel_prev)/self.sim_period
        self.body_lin_vel_prev = self.body_lin_vel
        # print(f"{np.sqrt(self.body_lin_vel[0]**2 + self.body_lin_vel[1]**2):.2f}")
          

        self.__pub_clocks()
        self.__pub_imu_msg(self.imu_data)
        self.__pub_joint_states(self.leg_data)
        self.__pub_robot_vel(self.body_lin_vel)

        if self.foot_contacts:
            self.__pub_foot_contacts(self.contact_flags)
        
        if self.ros_whole_body_state_enabled:
            self.robot_tf.sendTransform(self.__fill_tf_message("map", "base_link", self.body_lin_pos, self.imu_data[3:7]))
            self.__pub_odom_msg(self.body_lin_pos, self.imu_data)
            self.__pub_whole_body_state(imu_data=self.imu_data,
                                        leg_data=self.leg_data,
                                        base_pos=self.body_lin_pos,
                                        contact_points=self.contact_points)

        if self.lidar_enabled and self.env.lidar.is_lidar_updated():
            self.__pub_lidar_message(self.env.lidar.get_data())

        self.map_state_msg(observation)
        self.lc_servo_state.publish(self.lcm_servo_state_channel, self.servo_state_msg.encode())

    def __pub_robot_vel(self, vel):
        self.robot_vel_msg.x = vel[0]
        self.robot_vel_msg.y = vel[1]
        self.robot_vel_msg.z = vel[2]
        self.vel_pub.publish(self.robot_vel_msg)

    def __pub_imu_msg(self, imu_data : list):
        self.imu_msg.linear_acceleration.x = imu_data[0]
        self.imu_msg.linear_acceleration.y = imu_data[1]
        self.imu_msg.linear_acceleration.z = imu_data[2]
        self.imu_msg.angular_velocity.x = imu_data[7]
        self.imu_msg.angular_velocity.y = imu_data[8]
        self.imu_msg.angular_velocity.z = imu_data[9]
        self.imu_msg.orientation.x = imu_data[3]
        self.imu_msg.orientation.y = imu_data[4]
        self.imu_msg.orientation.z = imu_data[5]
        self.imu_msg.orientation.w = imu_data[6]
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = "base_link"
        self.imu_data_pub.publish(self.imu_msg)

    def __pub_joint_states(self, joint_states : dict):
        self.js_msg.name = []
        self.js_msg.position = []
        self.js_msg.velocity = []
        self.js_msg.effort = []
        # i = 0
        # for _ in joint_states["name"]:
        #     self.js_msg.name.append(joint_states["name"][i].decode('utf-8'))
        #     self.js_msg.position.append(joint_states["pos"][i]*self.leg_data_transform[i])
        #     self.js_msg.velocity.append(joint_states["vel"][i]*self.leg_data_transform[i])
        #     self.js_msg.effort.append(joint_states["torq"][i]*self.leg_data_transform[i])
        #     i += 1
        
        for o in self.leg_data_order:
            self.js_msg.name.append(joint_states["name"][o].decode('utf-8'))
            self.js_msg.position.append(joint_states["pos"][o]*self.leg_data_transform[o])
            self.js_msg.velocity.append(joint_states["vel"][o]*self.leg_data_transform[o])
            self.js_msg.effort.append(joint_states["torq"][o]*self.leg_data_transform[o])
            # i += 1

        self.js_msg.header.stamp = rospy.Time.now()
        self.js_msg.header.frame_id = ""
        self.js_pub.publish(self.js_msg)

    def __pub_whole_body_state(self, imu_data, leg_data, base_pos, contact_points):
        self.wbs.header.stamp = rospy.Time.now()
        self.wbs.header.frame_id = "map"
        self.wbs.time = self.wbs.header.stamp.secs
        # This represents the base state (CoM motion, angular motion and centroidal momenta)
        self.wbs.centroidal.com_position.x = base_pos[0]
        self.wbs.centroidal.com_position.y = base_pos[1]
        self.wbs.centroidal.com_position.z = base_pos[2]
        self.wbs.centroidal.base_orientation.x = imu_data[3]
        self.wbs.centroidal.base_orientation.y = imu_data[4]
        self.wbs.centroidal.base_orientation.z = imu_data[5]
        self.wbs.centroidal.base_orientation.w = imu_data[6]
        self.wbs.centroidal.base_angular_velocity.x = imu_data[7]
        self.wbs.centroidal.base_angular_velocity.y = imu_data[8]
        self.wbs.centroidal.base_angular_velocity.z = imu_data[9]
        # This represents the joint state (position, velocity, acceleration and effort)
        self.wbs.joints = []
        i = 0
        for _ in leg_data["name"]:
            js_msg = WBJointState()
            js_msg.name = leg_data["name"][i].decode('utf-8')
            js_msg.position = leg_data["pos"][i]
            js_msg.velocity = leg_data["vel"][i]
            self.wbs.joints.append(js_msg)
            i += 1
        # This represents the end-effector state (cartesian position and contact forces)
        self.wbs.contacts = []
        
        for contact_point in contact_points:
            contact_msg = WBContactState()
            contact_msg.name = "base_link"
            contact_msg.type = WBContactState.UNKNOWN
            contact_msg.pose.position.x = contact_point[5][0]
            contact_msg.pose.position.y = contact_point[5][1]
            contact_msg.pose.position.z = contact_point[5][2]
            contact_msg.wrench.force.z = contact_point[9]
            contact_msg.surface_normal.x = contact_point[7][0]
            contact_msg.surface_normal.y = contact_point[7][1]
            contact_msg.surface_normal.z = contact_point[7][2]
            contact_msg.friction_coefficient = 1.0
            self.wbs.contacts.append(contact_msg)
        self.wbc_state_pub.publish(self.wbs)

    def __fill_tf_message(self, parent_frame, child_frame, translation, rotation):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        return t
    
    def __pub_odom_msg(self, base_pos, imu_data):
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = "map"
        self.odom.child_frame_id = "base_link"
        self.odom.pose.pose.position.x = base_pos[0]
        self.odom.pose.pose.position.y = base_pos[1]
        self.odom.pose.pose.position.z = base_pos[2]
        self.odom.pose.pose.orientation.x = imu_data[3]
        self.odom.pose.pose.orientation.y = imu_data[4]
        self.odom.pose.pose.orientation.z = imu_data[5]
        self.odom.pose.pose.orientation.w = imu_data[6]

        self.odom_pub.publish(self.odom)

        t = self.__fill_tf_message(
            self.odom.header.frame_id, self.odom.child_frame_id, base_pos[0:3], imu_data[3:7])
        # because of SLAM
        self.robot_tf.sendTransform(t)
    
    def __pub_rgb_image(self, img):
        self.img_rgb_msg.header.stamp = rospy.Time().now()
        pil_img = pil.fromarray(img)
        self.img_rgb_msg.data = np.array(pil_img.convert('RGB')).tobytes()
        self.rgb_image_pub.publish(self.img_rgb_msg)
        self.rgb_info_pub.publish(self.cam_info_msg)

    def __pub_depth_image(self, img):
        self.img_depth_msg.header.stamp = rospy.Time().now()
        self.img_depth_msg.data = np.array(img).tobytes()

        self.depth_image_pub.publish(self.img_depth_msg)
        self.depth_info_pub.publish(self.cam_info_msg)

    def __pub_pointcloud(self, pointcloud):
        self.depth_points_pub.publish(pointcloud)

    def __pub_foot_contacts(self, contact_flags):
        self.contact_msg.header.stamp = rospy.Time().now()
        self.contact_msg.contacts = contact_flags
        self.contact_pub.publish(self.contact_msg)

    def __pub_lidar_message(self, hit_positions):
        self.lidar_msg.header.stamp = rospy.Time().now()
        self.lidar_msg.ranges = hit_positions
        self.lidar_pub.publish(self.lidar_msg)

    def __pub_clocks(self):
        self.clock_msg.clock = rospy.Time.from_sec(1*(rospy.get_time() - self.zero_time))
        self.clock_pub.publish(self.clock_msg)

    def get_cmd(self):
        # init LCM
        lc = lcm.LCM()
        subscription = lc.subscribe(self.lcm_servo_cmd_channel, self.cmd_handler)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass


    def cmd_handler(self, channel, data):
        msg = servo_cmd_msg.decode(data)
        # print(msg.kp)
        # print(f"Received message: {msg.position}")
        for i in range(12):
            self.ref_pos[i] = msg.position[i]
            self.ref_vel[i] = msg.velocity[i]
            self.ref_torq[i] = msg.torque[i]
            self.kp[i] = msg.kp[i]
            self.kd[i] = msg.kd[i]

    def map_state_msg(self, observation):
        for i in range(12):
            self.servo_state_msg.position[i] = self.joint_dir[i]*observation[i]-self.joint_offset[i]
            self.servo_state_msg.velocity[i] = self.joint_dir[0]*observation[i+12]
            self.servo_state_msg.torque[i] = self.joint_dir[0]*observation[i+24]

    def get_sim_period(self):
        return self.sim_period


def main():
    hw_level = Hardware_Level_Sim()
    sim_period = hw_level.get_sim_period()
    
    # it = 0
    # lst = []
    # while True:
    #     start_time = time.time()
    #     hw_level.loop()
    #     elapced_time = time.time() - start_time
    #     wait_time = sim_period - elapced_time
    #     if wait_time > 0:
    #         time.sleep(wait_time)

    #     if it < 100:
    #         it += 1
    #         lst.append(elapced_time)
    #     else:
    #         m_lst = np.mean(lst)
    #         print(f"Ref period: {sim_period:.5f}; Real execution time: {m_lst:.5f}; Freq could be: {1/m_lst:.2f}")
    #         it = 0
    #         lst = []

    rate = rospy.Rate(1/sim_period)
    while not rospy.is_shutdown():
        hw_level.loop()
        rate.sleep()


if __name__ == '__main__':
    main()
