import numpy as np
import time
from threading import Thread

from zmp_controller.ikine import IKineQuadruped
from zmp_controller.fkine import FKineQuadruped
from zmp_controller.BodyMovingControl import BodyMovingControl
from zmp_controller.ZMPLocomotion import ZMPLocomotion
from actions.action1 import GetUp
from actions.action2 import LayDown
from actions.action3 import Action3
from actions.action4 import Action4
from actions.action5 import Action5
from actions.action6 import Action6
from actions.action7 import Action7
from actions.action8 import Action8

X = 0
Y = 1
Z = 2

class RobotController():
    def __init__(self, 
                 freq=240,
                 locomotion_kp=[12.0, 12.0, 12.0], 
                 locomotion_kd=[0.3, 0.3, 0.3],
                 yaw_kp=0.4,
                 kinematic_scheme='x',
                 ef_init_x=0.166,
                 ef_init_y=0.12,
                 robot_height=0.17,
                 cog_offset_x=0.0,
                 cog_offset_y=0.0,
                 cog_offset_z=0.0,
                 stride_frequency=2.5,
                 preview_horizon=1.6) -> None:
        
        self.freq = freq
        self.locomotion_kp = [locomotion_kp[0], locomotion_kp[1], locomotion_kp[2]]*4
        self.locomotion_kd = [locomotion_kd[0], locomotion_kd[1], locomotion_kd[2]]*4
        # self.locomotion_kp[7] = 40
        # self.locomotion_kp[8] = 10
        # self.locomotion_kp[10] = 40
        # self.locomotion_kp[11] = 10
        self.yaw_kp = yaw_kp
        self.kinematic_scheme = kinematic_scheme
        self.ef_init_x = ef_init_x
        self.ef_init_y = ef_init_y
        self.cog_offset_x = cog_offset_x
        self.cog_offset_y = cog_offset_y
        self.cog_offset_z = cog_offset_z
        self.robot_height = robot_height
        self.stride_frequency = stride_frequency
        self.preview_horizon = preview_horizon
        
        self.it = 0
        self.inc = 1/self.freq

        self.ref_servo_pos = [0]*12
        self.ref_servo_vel = [0]*12
        self.ref_servo_torq = [0]*12
        self.ref_servo_kp = [0]*12
        self.ref_servo_kd = [0]*12

        self.mode_num = -1
        self.action_num = 0
        self.pre_action_num = 0

        self.stride_height = 0.0
        self.max_stride_height = 0.0
        self.stride_height_set = False

        self.cmd_joint_pos = [0]*12
        self.cmd_joint_vel = [0]*12
        self.cmd_joint_torq = [0]*12
        self.cmd_ef_pos = [0]*12
        self.cmd_ef_body_pos = [ self.ef_init_x, -self.ef_init_y, -self.robot_height,
                                 self.ef_init_x,  self.ef_init_y, -self.robot_height,
                                -self.ef_init_x, -self.ef_init_y, -self.robot_height,
                                -self.ef_init_x,  self.ef_init_y, -self.robot_height ]
        self.cmd_pose = [0]*6
        self.cmd_vel = [0]*3

        self.kp = [0]*12
        self.kd = [0]*12

        self.cur_active_legs = [0]*4

        self.euler = [0]*3

        # init control algorithms
        self.ik = IKineQuadruped(theta_offset=[0, -np.pi/2, 0])
        self.fk = FKineQuadruped(theta_offset=[0, -np.pi/2, 0])
        self.zmp_loc = ZMPLocomotion(FPS=self.freq, 
                                     t_preview=self.preview_horizon, 
                                     stride_frequency=self.stride_frequency,
                                     cog_offset_x=cog_offset_x,
                                     cog_offset_y=cog_offset_y,
                                     cog_offset_z=cog_offset_z,
                                     ef_init_x=ef_init_x,
                                     ef_init_y=ef_init_y,
                                     robot_height=robot_height,
                                     kinematic_scheme=kinematic_scheme,
                                     )
        for i in range(50):
            self.zmp_loc.set_feedback([0]*3, [0]*3)
            self.zmp_loc.set_walking_params(v_x=0.0, 
                                        v_y=0.0, 
                                        stride_frequency=self.stride_frequency, 
                                        stride_height=self.stride_height,
                                        rot_dir=0, 
                                        rot_r=0)
            self.zmp_loc.step(self.it)
            self.it += self.inc

        self.body_moving = BodyMovingControl()

        self.pre_lpf_cmd_vel = np.array([0]*3)
        self.pre_lpf_ref_pos = np.array([0]*12)
        self.pre_lpf_cmd_pose = np.array([0]*6)

        self.action_finished = False
        self.action_it = 0

        self.standing = False
        self.robot_disable = False

        # init actions
        self.get_up = GetUp(freq=self.freq,
                            kinematic_scheme=self.kinematic_scheme,
                            ef_init_x=self.ef_init_x,
                            ef_init_y=self.ef_init_y,
                            cog_offset_x=self.cog_offset_x,
                            cog_offset_y=self.cog_offset_y,
                            robot_height=self.robot_height,
                            kp=self.locomotion_kp,
                            kd=self.locomotion_kd)
        # self.getup_th = Thread(target=self.get_up.execute, args=())
        # self.getup_th.daemon = True
        self.get_up_started = False

        self.lay_down = LayDown(freq=self.freq,
                            kinematic_scheme=self.kinematic_scheme,
                            ef_init_x=self.ef_init_x,
                            ef_init_y=self.ef_init_y,
                            robot_height=self.robot_height,
                            kp=self.locomotion_kp,
                            kd=self.locomotion_kd)
        # self.laydown_th = Thread(target=self.lay_down.execute, args=())
        # self.laydown_th.daemon = True
        self.lay_down_started = False

        self.action3 = Action3(freq=self.freq,
                            kinematic_scheme=self.kinematic_scheme,
                            ef_init_x=self.ef_init_x,
                            ef_init_y=self.ef_init_y,
                            cog_offset_x=self.cog_offset_x,
                            cog_offset_y=self.cog_offset_y,
                            robot_height=self.robot_height,
                            kp=self.locomotion_kp,
                            kd=self.locomotion_kd)
        # self.action3_th = Thread(target=self.action3.execute, args=())
        # self.action3_th.daemon = True
        self.action3_started = False

        self.action4 = Action4(freq=self.freq,
                            kinematic_scheme=self.kinematic_scheme,
                            ef_init_x=self.ef_init_x,
                            ef_init_y=self.ef_init_y,
                            cog_offset_x=self.cog_offset_x,
                            cog_offset_y=self.cog_offset_y,
                            robot_height=self.robot_height,
                            kp=self.locomotion_kp,
                            kd=self.locomotion_kd)
        # self.action4_th = Thread(target=self.action4.execute, args=())
        # self.action4_th.daemon = True
        self.action4_started = False

        self.action5 = Action5(freq=self.freq,
                            kinematic_scheme=self.kinematic_scheme,
                            ef_init_x=self.ef_init_x,
                            ef_init_y=self.ef_init_y,
                            cog_offset_x=self.cog_offset_x,
                            cog_offset_y=self.cog_offset_y,
                            robot_height=self.robot_height,
                            kp=self.locomotion_kp,
                            kd=self.locomotion_kd)
        # self.action5_th = Thread(target=self.action5.execute, args=())
        # self.action5_th.daemon = True
        self.action5_started = False

        self.action6 = Action6(freq=self.freq,
                            kinematic_scheme=self.kinematic_scheme,
                            ef_init_x=self.ef_init_x,
                            ef_init_y=self.ef_init_y,
                            cog_offset_x=self.cog_offset_x,
                            cog_offset_y=self.cog_offset_y,
                            robot_height=self.robot_height,
                            kp=self.locomotion_kp,
                            kd=self.locomotion_kd)
        # self.action6_th = Thread(target=self.action6.execute, args=())
        # self.action6_th.daemon = True
        self.action6_started = False

        self.action7 = Action7(freq=self.freq,
                            kinematic_scheme=self.kinematic_scheme,
                            ef_init_x=self.ef_init_x,
                            ef_init_y=self.ef_init_y,
                            cog_offset_x=self.cog_offset_x,
                            cog_offset_y=self.cog_offset_y,
                            robot_height=self.robot_height,
                            kp=self.locomotion_kp,
                            kd=self.locomotion_kd)
        # self.action7_th = Thread(target=self.action7.execute, args=())
        # self.action7_th.daemon = True
        self.action7_started = False

        self.action8 = Action8(freq=self.freq,
                            kinematic_scheme=self.kinematic_scheme,
                            ef_init_x=self.ef_init_x,
                            ef_init_y=self.ef_init_y,
                            cog_offset_x=self.cog_offset_x,
                            cog_offset_y=self.cog_offset_y,
                            robot_height=self.robot_height,
                            kp=self.locomotion_kp,
                            kd=self.locomotion_kd)
        # self.action8_th = Thread(target=self.action8.execute, args=())
        # self.action8_th.daemon = True
        self.action8_started = False

        # yaw direction
        self.yaw_des = 0
        self.yaw_cur = 0
        self.yaw_e = 0

        self.K_body = [0.82, 0.82, 0.82, 0.82, 0.82, 0.88] # x y z roll pitch yaw
        self.K_robot = [0.92, 0.92, 0.72] # x y z

    def step(self):
        # low pass filter for cmd vel
        param = np.array([self.cmd_vel[X], self.cmd_vel[Y], self.cmd_vel[Z]])
        d_param = param - self.pre_lpf_cmd_vel
        out_param = param - np.multiply(self.K_robot, d_param)
        self.pre_lpf_cmd_vel = out_param[:]
        self.cmd_vel[X] = out_param[0]
        self.cmd_vel[Y] = out_param[1]
        self.cmd_vel[Z] = out_param[2]

        param = np.array(self.cmd_pose)
        d_param = param - self.pre_lpf_cmd_pose
        
        out_param = param - np.multiply(self.K_body, d_param)
        self.pre_lpf_cmd_pose = out_param[:]
        self.cmd_pose = out_param[:]

        # check if node is disabled
        if self.robot_disable == True:
            self.standing = False
            # self.mode_num = 4
            self.action_num = 0

        # print(self.robot_disable, self.standing)

        # choose working mode
        if self.action_num != 0:
            ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.action()
        else:
            self.action_finished = True
            if self.standing:
                if self.mode_num == 0:
                    ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.locomotion_control()
                elif self.mode_num == 1:
                    ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.ef_control()
                elif self.mode_num == 2:
                    ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.body_control()
                elif self.mode_num == 3:
                    ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.joint_control()
                else:
                    ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.do_nothing()
            else:
                if self.mode_num == 3:
                    ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.joint_control()
                else:
                    ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.do_nothing()

        self.pre_action_num = self.action_num

        # low pass filter for ref angles
        angles = np.array(ref_servo_pos)
        d_angles = angles - self.pre_lpf_ref_pos
        out_angles = angles - 0.7*d_angles
        self.pre_lpf_ref_pos = out_angles[:]
        ref_servo_pos = out_angles[:]

        return ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd
    
    def do_nothing(self):
        ref_servo_pos = [0]*12
        ref_servo_vel = [0]*12
        ref_servo_torq = [0]*12
        ref_servo_kp = [0]*12
        ref_servo_kd = self.kd[:]#[0.0]*12

        return ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd

    def action(self):
        
        if self.action_num == 1:
            if self.get_up_started == False:
                self.get_up.reset()
                self.get_up.set_cur_joint_pos(self.cur_joint_pos)
                self.get_up.execute()
                self.get_up_started = True
            self.action_finished, ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.get_up.step(self.action_it)
            if self.action_finished == False:
                self.action_it += 1   
            else:
                self.action_num = 0
                self.action_it = 0
                self.get_up_started = False
                self.yaw_des = self.euler[2]
                self.standing = True

        elif self.action_num == 2:
            if self.lay_down_started == False:
                self.lay_down.reset()
                self.lay_down.set_cur_joint_pos(self.cur_joint_pos)
                self.lay_down.execute()
                self.lay_down_started = True
            self.action_finished, ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.lay_down.step(self.action_it)
            if self.action_finished == False:
                self.action_it += 1   
            else:
                self.action_num = 0
                self.action_it = 0
                self.lay_down_started = False
                self.standing = False
        
        elif self.action_num == 3:
            if self.action3_started == False:
                self.action3.reset()
                self.action3.set_cur_joint_pos(self.cur_joint_pos)
                self.action3.execute()
                self.action3_started = True
            self.action_finished, ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.action3.step(self.action_it)
            if self.action_finished == False:
                self.action_it += 1   
            else:
                self.action_num = 0
                self.action_it = 0
                self.action3_started = False

        elif self.action_num == 4:
            if self.action4_started == False:
                self.action4.reset()
                self.action4.set_cur_joint_pos(self.cur_joint_pos)
                self.action4.execute()
                self.action4_started = True
            self.action_finished, ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.action4.step(self.action_it)
            if self.action_finished == False:
                self.action_it += 1   
            else:
                self.action_num = 0
                self.action_it = 0
                self.action4_started = False

        elif self.action_num == 5:
            if self.action5_started == False:
                self.action5.reset()
                self.action5.set_cur_joint_pos(self.cur_joint_pos)
                self.action5.execute()
                self.action5_started = True
            self.action_finished, ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.action5.step(self.action_it)
            if self.action_finished == False:
                self.action_it += 1   
            else:
                self.action_num = 0
                self.action_it = 0
                self.action5_started = False
        
        elif self.action_num == 6:
            if self.action6_started == False:
                self.action6.reset()
                self.action6.set_cur_joint_pos(self.cur_joint_pos)
                self.action6.execute()
                self.action6_started = True
            self.action_finished, ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.action6.step(self.action_it)
            if self.action_finished == False:
                self.action_it += 1   
            else:
                self.action_num = 0
                self.action_it = 0
                self.action6_started = False

        elif self.action_num == 7:
            if self.action7_started == False:
                self.action7.reset()
                self.action7.set_cur_joint_pos(self.cur_joint_pos)
                self.action7.execute()
                self.action7_started = True
            self.action_finished, ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.action7.step(self.action_it)
            if self.action_finished == False:
                self.action_it += 1   
            else:
                self.action_num = 0
                self.action_it = 0
                self.action7_started = False

        elif self.action_num == 8:
            if self.action8_started == False:
                self.action8.reset()
                self.action8.set_cur_joint_pos(self.cur_joint_pos)
                self.action8.execute()
                self.action8_started = True
            self.action_finished, ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.action8.step(self.action_it)
            if self.action_finished == False:
                self.action_it += 1   
            else:
                self.action_num = 0
                self.action_it = 0
                self.action8_started = False

        else:
            self.action_num = 0
            ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.do_nothing()

        return ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd

    def locomotion_control(self):
        ref_servo_kp = self.locomotion_kp
        ref_servo_kd = self.locomotion_kd
        # ref_servo_kp[2] = 10
        # ref_servo_kp[5] = 10
        # ref_servo_kp[8] = 10
        # ref_servo_kp[11] = 10
        ref_servo_vel = [0]*12
        ref_servo_torq = [0]*12
        
        self.yaw_cur = self.euler[Z]
        if np.abs(self.cmd_vel[Z]) > 0.01:
            yaw_u = self.cmd_vel[Z]
            self.yaw_des = self.euler[Z]
        else:
            
            self.yaw_e = self.yaw_des - self.yaw_cur
            if self.cmd_vel[X] > 0:
                yaw_u = self.yaw_kp * self.yaw_e
            else:
                yaw_u = -self.yaw_kp * self.yaw_e

        if np.abs(self.cmd_vel[X]) < 0.01:
            yaw_u = 0.0

        # print(self.yaw_kp)
        # print(f"{self.cmd_vel[Z]}, {self.yaw_des}, {self.yaw_cur}")
        
        if yaw_u > 0:
            rot_dir = -1
        elif yaw_u < 0:
            rot_dir = 1
        else:
            rot_dir = 0
        
        if yaw_u == 0:
            rot_r = 5000000
        else:
            rot_r = 1 - np.abs(np.clip(yaw_u, -0.9, 0.9))

        if np.abs(self.cmd_vel[X]) > 0.05 or np.abs(self.cmd_vel[Y]) > 0.05 or np.abs(self.cmd_vel[Z]) > 0.05:
            if self.stride_height_set == False:
                self.stride_height = self.max_stride_height
                self.stride_height_set = True
        else:
            if self.stride_height_set == True:
                self.stride_height = 0.0
                self.stride_height_set = False
                self.cmd_vel[X] = 0.0
                self.cmd_vel[Y] = 0.0
                self.cmd_vel[Z] = 0.0

        self.zmp_loc.set_feedback(self.euler, [0]*3)
        self.zmp_loc.set_walking_params(v_x=self.cmd_vel[X]*2, 
                                    v_y=self.cmd_vel[Y]*2, 
                                    stride_frequency=self.stride_frequency, 
                                    stride_height=self.stride_height,
                                    rot_dir=rot_dir, 
                                    rot_r=rot_r)
        ref_servo_pos = self.zmp_loc.step(self.it)
        self.cur_active_legs = self.zmp_loc.get_state()
        self.it += self.inc

        return ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd

    def body_control(self):
        ref_servo_kp = self.locomotion_kp
        ref_servo_kd = self.locomotion_kd
        ref_servo_vel = [0]*12
        ref_servo_torq = [0]*12

        self.body_moving.set_body_lin_pos(self.cmd_pose[0:3])
        self.body_moving.set_body_ang_pos(self.cmd_pose[3:6])

        ef_pos = self.body_moving.step(self.cmd_ef_body_pos)
        # print(f"R1: {ef_pos[:3]}")
        # print(f"L1: {ef_pos[3:6]}")
        # print(f"R2: {ef_pos[6:9]}")
        # print(f"L2: {ef_pos[9:12]}")
        # print("===============")

        ref_servo_pos = self.ik.calculate(p_ref=ef_pos, config=self.kinematic_scheme)

        return ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd


    def ef_control(self):
        ef_pos = [0]*12

        ef_pos[0] = self.cmd_ef_pos[0] + self.ef_init_x + self.cog_offset_x
        ef_pos[1] = self.cmd_ef_pos[1] - self.ef_init_y + self.cog_offset_y
        ef_pos[2] = self.cmd_ef_pos[2] - self.robot_height
        
        ef_pos[3] = self.cmd_ef_pos[3] + self.ef_init_x + self.cog_offset_x
        ef_pos[4] = self.cmd_ef_pos[4] + self.ef_init_y + self.cog_offset_y
        ef_pos[5] = self.cmd_ef_pos[5] - self.robot_height
        
        ef_pos[6] = self.cmd_ef_pos[6] - self.ef_init_x + self.cog_offset_x
        ef_pos[7] = self.cmd_ef_pos[7] - self.ef_init_y + self.cog_offset_y
        ef_pos[8] = self.cmd_ef_pos[8] - self.robot_height
        
        ef_pos[9] = self.cmd_ef_pos[9] - self.ef_init_x + self.cog_offset_x
        ef_pos[10] = self.cmd_ef_pos[10] + self.ef_init_y + self.cog_offset_y
        ef_pos[11] = self.cmd_ef_pos[11] - self.robot_height

        ref_servo_kp = self.locomotion_kp
        ref_servo_kd = self.locomotion_kd
        ref_servo_vel = [0]*12
        ref_servo_torq = [0]*12
        ref_servo_pos = self.ik.calculate(p_ref=ef_pos, config=self.kinematic_scheme)

        return ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd

    def joint_control(self):
        
        ref_servo_kp = self.kp[:]
        ref_servo_kd = self.kd[:]
        ref_servo_pos = self.cmd_joint_pos
        ref_servo_vel = self.cmd_joint_vel
        ref_servo_torq = self.cmd_joint_torq

        return ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd

    def set_euler(self, euler):
        self.euler = euler[:]
    
    def set_mode_num(self, mode : int):
        self.mode_num = mode

    def set_action_num(self, action : int):
        self.action_num = action

    def set_stride_height(self, height):
        self.max_stride_height = height

    def set_kp(self, kp):
        self.kp = kp[:]

    def set_kd(self, kd):
        self.kd = kd[:]

    def set_cmd_vel(self, vel):
        self.cmd_vel = vel[:]

        if self.cmd_vel[X] > 0.8:
            self.cmd_vel[X] = 0.8
        if self.cmd_vel[X] < -0.4:
            self.cmd_vel[X] = -0.4

        if self.cmd_vel[Y] > 0.2:
            self.cmd_vel[Y] = 0.2
        if self.cmd_vel[Y] < -0.2:
            self.cmd_vel[Y] = -0.2
            
        if self.cmd_vel[Z] > 0.91:
            self.cmd_vel[Z] = 0.91
        if self.cmd_vel[Z] < -0.91:
            self.cmd_vel[Z] = -0.91

    def set_cmd_pose(self, pose):
        self.cmd_pose = pose[:]

    def set_cmd_ef_pos(self, ef_pos):
        self.cmd_ef_pos = ef_pos[:]

    def set_cmd_joint_pos(self, pos, vel, torq):
        self.cmd_joint_pos = pos[:]
        self.cmd_joint_vel = vel[:]
        self.cmd_joint_torq = torq[:]

    def set_cur_joint_pos(self, joint_pos):
        self.cur_joint_pos = joint_pos

    def set_dis_status(self, disable):
        self.robot_disable = disable

    def get_cur_ef(self, joint_pos):
        return self.fk.calculate(joint_pos)
    
    def get_foot_contacts(self):
        if self.stride_height == 0 or self.mode_num != 0:
            foot_contacts = [False, False, False, False]
        else:
            foot_contacts = self.cur_active_legs

        return foot_contacts
    
    def is_action_finished(self):
        return self.action_finished
