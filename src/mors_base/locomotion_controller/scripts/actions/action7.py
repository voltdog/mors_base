import numpy as np
from zmp_controller.TrajectoryGenerator import create_multiple_trajectory
import time
from zmp_controller.ikine import IKineQuadruped


class Action7():
    def __init__(self,
                 freq=300,
                 kinematic_scheme='x',
                 ef_init_x=0.149,
                 ef_init_y=0.13,
                 cog_offset_x=0.0,
                 cog_offset_y=0.0,
                 robot_height=0.17,
                 kp=[12, 12, 12],
                 kd=[0.1, 0.1, 0.1]):
        
        self.freq = freq
        self.kinematic_scheme = kinematic_scheme
        self.ef_init_x = ef_init_x
        self.ef_init_y = ef_init_y
        self.cog_offset_x = cog_offset_x
        self.cog_offset_y = cog_offset_y
        self.robot_height = robot_height
        self.max_kp = np.array(kp[:3])
        self.max_kd = np.array(kd[:3]) + [0.4, 0.4, 0.4]

        self.finished = False
        self.cur_joint_pos = np.array([0.0]*12)
        self.ref_joint_pos = np.array([0.0]*12)
        self.ref_joint_vel = np.array([0.0]*12)
        self.ref_joint_torq = np.array([0.0]*12)
        self.ref_joint_kp = np.array([0.0]*12)
        self.ref_joint_kd = np.array([0.0]*12)

        self.ik = IKineQuadruped(theta_offset=[0, -np.pi/2, 0])

        self.start_time = int(time.time()*1000.0)

        self.all_theta_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_vel_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_torq_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_kp_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_kd_refs = [[], [], [], [], [], [], [], [], [], [], [], []]

        self.hand = False

        # print(self.max_kp)

    def is_finished(self):
        return self.finished

    def set_cur_joint_pos(self, cur_pos):
        self.cur_joint_pos = cur_pos[:]

    def get_ref_joint_pos(self):
        return self.ref_joint_pos, self.ref_joint_vel, self.ref_joint_torq, self.ref_joint_kp, self.ref_joint_kd
    
    def reset(self):
        self.all_theta_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_vel_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_torq_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_kp_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_kd_refs = [[], [], [], [], [], [], [], [], [], [], [], []]

    def step(self, it):
        self.ref_joint_pos = []
        self.ref_joint_vel = [0]*12
        self.ref_joint_torq = [0]*12
        self.ref_joint_kp = []
        self.ref_joint_kd = []

        if it >= len(self.all_theta_refs[0]):
            finished = True
            for i in range(12):
                self.ref_joint_pos.append(self.all_theta_refs[i][-1])
                self.ref_joint_kp.append(self.all_kp_refs[i][-1])
                self.ref_joint_kd.append(self.all_kd_refs[i][-1])
        else:
            finished = False
            for i in range(12):
                self.ref_joint_pos.append(self.all_theta_refs[i][it])
                self.ref_joint_kp.append(self.all_kp_refs[i][it])
                self.ref_joint_kd.append(self.all_kd_refs[i][it])

        # print(self.ref_joint_kp[0])

        return finished, self.ref_joint_pos, self.ref_joint_vel, self.ref_joint_torq, self.ref_joint_kp, self.ref_joint_kd

    def execute(self):
        self.finished = False

        # Put your code here
        # self.hand = not self.hand
        # установим kp и kd
        self.ref_joint_kp = [self.max_kp[0], self.max_kp[1], self.max_kp[2]]*4
        self.ref_joint_kd = [self.max_kd[0], self.max_kd[1], self.max_kd[2]]*4

        # установим текущие углы в качестве желаемых
        theta_cur = list(self.cur_joint_pos[:])
        for i in range(12):
            self.all_theta_refs[i].append(theta_cur[i])
            self.all_kp_refs[i].append(self.ref_joint_kp[i])
            self.all_kd_refs[i].append(self.ref_joint_kd[i])

        # сместим корпус
        if self.hand == True:
            cog_point_x = 0.04
            cog_point_y = 0.04
            r1_point_z = 0.08
            l1_point = 0.0
            sit_point = 0.02
        else:
            cog_point_x = 0.04
            cog_point_y = -0.04
            r1_point_z = 0.0
            l1_point = 0.08
            sit_point = 0.02
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height-sit_point+r1_point_z,
                                    self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height-sit_point+l1_point,
                                    -self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height+sit_point,
                                    -self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height+sit_point], config=self.kinematic_scheme)

        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.25, 1/self.freq)

        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        # вытянем лапу
        theta_cur = theta_ref[:]
        if self.hand == True:
            r1_point_x = 0.19
            l1_point_x = 0.0
            r1_point_z = 0.1
            l1_point_z = 0.0
        else:
            r1_point_x = 0.0
            l1_point_x = 0.19
            r1_point_z = 0.0
            l1_point_z = 0.1

        theta_ref = self.ik.calculate([self.ef_init_x+self.cog_offset_x+cog_point_x+r1_point_x, -self.ef_init_y-cog_point_y, -self.robot_height-sit_point+r1_point_z,
                                       self.ef_init_x+self.cog_offset_x+cog_point_x+l1_point_x,  self.ef_init_y-cog_point_y, -self.robot_height-sit_point+l1_point_z,
                                      -self.ef_init_x+self.cog_offset_x+cog_point_x,            -self.ef_init_y-cog_point_y, -self.robot_height+sit_point,
                                      -self.ef_init_x+self.cog_offset_x+cog_point_x,             self.ef_init_y-cog_point_y, -self.robot_height+sit_point], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.3, 1/self.freq)

        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        # пожмем лапу
        if self.hand == True:
            r1_shake = 0.03
            l1_shake = 0.0
        else:
            r1_shake = 0.0
            l1_shake = 0.03
        for _ in range(4):
            theta_cur = theta_ref[:]
            theta_ref = self.ik.calculate([self.ef_init_x+self.cog_offset_x+cog_point_x+r1_point_x, -self.ef_init_y-cog_point_y, -self.robot_height-sit_point+r1_point_z+r1_shake,
                                           self.ef_init_x+self.cog_offset_x+cog_point_x+l1_point_x,  self.ef_init_y-cog_point_y, -self.robot_height-sit_point+l1_point_z+l1_shake,
                                          -self.ef_init_x+self.cog_offset_x+cog_point_x,            -self.ef_init_y-cog_point_y, -self.robot_height+sit_point,
                                          -self.ef_init_x+self.cog_offset_x+cog_point_x,             self.ef_init_y-cog_point_y, -self.robot_height+sit_point], config=self.kinematic_scheme)
            theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.1, 1/self.freq)

            for i in range(12):
                self.all_theta_refs[i] += theta_refs[i]
                self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
                self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

            theta_cur = theta_ref[:]
            theta_ref = self.ik.calculate([self.ef_init_x+self.cog_offset_x+cog_point_x+r1_point_x, -self.ef_init_y-cog_point_y, -self.robot_height-sit_point+r1_point_z+0.0,
                                           self.ef_init_x+self.cog_offset_x+cog_point_x+l1_point_x,  self.ef_init_y-cog_point_y, -self.robot_height-sit_point+l1_point_z+0.0,
                                          -self.ef_init_x+self.cog_offset_x+cog_point_x,            -self.ef_init_y-cog_point_y, -self.robot_height+sit_point,
                                          -self.ef_init_x+self.cog_offset_x+cog_point_x,             self.ef_init_y-cog_point_y, -self.robot_height+sit_point], config=self.kinematic_scheme)
            theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.1, 1/self.freq)

            for i in range(12):
                self.all_theta_refs[i] += theta_refs[i]
                self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
                self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        # вернем лапу обратно
        theta_cur = theta_ref[:]
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height-sit_point+r1_point_z,
                                       self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height-sit_point+l1_point_z,
                                      -self.ef_init_x+self.cog_offset_x+cog_point_x, -self.ef_init_y-cog_point_y, -self.robot_height+sit_point,
                                      -self.ef_init_x+self.cog_offset_x+cog_point_x,  self.ef_init_y-cog_point_y, -self.robot_height+sit_point], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.25, 1/self.freq)
        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        # вернем корпус обратно
        theta_cur = theta_ref[:]
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                       self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height,
                                      -self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                      -self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.35, 1/self.freq)
        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])
        
        # Finish


    # Put your functions below
    