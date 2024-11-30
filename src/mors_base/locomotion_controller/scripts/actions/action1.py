import numpy as np
from zmp_controller.TrajectoryGenerator import create_multiple_trajectory
import time
from zmp_controller.ikine import IKineQuadruped
import rospy

from datetime import datetime


class GetUp():
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
        # self.finished = False

        # Put your code here
        # self.start_time = int(time.time()*1000.0)

        self.kpkd_null()

        theta_cur = list(self.cur_joint_pos[:])
        theta_ref = theta_cur[:]

        if theta_cur[1] < 0 and theta_cur[4] > 0 and theta_cur[7] < 0 and theta_cur[10] > 0:
            print("___")
            print("> >")
            start_pos = 'm'
        elif theta_cur[1] < 0 and theta_cur[4] > 0 and theta_cur[7] > 0 and theta_cur[10] < 0:
            print("___")
            print("> <")
            start_pos = 'x'
        elif theta_cur[1] > 0 and theta_cur[4] < 0 and theta_cur[7] > 0 and theta_cur[10] < 0:
            print("___")
            print("< <")
            start_pos = 'inv_m'
        elif theta_cur[1] > 0 and theta_cur[4] < 0 and theta_cur[7] < 0 and theta_cur[10] > 0:
            print("___")
            print("< >")
            start_pos = 'o'
        else:
            start_pos = 'z'

        for i in range(12):
            self.all_theta_refs[i].append(theta_cur[i])
            self.all_kp_refs[i].append(self.ref_joint_kp[i])
            self.all_kd_refs[i].append(self.ref_joint_kd[i])

        for _ in range(int(self.freq*0.4)):
            self.kpkd_inc(0.4)
            # print(self.ref_joint_kp)
            for i in range(12):
                self.all_theta_refs[i].append(theta_cur[i])
                self.all_kp_refs[i].append(self.ref_joint_kp[i])
                self.all_kd_refs[i].append(self.ref_joint_kd[i])

        if start_pos != 'm':
            # установить ноги в позицию когда бедра наверху
            theta_cur = theta_ref[:]
            theta_ref[0] = -1.57
            theta_ref[3] = 1.57
            theta_ref[6] = 1.57
            theta_ref[9] = -1.57
            theta_ref[2] = 3.14
            theta_ref[5] = -3.14
            theta_ref[8] = 3.14
            theta_ref[11] = -3.14

            theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.5, 1/self.freq)

            for i in range(12):
                self.all_theta_refs[i] = self.all_theta_refs[i] + theta_refs[i]
                self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
                self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

            # установить ноги в позицию когда задние колени смотрят наружу
            theta_cur = theta_ref[:]
            theta_ref = [-1.57, -1.57,  3.14, 
                        1.57,  1.57, -3.14, 
                        1.57, -1.57,  3.14, 
                        -1.57,  1.57, -3.14]
            theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.5, 1/self.freq)

            for i in range(12):
                self.all_theta_refs[i] = self.all_theta_refs[i] + theta_refs[i]
                self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
                self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        theta_cur = theta_ref[:]
        if self.kinematic_scheme == 'm':
            theta_ref[2] = 3.14
            theta_ref[5] = -3.14
            theta_ref[8] = 3.14
            theta_ref[11] = -3.14
        elif self.kinematic_scheme == 'x':
            theta_ref[2] = 3.14
            theta_ref[5] = -3.14
            theta_ref[8] = -3.14
            theta_ref[11] = 3.14
        if self.kinematic_scheme == 'o':
            theta_ref[2] = -3.14
            theta_ref[5] = 3.14
            theta_ref[8] = 3.14
            theta_ref[11] = -3.14

        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.4, 1/self.freq)

        for i in range(12):
            self.all_theta_refs[i] = self.all_theta_refs[i] + theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        theta_cur = theta_ref[:]
        if self.kinematic_scheme == 'm':
            theta_ref = [0, -1.57, 3.14, 0, 1.57, -3.14, 0, -1.57, 3.14, 0, 1.57, -3.14]
        elif self.kinematic_scheme == 'x':
            theta_ref = [0, -1.57, 3.14, 0, 1.57, -3.14, 0, 1.57, -3.14, 0, -1.57, 3.14]
        elif self.kinematic_scheme == 'o':
            theta_ref = [0, 1.57, -3.14, 0, -1.57, 3.14, 0, -1.57, 3.14, 0, 1.57, -3.14]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.4, 1/self.freq)

        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        l = 0.09585
        abad_attach_y = 0.066
        angle = np.arccos((self.ef_init_y-abad_attach_y)/l)

        theta_cur = theta_ref[:]
        theta_ref = theta_cur[:]
        theta_ref[0] = angle
        theta_ref[6] = -angle
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.3, 1/self.freq)
        
        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        theta_cur = theta_ref[:]
        theta_ref = theta_cur[:]
        theta_ref[3] = -angle
        theta_ref[9] = angle
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.3, 1/self.freq)

        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        theta_cur = theta_ref[:]
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                             self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height,
                                            -self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                            -self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.6, 1/self.freq)

        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        # Finish

        # self.finished = True

    # Put your functions below

    def kpkd_inc(self, t):
        ts = self.freq*t
        kp_inc = self.max_kp[:3]/ts
        kd_inc = self.max_kd[:3]/ts

        for i in range(3):
            if self.ref_joint_kp[i] < self.max_kp[i]:
                self.ref_joint_kp[i]   += kp_inc[i]
                self.ref_joint_kp[i+3] += kp_inc[i]
                self.ref_joint_kp[i+6] += kp_inc[i]
                self.ref_joint_kp[i+9] += kp_inc[i]

            if self.ref_joint_kd[i] < self.max_kd[i]:
                self.ref_joint_kd[i]   += kd_inc[i]
                self.ref_joint_kd[i+3] += kd_inc[i]
                self.ref_joint_kd[i+6] += kd_inc[i]
                self.ref_joint_kd[i+9] += kd_inc[i]

        # time.sleep(1/self.freq)

    def kpkd_null(self):
        self.ref_joint_kp = [0.0]*12
        self.ref_joint_kd = [0.0]*12
        # time.sleep(10/self.freq)

    def take_position(self, theta_refs):
        step_period = 0.025
        for i in range(len(theta_refs[0])):
            # print(int(time.time()*1000.0 - self.start_time))
            # start_time = time.time()

            for j in range(12):
                self.ref_joint_pos[j] = theta_refs[j][i]

            # elapced_time = time.time() - start_time
            # print(f"{elapced_time:.5f}")
            # wait_time = step_period - elapced_time
            # if wait_time > 0:
            #     time.sleep(wait_time)

            # time.sleep(0.02)




    