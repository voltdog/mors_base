import numpy as np
from zmp_controller.TrajectoryGenerator import create_multiple_trajectory
import time
from zmp_controller.ikine import IKineQuadruped


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
        self.max_kd = np.array(kd[:3])

        self.finished = False
        self.cur_joint_pos = np.array([0.0]*12)
        self.ref_joint_pos = np.array([0.0]*12)
        self.ref_joint_vel = np.array([0.0]*12)
        self.ref_joint_torq = np.array([0.0]*12)
        self.ref_joint_kp = np.array([0.0]*12)
        self.ref_joint_kd = np.array([0.0]*12)

        self.ik = IKineQuadruped(theta_offset=[0, -np.pi/2, 0])

        # print(self.max_kp)

    def is_finished(self):
        return self.finished

    def set_cur_joint_pos(self, cur_pos):
        self.cur_joint_pos = cur_pos[:]

    def get_ref_joint_pos(self):
        return self.ref_joint_pos, self.ref_joint_vel, self.ref_joint_torq, self.ref_joint_kp, self.ref_joint_kd

    def execute(self):
        self.finished = False

        # Put your code here
        self.kpkd_null()
        
        self.ref_joint_pos = np.array(self.cur_joint_pos[:])
        time.sleep(0.1)
        # print(self.ref_joint_kp)
        for _ in range(int(self.freq*0.2)):
            self.kpkd_inc(0.2)
            # print(self.ref_joint_kp)

        theta_refs = np.array(self.cur_joint_pos[:])
        if self.kinematic_scheme == 'm':
            theta_refs[2] = 3.14
            theta_refs[5] = -3.14
            theta_refs[8] = 3.14
            theta_refs[11] = -3.14
        elif self.kinematic_scheme == 'x':
            theta_refs[2] = 3.14
            theta_refs[5] = -3.14
            theta_refs[8] = -3.14
            theta_refs[11] = 3.14
        if self.kinematic_scheme == 'o':
            theta_refs[2] = -3.14
            theta_refs[5] = 3.14
            theta_refs[8] = 3.14
            theta_refs[11] = -3.14

        theta_refs = create_multiple_trajectory(self.cur_joint_pos, theta_refs, 0.2, 1/self.freq)
        self.take_position(theta_refs)

        # time.sleep(2)

        theta_cur = np.array(self.cur_joint_pos[:])
        if self.kinematic_scheme == 'm':
            theta_refs = [0, -1.57, 3.14, 0, 1.57, -3.14, 0, -1.57, 3.14, 0, 1.57, -3.14]
        elif self.kinematic_scheme == 'x':
            theta_refs = [0, -1.57, 3.14, 0, 1.57, -3.14, 0, 1.57, -3.14, 0, -1.57, 3.14]
        elif self.kinematic_scheme == 'o':
            theta_refs = [0, 1.57, -3.14, 0, -1.57, 3.14, 0, -1.57, 3.14, 0, 1.57, -3.14]
        theta_refs = create_multiple_trajectory(theta_cur, theta_refs, 0.4, 1/self.freq)
        self.take_position(theta_refs)
        
        # time.sleep(2)
        l = 0.09585
        abad_attach_y = 0.066
        angle = np.arccos((self.ef_init_y-abad_attach_y)/l)

        theta_cur = np.array(self.cur_joint_pos[:])
        theta_ref = np.array(self.cur_joint_pos[:])
        theta_ref[0] = angle
        theta_ref[6] = -angle
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.2, 1/self.freq)
        self.take_position(theta_refs)
        
        # time.sleep(1)
 
        theta_cur = np.array(self.cur_joint_pos[:])
        theta_ref = np.array(self.cur_joint_pos[:])
        theta_ref[3] = -angle
        theta_ref[9] = angle
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.2, 1/self.freq)
        self.take_position(theta_refs)
        
        # time.sleep(1)

        # theta_cur = np.array(self.cur_joint_pos[:])
        # theta_ref = self.ik.calculate([  self.ef_init_x, -self.ef_init_y, -0.08,
        #                                  self.ef_init_x,  self.ef_init_y, -0.08,
        #                                 -self.ef_init_x, -self.ef_init_y, -0.08,
        #                                 -self.ef_init_x,  self.ef_init_y, -0.08], config=self.kinematic_scheme)
        # theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.5, 1/self.freq)
        # self.take_position(theta_refs)

        
        theta_cur = np.array(self.cur_joint_pos[:])
        theta_ref = self.ik.calculate([ self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                             self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height,
                                            -self.ef_init_x+self.cog_offset_x, -self.ef_init_y, -self.robot_height,
                                            -self.ef_init_x+self.cog_offset_x,  self.ef_init_y, -self.robot_height], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.6, 1/self.freq)
        self.take_position(theta_refs)

        # time.sleep(1)

        # Finish

        self.finished = True

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

        time.sleep(1/self.freq)

    def kpkd_null(self):
        self.ref_joint_kp = np.array([0.0]*12)
        self.ref_joint_kd = np.array([0.0]*12)
        time.sleep(10/self.freq)

    def take_position(self, theta_refs):
        for i in range(len(theta_refs[0])):
            for j in range(12):
                self.ref_joint_pos[j] = theta_refs[j][i]

            time.sleep(1/self.freq)




    