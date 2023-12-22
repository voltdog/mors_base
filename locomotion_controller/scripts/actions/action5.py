import numpy as np
from zmp_controller.TrajectoryGenerator import create_multiple_trajectory
import time
from zmp_controller.ikine import IKineQuadruped
from zmp_controller.fkine import FKineQuadruped


class Action5():
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
        self.fk = FKineQuadruped(theta_offset=[0, -np.pi/2, 0])

        self.start_time = int(time.time()*1000.0)

        self.all_theta_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_vel_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_torq_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_kp_refs = [[], [], [], [], [], [], [], [], [], [], [], []]
        self.all_kd_refs = [[], [], [], [], [], [], [], [], [], [], [], []]

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
        
        self.ref_joint_kp = [self.max_kp[0], self.max_kp[1], self.max_kp[2]]*4
        self.ref_joint_kd = [self.max_kd[0], self.max_kd[1], self.max_kd[2]]*4

        theta_cur = list(self.cur_joint_pos[:])
        for i in range(12):
            self.all_theta_refs[i].append(theta_cur[i])
            self.all_kp_refs[i].append(self.ref_joint_kp[i])
            self.all_kd_refs[i].append(self.ref_joint_kd[i])

        # sit down
        theta_ref = [0.3, -0.57, 0.31, 
                      -0.3, 0.57, -0.31, 
                      -0.34, -1.29, 2.28, 
                      0.34, 1.29, -2.28]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.7, 1/self.freq)
        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        # time.sleep(2)

        # raise hand
        theta_cur = theta_ref[:]

        l2_cur_pos = self.fk.calculate(self.cur_joint_pos)[3:6]
        theta_ref[3:6] = self.ik.ikine_L1([l2_cur_pos[0], l2_cur_pos[1]+0.02, l2_cur_pos[2]+0.04], config=self.kinematic_scheme)
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.3, 1/self.freq)
        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        # time.sleep(2)

        theta_cur = theta_ref[:]
        theta_ref = [0.3, -0.57, 0.31, 
                      -0.1, -2.89, 0.52, 
                      -0.34, -1.29, 2.28, 
                      0.34, 1.29, -2.28]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.5, 1/self.freq)
        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        # time.sleep(2)

        # wave left hand
        for i in range(3):
            # move L2 left
            theta_cur = theta_ref[:]
            theta_ref = [0.3, -0.57, 0.31, 
                          0.3, -2.89, 0.52, 
                          -0.34, -1.29, 2.28, 
                          0.34, 1.29, -2.28]
            theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.3, 1/self.freq)
            for i in range(12):
                self.all_theta_refs[i] += theta_refs[i]
                self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
                self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

            # move L2 right
            theta_cur = theta_ref[:]
            theta_ref = [0.3, -0.57, 0.31, 
                          -0.1, -2.89, 0.52, 
                          -0.34, -1.29, 2.28, 
                          0.34, 1.29, -2.28]
            theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.3, 1/self.freq)
            for i in range(12):
                self.all_theta_refs[i] += theta_refs[i]
                self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
                self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])

        # put the hand down
        theta_cur = theta_ref[:]
        theta_ref = [0.3, -0.57, 0.31, 
                      -0.3, 0.57, -0.31, 
                      -0.34, -1.29, 2.28, 
                      0.34, 1.29, -2.28]
        theta_refs = create_multiple_trajectory(theta_cur, theta_ref, 0.4, 1/self.freq)
        for i in range(12):
            self.all_theta_refs[i] += theta_refs[i]
            self.all_kp_refs[i] += [self.ref_joint_kp[i]]*len(theta_refs[i])
            self.all_kd_refs[i] += [self.ref_joint_kd[i]]*len(theta_refs[i])


        # stand up
        theta_cur = theta_ref
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

        self.finished = True

    # Put your functions below