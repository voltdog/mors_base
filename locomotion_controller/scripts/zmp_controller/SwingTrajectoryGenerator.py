import numpy as np

class SwingTrajectoryGenerator():
    def __init__(self, 
                 sim_freq=240,
                 ef_init_x=0.149,
                 ef_init_y=0.13,
                 robot_height=0.17,
                 ) -> None:
        self.inc = 1/sim_freq
        self.ef_init_x = ef_init_x
        self.ef_init_y = ef_init_y
        self.robot_height = robot_height

        self.t_zr = [0]*4
        self.t_zd = [0]*4
        self.t_xsw = [0]*4
        self.t_ysw = [0]*4
        self.a_zr = [0]*4
        self.a_zd = [0]*4
        self.a_xsw = [0]*4
        self.a_ysw = [0]*4

        self.pre_p_rise = [0]*4
        self.pre_p_desc = [0]*4
        self.pre_px_swing = [0]*4
        self.pre_px_stance = [0]*4
        self.pre_py_swing = [0]*4
        self.pre_py_stance = [0]*4
        self.p_start = [[0]*3]*4
        self.p_rise = [[0]*3]*4
        self.p_finish = [[0]*3]*4

        self.p_x=[self.ef_init_x, self.ef_init_x, -self.ef_init_x, -self.ef_init_x]
        self.p_y=[-self.ef_init_y, self.ef_init_y, -self.ef_init_y, self.ef_init_y]
        self.p_z=[-self.robot_height]*4
        self.p_ref = [self.p_x[0], self.p_y[0], self.p_z[0], 
                        self.p_x[1], self.p_y[1], self.p_z[1], 
                        self.p_x[2], self.p_y[2], self.p_z[2], 
                        self.p_x[3], self.p_y[3], self.p_z[3]]

        self.tf = [0]*4

    def calc_a(self, p0, pf, tf, v0=0, vf=0):
        a0 = p0
        a1 = v0
        a2 = (3/tf**2)*(pf - p0) - 2*v0/tf - vf/tf
        a3 = -(2/tf**3)*(pf - p0) + (v0 + vf)/tf**2
        return [a0, a1, a2, a3]
    
    def map_z_rising(self, leg_num, it, p_start, p_rise, tf, cnt):
        # if it % (tf/2) < self.inc:
        if cnt % (self.cnt_stride/2) == 0:
            self.a_zr[leg_num] = self.calc_a(p_start[2], p_rise[2], tf/2, 0, 0)
            self.t_zr[leg_num] = 0#self.inc
        
        p_zr = self.a_zr[leg_num][0] + self.a_zr[leg_num][1]*self.t_zr[leg_num] + self.a_zr[leg_num][2]*self.t_zr[leg_num]**2 + self.a_zr[leg_num][3]*self.t_zr[leg_num]**3
        self.t_zr[leg_num] += self.inc

        if p_rise[2] != self.pre_p_rise[leg_num]:
            v_zr = self.a_zr[leg_num][1] + 2*self.a_zr[leg_num][2]*self.t_zr[leg_num] + 3*self.a_zr[leg_num][3]*self.t_zr[leg_num]**2
            self.a_zr[leg_num] = self.calc_a(p_zr, p_rise[2], (tf/2 - (it+self.inc)%(tf/2)), v_zr, 0)
            self.t_zr[leg_num] = self.inc
        self.pre_p_rise[leg_num] = p_rise[2]

        return p_zr
    
    def map_z_descending(self, leg_num, it, p_rise, p_finish, tf, cnt):
        # if it % (tf/2) < self.inc:
        if cnt % (self.cnt_stride/2) == 0:
            self.a_zd[leg_num] = self.calc_a(p_rise[2], p_finish[2], tf/2, 0, 0)
            self.t_zd[leg_num] = self.inc
        
        p_zd = self.a_zd[leg_num][0] + self.a_zd[leg_num][1]*self.t_zd[leg_num] + self.a_zd[leg_num][2]*self.t_zd[leg_num]**2 + self.a_zd[leg_num][3]*self.t_zd[leg_num]**3
        self.t_zd[leg_num] += self.inc

        if p_finish[2] != self.pre_p_desc[leg_num]:
            v_zd = self.a_zd[leg_num][1] + 2*self.a_zd[leg_num][2]*self.t_zd[leg_num] + 3*self.a_zd[leg_num][3]*self.t_zd[leg_num]**2
            self.a_zd[leg_num] = self.calc_a(p_zd, p_finish[2], (tf/2 - (it+self.inc)%(tf/2)), v_zd, 0)
            self.t_zd[leg_num] = self.inc
        self.pre_p_desc[leg_num] = p_finish[2]

        return p_zd

    def map_x_swing(self, leg_num, it, p_start, p_finish, tf, cnt):
        # if it % (tf) < self.inc:
        if cnt % self.cnt_stride == 0:
            self.a_xsw[leg_num] = self.calc_a(p_start[0], p_finish[0], tf, 0, 0)
            self.t_xsw[leg_num] = 0#self.inc
        
        p_xsw = self.a_xsw[leg_num][0] + self.a_xsw[leg_num][1]*self.t_xsw[leg_num] + self.a_xsw[leg_num][2]*self.t_xsw[leg_num]**2 + self.a_xsw[leg_num][3]*self.t_xsw[leg_num]**3
        self.t_xsw[leg_num] += self.inc

        if p_finish[0] != self.pre_px_swing[leg_num]:
            v_xsw = self.a_xsw[leg_num][1] + 2*self.a_xsw[leg_num][2]*self.t_xsw[leg_num] + 3*self.a_xsw[leg_num][3]*self.t_xsw[leg_num]**2
            self.a_xsw[leg_num] = self.calc_a(p_xsw, p_finish[0], (tf - (it+self.inc)%tf), v_xsw, 0)
            self.t_xsw[leg_num] = self.inc
        self.pre_px_swing[leg_num] = p_finish[0]

        return p_xsw
    
    def map_y_swing(self, leg_num, it, p_start, p_finish, tf, cnt):
        # if it % (tf) < self.inc:
        if cnt % self.cnt_stride == 0:
            self.a_ysw[leg_num] = self.calc_a(p_start[1], p_finish[1], tf, 0, 0)
            self.t_ysw[leg_num] = self.inc
        
        p_ysw = self.a_ysw[leg_num][0] + self.a_ysw[leg_num][1]*self.t_ysw[leg_num] + self.a_ysw[leg_num][2]*self.t_ysw[leg_num]**2 + self.a_ysw[leg_num][3]*self.t_ysw[leg_num]**3
        self.t_ysw[leg_num] += self.inc

        if p_finish[1] != self.pre_py_swing[leg_num]:
            v_ysw = self.a_ysw[leg_num][1] + 2*self.a_ysw[leg_num][2]*self.t_ysw[leg_num] + 3*self.a_ysw[leg_num][3]*self.t_ysw[leg_num]**2
            self.a_ysw[leg_num] = self.calc_a(p_ysw, p_finish[1], (tf - (it+self.inc)%tf), v_ysw, 0)
            self.t_ysw[leg_num] = self.inc
        self.pre_py_swing[leg_num] = p_finish[1]

        return p_ysw

    def set_gait_params(self, omega, cnt_stride):
        self.omega = omega
        self.tf = 1/(2*omega)
        self.cnt_stride = cnt_stride

    def set_points(self, p_start, p_rise, p_finish):
        self.p_start = p_start
        self.p_rise = p_rise
        self.p_finish = p_finish
    
    def set_init_points(self, p):
        self.p_init = p

    def step(self, it, cnt, active_legs):
        

        for i in range(4):
            if active_legs[i] == True:
                p_zr = self.map_z_rising(leg_num=i, it=it, p_start=self.p_start[i], p_rise=self.p_rise[i], tf=self.tf, cnt=cnt)
                p_zd = self.map_z_descending(leg_num=i, it=it, p_rise=self.p_rise[i], p_finish=self.p_finish[i], tf=self.tf, cnt=cnt)
                self.p_x[i] = self.map_x_swing(leg_num=i, it=it, p_start=self.p_start[i], p_finish=self.p_finish[i], tf=self.tf, cnt=cnt)
                self.p_y[i] = self.map_y_swing(leg_num=i, it=it, p_start=self.p_start[i], p_finish=self.p_finish[i], tf=self.tf, cnt=cnt)

                if 0 <= cnt < int(self.cnt_stride/2):
                    self.p_z[i] = p_zr
                else:
                    self.p_z[i] = p_zd

                self.p_ref[3*i] = self.p_x[i]
                self.p_ref[3*i+1] = self.p_y[i]
                self.p_ref[3*i+2] = self.p_z[i]

        return self.p_ref