import numpy as np

R1 = 0
L1 = 1
R2 = 2
L2 = 3

X = 0
Y = 1
Z = 2

class ZMPTrajectoryGenerator():
    def __init__(self, com_real) -> None:
        self.com_real = com_real

    def set_params(self, cnt_stride, Nl):
        self.cnt_stride = cnt_stride
        self.Nl = Nl

    def find_sup_lines_cross(self, r1, l1, r2, l2):
        a = r1[1] - l2[1]
        b = l2[0] - r1[0]
        c = r1[0]*l2[1] - l2[0]*r1[1]
        d = l1[1] - r2[1]
        e = r2[0] - l1[0]
        f = l1[0]*r2[1] - r2[0]*l1[1]

        y0 = (a*f - d*c)/(d*b - a*e) - self.com_real[Y]
        x0 = -(b*y0 + c)/a - self.com_real[X] 

        return x0, y0
    
    def step(self, it, cnt, leg_lst):
        zmp_milestones = []
        for i in range(len(leg_lst)):
            r1 = leg_lst[i][R1]
            l1 = leg_lst[i][L1]
            r2 = leg_lst[i][R2]
            l2 = leg_lst[i][L2]
            x0, y0 = self.find_sup_lines_cross(r1, l1, r2, l2)
            zmp_milestones.append([x0, y0])

        # найдем траекторию для ref zmp
        if it == 0:
            k = -1
        else:
            k = 0
        zmp_traj_x_tmp = []
        zmp_traj_y_tmp = []
        zmp_time_vec = []
        
        i = 0
        for t in  np.arange(0, self.Nl+self.cnt_stride):
            x = zmp_milestones[i][0]
            y = zmp_milestones[i][1]

            if (t != 0) and (t%self.cnt_stride == 0):
                i += 1
            
            zmp_traj_x_tmp.append(x)
            zmp_traj_y_tmp.append(y)
            zmp_time_vec.append(i)
        
        zmp_traj_x = zmp_traj_x_tmp[cnt:self.Nl+cnt]
        zmp_traj_y = zmp_traj_y_tmp[cnt:self.Nl+cnt]


        return zmp_traj_x, zmp_traj_y