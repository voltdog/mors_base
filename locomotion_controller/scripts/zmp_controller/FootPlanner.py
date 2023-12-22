import numpy as np

R1 = 0
L1 = 1
R2 = 2
L2 = 3

X = 0
Y = 1
Z = 2

class FootPlanner():
    def __init__(self,
                 ef_init_x=0.149,
                 ef_init_y=0.13,
                 ) -> None:
        self.ef_init_x = ef_init_x
        self.ef_init_y = ef_init_y
        
        self.leg_lst = [[[self.ef_init_x, -self.ef_init_y], 
                    [self.ef_init_x, self.ef_init_y], 
                    [-self.ef_init_x, -self.ef_init_y], 
                    [-self.ef_init_x, self.ef_init_y]]]
        
        self.cur_active_legs = np.array([False, True, True, False])
        self.ftr_active_legs = np.array([False, True, True, False])
        self.phi_lst = [0]

        self.front_x_pos =  self.ef_init_x
        self.rear_x_pos  = -self.ef_init_x
        self.left_y_pos = self.ef_init_y
        self.right_y_pos = -self.ef_init_y
        self.PHI = 0

        self.x_l1 = self.ef_init_x
        self.x_r1 = self.ef_init_x
        self.x_l2 = -self.ef_init_x
        self.x_r2 = -self.ef_init_x
        self.y_r1 = -self.ef_init_y
        self.y_l1 = self.ef_init_y
        self.y_r2 = -self.ef_init_y
        self.y_l2 = self.ef_init_y

        self.act_l1 = 1
        self.act_l2 = 2
        self.act_y_l1 = 1
        self.act_y_l2 = 2

    def set_params(self, preview_strides, Sl_x, Sl_y):#, v_err):
        self.preview_strides = preview_strides
        self.Sl_x = Sl_x
        self.Sl_y = Sl_y
        # self.v_err = v_err

    def step(self, cnt):
        if cnt == 1:
            if len(self.leg_lst) >= 2:
                if self.cur_active_legs[L1] and self.cur_active_legs[R2]:
                    self.act_l1 = 1
                    self.act_l2 = 2
                    self.act_y_l1 = 1
                    self.act_y_l2 = 2
                else:
                    self.act_l1 = 0
                    self.act_l2 = 3
                    self.act_y_l1 = 3
                    self.act_y_l2 = 0
                self.front_x_pos = self.leg_lst[1][self.act_l1][X]
                rear_x_pos = self.leg_lst[1][self.act_l2][X]
                left_y_pos = self.leg_lst[1][self.act_y_l1][Y]
                right_y_pos = self.leg_lst[1][self.act_y_l2][Y]
            
                [self.x_r1, self.y_r1], [self.x_l1, self.y_l1], [self.x_r2, self.y_r2], [self.x_l2, self.y_l2] = self.leg_lst[1]
                self.leg_lst = [self.leg_lst[1]]
                self.cur_active_legs = np.invert(self.cur_active_legs)
        else:
            self.front_x_pos =  self.leg_lst[0][self.act_l1][X]
            self.rear_x_pos = self.leg_lst[0][self.act_l2][X]
            self.left_y_pos = self.leg_lst[0][self.act_y_l1][Y]
            self.right_y_pos = self.leg_lst[0][self.act_y_l2][Y]
            self.leg_lst = [self.leg_lst[0]]
            [self.x_r1, self.y_r1], [self.x_l1, self.y_l1], [self.x_r2, self.y_r2], [self.x_l2, self.y_l2] = self.leg_lst[0]
            
        self.ftr_active_legs = self.cur_active_legs[:]
        for i in range(self.preview_strides):
            self.front_x_pos += (self.Sl_x/2)# - 0.1*self.v_err[0])
            self.rear_x_pos += (self.Sl_x/2)# - 0.1*self.v_err[0])
            self.left_y_pos += (self.Sl_y/2)# - 0.1*self.v_err[1])
            self.right_y_pos += (self.Sl_y/2)# - 0.1*self.v_err[1])

            if self.ftr_active_legs[L1] and self.ftr_active_legs[R2]:
                self.x_l1 = self.front_x_pos 
                self.y_l1 = self.left_y_pos 
                self.x_r2 = self.rear_x_pos 
                self.y_r2 = self.right_y_pos 
                self.ftr_active_legs = np.invert(self.ftr_active_legs)
            elif self.ftr_active_legs[L2] and self.ftr_active_legs[R1]:
                self.x_r1 = self.front_x_pos
                self.y_r1 = self.right_y_pos 
                self.x_l2 = self.rear_x_pos 
                self.y_l2 = self.left_y_pos
                self.ftr_active_legs = np.invert(self.ftr_active_legs)
            
            self.leg_lst.append([[self.x_r1, self.y_r1], [self.x_l1, self.y_l1], [self.x_r2, self.y_r2], [self.x_l2, self.y_l2]])
        
        return self.leg_lst, self.cur_active_legs