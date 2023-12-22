import numpy as np
from control.matlab import dare # for solving the discrete algebraic Riccati equation

class ZMP_Preview_Control():
    def __init__(self, 
                sim_freq = 240):
        self.g = 9.81
        self.dt = 1/sim_freq
        self.sim_freq = sim_freq

        self.ux_3 = np.asmatrix(np.zeros((1, 1)))
        self.uy_3 = np.asmatrix(np.zeros((1, 1)))
        self.COM_x_3 = np.asmatrix(np.zeros((3, 1)))
        self.COM_y_3 = np.asmatrix(np.zeros((3, 1)))

        self.e_x_3 = np.zeros((1, 1))
        self.e_y_3 = np.zeros((1, 1))

        self.sum_e_x = 0
        self.sum_e_y = 0
    
    def __set_robot_params(self, z_c):
        self.z_c = z_c
    
    def __set_gait_params(self, t_step):
        self.t_step = t_step
    
    def __set_regulator_params(self, t_preview, Q, R):
        self.t_preview = t_preview
        self.N_preview = int(self.t_preview*self.sim_freq)
        self.Q = Q
        self.R = R

    def __define_basic_matrix(self):
        A = np.mat(([1, self.dt, self.dt**2/2],
                [0, 1, self.dt],
                [0, 0, 1]))
        B = np.mat((self.dt**3/6, self.dt**2/2, self.dt)).T
        C = np.mat((1, 0, -self.z_c/self.g))

        return A, B, C
    
    def __calculatePreviewControlParams(self, A, B, C, Q, R, N):
        C_dot_A = C*A
        C_dot_B = C*B

        A_tilde = np.matrix([[1, C_dot_A[0,0], C_dot_A[0,1], C_dot_A[0,2]],
                                [0, A[0,0], A[0,1], A[0,2]],
                                [0, A[1,0], A[1,1], A[1,2]],
                                [0, A[2,0], A[2,1], A[2,2]]])
        B_tilde = np.matrix([[C_dot_B[0,0]],
                                [B[0,0]],
                                [B[1,0]],
                                [B[2,0]]])
        C_tilde = np.matrix([[1, 0, 0, 0]])

        [P_tilde, _, _] = dare(A_tilde, B_tilde, C_tilde.T*Q*C_tilde, R)

        K_tilde = (R + B_tilde.T*P_tilde*B_tilde).I*(B_tilde.T*P_tilde*A_tilde)

        Ks = K_tilde[0, 0]
        Kx = K_tilde[0, 1:]

        Ac_tilde = A_tilde - B_tilde*K_tilde

        G = np.zeros((1, N))

        G[0] = -Ks
        I_tilde = np.matrix([[1],[0],[0],[0]])
        X_tilde = -Ac_tilde.T*P_tilde*I_tilde

        for i in range(N):
            G[0,i] = (R + B_tilde.T*P_tilde*B_tilde).I*(B_tilde.T)*X_tilde
            X_tilde = Ac_tilde.T*X_tilde

        return Ks, Kx, G

    def init_regulator(self, z_c, t_step, t_preview, Q, R):
        self.__set_robot_params(z_c=z_c)
        self.__set_gait_params(t_step=t_step)
        self.__set_regulator_params(t_preview=t_preview, Q=Q, R=R)
        self.A, self.B, self.C = self.__define_basic_matrix()
        self.Ks, self.Kx, self.G = self.__calculatePreviewControlParams(self.A, self.B, self.C, Q, R, self.N_preview)
    
    def set_gait_params(self, t_step):
        self.__set_gait_params(t_step=t_step)
        # self.__set_regulator_params(t_preview=t_preview, Q=Q, R=R)

    def set_zmp_trajectory(self, zmp_x_ref, zmp_y_ref):
        self.ZMP_x_ref = zmp_x_ref
        self.ZMP_y_ref = zmp_y_ref

    def step(self):
        ZMP_x_preview = np.asmatrix(self.ZMP_x_ref[:])
        ZMP_y_preview = np.asmatrix(self.ZMP_y_ref[:])

        # update ZMP
        ZMP_x = self.C*self.COM_x_3
        ZMP_y = self.C*self.COM_y_3

        # calculate errors
        self.e_x_3 = ZMP_x - self.ZMP_x_ref[0] 
        self.e_y_3 = ZMP_y - self.ZMP_y_ref[0]
        self.sum_e_x += self.e_x_3
        self.sum_e_y += self.e_y_3

        # update u
        self.ux_3 = -self.Ks*self.sum_e_x - self.Kx*self.COM_x_3 - self.G*ZMP_x_preview
        self.uy_3 = -self.Ks*self.sum_e_y - self.Kx*self.COM_y_3 - self.G*ZMP_y_preview

        # update COM state
        self.COM_x_3 = self.A*self.COM_x_3 + self.B*self.ux_3
        self.COM_y_3 = self.A*self.COM_y_3 + self.B*self.uy_3

        # print(ZMP_x[0,0])

        return [self.COM_x_3[0,0], self.COM_y_3[0,0]], [ZMP_x[0,0], ZMP_y[0,0]]
