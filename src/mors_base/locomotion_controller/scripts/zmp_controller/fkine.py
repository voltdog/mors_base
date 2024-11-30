import numpy as np

class FKineQuadruped(object):
    def __init__(self,
                bx = 0.1655,
                by = 0.066,
                bz = 0.0,
                L1 = 0.09585,
                L2 = 0.13,
                L3 = 0.1485,
                theta_offset = [0, 0, 0]):        
        self.bx = bx
        self.by = by
        self.bz = bz
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.theta_offset = np.array(theta_offset)

        self.ef_curs = [0]*12
    def fkine_R1(self, theta: np.ndarray):
        theta = theta + self.theta_offset
        ef_pos = np.array([0, 0, 0], dtype=np.float32)
        ef_pos[0] = self.bx + self.L3*np.cos(theta[1]+theta[2]) + self.L2*np.cos(theta[1])
        ef_pos[1] = -self.by - self.L3*np.sin(theta[0])*np.sin(theta[1]+theta[2]) - self.L1*np.cos(theta[0]) - self.L2*np.sin(theta[0])*np.sin(theta[1])
        ef_pos[2] = self.L3*np.cos(theta[0])*np.sin(theta[1]+theta[2]) - self.L1*np.sin(theta[0]) + self.L2*np.cos(theta[0])*np.sin(theta[1])
        return ef_pos

    def fkine_R2(self, theta: np.ndarray):
        theta = theta - self.theta_offset
        ef_pos = np.array([0, 0, 0], dtype=np.float32)
        ef_pos[0] = -(self.bx + self.L3*np.cos(theta[1]+theta[2]) + self.L2*np.cos(theta[1]))
        ef_pos[1] = -self.by - self.L3*np.sin(theta[0])*np.sin(theta[1]+theta[2]) - self.L1*np.cos(theta[0]) - self.L2*np.sin(theta[0])*np.sin(theta[1])
        ef_pos[2] = -(self.L3*np.cos(theta[0])*np.sin(theta[1]+theta[2]) - self.L1*np.sin(theta[0]) + self.L2*np.cos(theta[0])*np.sin(theta[1]))
        return ef_pos

    def fkine_L1(self, theta: np.ndarray):
        theta = theta - self.theta_offset
        ef_pos = np.array([0, 0, 0], dtype=np.float32)
        ef_pos[0] = self.bx + self.L3*np.cos(theta[1]+theta[2]) + self.L2*np.cos(theta[1])
        ef_pos[1] = -(-self.by - self.L3*np.sin(theta[0])*np.sin(theta[1]+theta[2]) - self.L1*np.cos(theta[0]) - self.L2*np.sin(theta[0])*np.sin(theta[1]))
        ef_pos[2] = -(self.L3*np.cos(theta[0])*np.sin(theta[1]+theta[2]) - self.L1*np.sin(theta[0]) + self.L2*np.cos(theta[0])*np.sin(theta[1]))
        return ef_pos

    def fkine_L2(self, theta: np.ndarray):
        theta = theta + self.theta_offset
        ef_pos = np.array([0, 0, 0], dtype=np.float32)
        ef_pos[0] = -(self.bx + self.L3*np.cos(theta[1]+theta[2]) + self.L2*np.cos(theta[1]))
        ef_pos[1] = -(-self.by - self.L3*np.sin(theta[0])*np.sin(theta[1]+theta[2]) - self.L1*np.cos(theta[0]) - self.L2*np.sin(theta[0])*np.sin(theta[1]))
        ef_pos[2] = self.L3*np.cos(theta[0])*np.sin(theta[1]+theta[2]) - self.L1*np.sin(theta[0]) + self.L2*np.cos(theta[0])*np.sin(theta[1])
        return ef_pos

    def calculate(self, theta: np.ndarray):
        self.ef_curs[0:3] = self.fkine_R1(theta[0:3])
        self.ef_curs[3:6] = self.fkine_L1(theta[3:6])
        self.ef_curs[6:9] = self.fkine_R2(theta[6:9])
        self.ef_curs[9:] = self.fkine_L2(theta[9:])
        return self.ef_curs