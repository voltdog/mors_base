import numpy as np

class BodyMovingControl():
    def __init__(self) -> None:
        self.lin_pos = [0,0,0]
        self.rotation = [0,0,0]

    def set_body_lin_pos(self, lin_pos):
        self.lin_pos = lin_pos
    
    def set_body_ang_pos(self, ang_pos):
        self.rotation = ang_pos
        for i in range(3):
            if self.rotation[i] > 0.7:
                self.rotation[i] = 0.7
            if self.rotation[i] < -0.7:
                self.rotation[i] = -0.7

    def set_body_ang_pos_incr(self, ang_pos_incr):
        for i in range(3):
            self.rotation[i] += ang_pos_incr[i]
            if self.rotation[i] > 0.7:
                self.rotation[i] = 0.7
            if self.rotation[i] < -0.7:
                self.rotation[i] = -0.7

    def step(self, p):
        p_new = p[:]

        for i in range(4):
            p_new[3*i+0] = p_new[3*i+0]*np.cos(self.rotation[2]) - p_new[3*i+1]*np.sin(self.rotation[2])
            p_new[3*i+1] = p_new[3*i+0]*np.sin(self.rotation[2]) + p_new[3*i+1]*np.cos(self.rotation[2])

            p_new[3*i+1] = p_new[3*i+1]*np.cos(self.rotation[0]) - p_new[3*i+2]*np.sin(self.rotation[0])
            p_new[3*i+2] = p_new[3*i+1]*np.sin(self.rotation[0]) + p_new[3*i+2]*np.cos(self.rotation[0])

            p_new[3*i+0] = p_new[3*i+0]*np.cos(self.rotation[1]) + p_new[3*i+2]*np.sin(self.rotation[1])
            p_new[3*i+2] = -p_new[3*i+0]*np.sin(self.rotation[1]) + p_new[3*i+2]*np.cos(self.rotation[1])

            for j in range(3):
                p_new[3*i + j] -= self.lin_pos[j]

        return p_new