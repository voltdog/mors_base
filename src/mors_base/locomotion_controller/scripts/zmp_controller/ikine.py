from math import *
import numpy as np

MAX_EF = [[0,  0.5], [-0.3,  0.05], [-0.35, -0.1],  #R1
          [0,  0.5], [-0.05, 0.3],  [-0.35, -0.1],  #L1
          [-0.5, 0], [-0.3,  0.05], [-0.35, -0.1],  #R2
          [-0.5, 0], [-0.05, 0.3],  [-0.35, -0.1]]  #L2

class IKineQuadruped(object):
    def __init__(self,
                bx = 0.1655,
                by = 0.066,
                bz = 0.0,
                L1 = 0.09585,
                L2 = 0.13,
                L3 = 0.1485,
                theta_offset = [0, 0, 0],
                ):
        self.bx = bx
        self.by = by
        self.bz = bz
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.theta_offset = np.array(theta_offset)

        self.theta_ref = [0]*12

        self.theta2_prev = 0
    # config: m, x, o
    def calculate(self, p_ref: np.ndarray, config="m"):
        for i in range(4):
            if p_ref[3*i+2] >= -0.1:
                p_ref[3*i+2] = -0.1
        
        self.theta_ref[0:3] = self.ikine_R1(p_ref[:3], config)
        self.theta_ref[3:6] = self.ikine_L1(p_ref[3:6], config)
        self.theta_ref[6:9] = self.ikine_R2(p_ref[6:9], config)
        self.theta_ref[9:12] = self.ikine_L2(p_ref[9:12], config)
        return self.theta_ref

    def restrict_pos(self, p_ref):
        for i in range(12):
            p_ref[i] = np.clip(p_ref[i], MAX_EF[i][0], MAX_EF[i][1])
        return p_ref

    def ikine_R1(self, p_ref: np.ndarray, config="m"):
        if config == "m":
            sign1 = -1
            sign2 = -1
        elif config == "o":
            sign1 = 1
        elif config == "x":
            sign1 = -1
            sign2 = 1

        px = p_ref[2]
        py = -self.by - p_ref[1]
        pz = p_ref[0] - self.bx

        a2 = px**2 + py**2 - self.L1**2
        if a2 < 0:
            a2 = 0
        
        if py > 0:
            theta1 = (atan2(py, px) - atan2(self.L1, -sqrt(a2)))
        else:
            theta1 = (atan2(py, px)+2*pi - atan2(self.L1, -sqrt(a2)))
        
        a3 = (px**2 + py**2 + pz**2 - self.L1**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)
        if a3 > 1:
            theta3 = sign1*acos(1)
        else: 
            if a3 < -1:
                theta3 = sign1*acos(-1)
            else:
                theta3 = sign1*acos(a3)

        a1 = (self.L3*sin(theta3))**2 + (self.L3*cos(theta3) + self.L2)**2 - pz**2
        if a1 < 0 :
            a1 = 0
        
        #print("LEG R1 THETA: {0}".format(theta_cur))
        # if theta_cur[1] < -pi:
        #     theta2 = -sign2*pi + (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, -sqrt(a1))) % -pi
        # else:
        theta2 = (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1))) #- pi

        # if abs(theta2 - theta_cur[1]) >= (pi/2-0.2):
        #     theta2 = theta2 + sign2*pi

        theta = np.array([theta1, -theta2, -theta3]) - self.theta_offset
        return theta

    def ikine_R2(self, p_ref: np.ndarray, config="m"):
        if config == "m":
            sign1 = -1
        elif config == "o":
            sign1 = -1
        elif config == "x":
            sign1 = 1

        px = p_ref[2]
        py = -self.by - p_ref[1]
        pz = p_ref[0] + self.bx

        a2 = px**2 + py**2 - self.L1**2
        if a2 < 0:
            a2 = 0
        
        if py > 0:
            theta1 = (atan2(py, px) - atan2(self.L1, -sqrt(a2)))
        else:
            theta1 = (atan2(py, px)+2*pi - atan2(self.L1, -sqrt(a2)))
        
        a3 = (px**2 + py**2 + pz**2 - self.L1**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)
        if a3 > 1:
            theta3 = sign1*acos(1)
        else: 
            if a3 < -1:
                theta3 = sign1*acos(-1)
            else:
                theta3 = sign1*acos(a3)

        a1 = (self.L3*sin(theta3))**2 + (self.L3*cos(theta3) + self.L2)**2 - pz**2
        if a1 < 0 :
            a1 = 0
        
        #print("LEG R2 THETA: {0}".format(theta_cur))
        # if theta_cur[1] < 0:
        #     if config == "m":
        #         theta2 = (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1))) % pi
        #     elif config == "x":
        #         theta2 = -(pi - (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1))) % pi)
        # else:
        theta2 = (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1))) - pi#% -pi

        # if abs(theta2 - theta_cur[1]) >= (pi-0.2):
        #     theta2 = theta2 % -pi
            
        # print("LEG R2 THETA CUR: {0}; THETA REF: {1}".format(theta_cur[1], theta2))
        theta = np.array([-theta1, -theta2, -theta3]) + self.theta_offset
        return theta

    def ikine_L1(self, p_ref: np.ndarray, config="m"):
        if config == "m":
            sign1 = -1
        elif config == "o":
            sign1 = 1
        elif config == "x":
            sign1 = -1

        px = p_ref[2]
        py = self.by - p_ref[1]
        pz = p_ref[0] - self.bx

        a2 = px**2 + py**2 - self.L1**2
        if a2 < 0:
            a2 = 0
        
        if py > 0:
            theta1 = (atan2(py, px) - atan2(self.L1, sqrt(a2))) - pi
        else:
            theta1 = (atan2(py, px)+2*pi - atan2(self.L1, sqrt(a2))) - pi
        
        a3 = (px**2 + py**2 + pz**2 - self.L1**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)
        if a3 > 1:
            theta3 = sign1*acos(1)
        else: 
            if a3 < -1:
                theta3 = sign1*acos(-1)
            else:
                theta3 = sign1*acos(a3)

        a1 = (self.L3*sin(theta3))**2 + (self.L3*cos(theta3) + self.L2)**2 - pz**2
        if a1 < 0 :
            a1 = 0
        theta2 = (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1)))# % -pi

        theta = np.array([theta1, theta2, theta3]) + self.theta_offset
        return theta

    def ikine_L2(self, p_ref: np.ndarray, config="m"):
        if config == "m":
            sign1 = -1
        elif config == "o":
            sign1 = -1
        elif config == "x":
            sign1 = 1

        px = p_ref[2]
        py = self.by - p_ref[1]
        pz = p_ref[0] + self.bx

        a2 = px**2 + py**2 - self.L1**2
        if a2 < 0:
            a2 = 0
        
        if py > 0:
            theta1 = -((atan2(py, px) - atan2(self.L1, sqrt(a2))) - pi)
        else:
            theta1 = -((atan2(py, px)+2*pi - atan2(self.L1, sqrt(a2))) - pi)
        
        a3 = (px**2 + py**2 + pz**2 - self.L1**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3)
        if a3 > 1:
            theta3 = sign1*acos(1)
        else: 
            if a3 < -1:
                theta3 = sign1*acos(-1)
            else:
                theta3 = sign1*acos(a3)

        a1 = (self.L3*sin(theta3))**2 + (self.L3*cos(theta3) + self.L2)**2 - pz**2
        if a1 < 0 :
            a1 = 0
        theta2 = (atan2(self.L3*cos(theta3)+self.L2, self.L3*sin(theta3)) - atan2(pz, sqrt(a1))) - pi

        theta = np.array([theta1, theta2, theta3]) - self.theta_offset
        return theta