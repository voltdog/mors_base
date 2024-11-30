import math
import time
import pybullet as p
import numpy as np

class Lidar():
    def __init__(self, 
                 pybullet_client : p, 
                 robot_id,
                 joint_id : int, 
                 frequency : int, 
                 base_frequency : int, 
                 render=True,
                 
                 angle_min=np.radians(130),
                 angle_max=-np.radians(130),
                 point_num=360,
                 range_min=0.25,
                 range_max=8.0, 
                 ):
        self._pybullet_client = pybullet_client
        self._robot = robot_id
        self._joint_id = joint_id
        self._freq = frequency
        self._sim_freq = base_frequency
        self._render = render
        self._it_max = int(self._sim_freq / self._freq)-1
        self._angle_min = angle_min #
        self._angle_max = angle_max #
        self._point_num = point_num #
        self._range_min = range_min #
        self._range_max = range_max #

        self.lastLidarTime = 0

        self.hit_distance = []

        self.replaceLines=True
        self.rayFrom=[]
        self.rayTo=[]
        self.rayIds=[]
        self.rayHitColor = [1,0,0]
        self.rayMissColor = [0,1,0]

        self.diap = (np.abs(self._angle_min) + np.abs(self._angle_max))/(2.0*np.pi)
        self.offset = -0.5 + self.diap
        self.lidar_updated = False

    def set_params(self, render=True, angle_min=np.radians(130), angle_max=-np.radians(130), point_num=360, range_min=0.25, range_max=8.0):
        self._render = render
        self._angle_min = angle_min 
        self._angle_max = angle_max 
        self._point_num = point_num 
        self._range_min = range_min 
        self._range_max = range_max 

        self.diap = (np.abs(self._angle_min) + np.abs(self._angle_max))/(2.0*np.pi)
        self.offset = -0.5 + self.diap
        self._delta = self._range_max - self._range_min

    def reset(self):
        self.rayFrom=[]
        self.rayTo=[]
        self.rayIds=[]
        self._pybullet_client.removeAllUserDebugItems()
        for i in range (self._point_num):
            #rayFrom.append([0,0,0])
            self.rayFrom.append([self._range_min*math.sin(-self.offset*0.5*2.*math.pi+self.diap*2.*math.pi*float(i)/self._point_num), self._range_min*math.cos(-self.offset*0.5*2.*math.pi+self.diap*2.*math.pi*float(i)/self._point_num),0])
            self.rayTo.append([self._range_max*math.sin(-self.offset*0.5*2.*math.pi+self.diap*2.*math.pi*float(i)/self._point_num), self._range_max*math.cos(-self.offset*0.5*2.*math.pi+self.diap*2.*math.pi*float(i)/self._point_num),0])
            if self._render == True:
                if (self.replaceLines):
                    self.rayIds.append(self._pybullet_client.addUserDebugLine(self.rayFrom[i], self.rayTo[i], self.rayMissColor,parentObjectUniqueId=self._robot, parentLinkIndex=self._joint_id ))
                else:
                    self.rayIds.append(-1)
        
        print("Hello")

    def update(self, it):
        nowLidarTime = it
        #lidar at 20Hz
        self.lidar_updated = False
        if (nowLidarTime-self.lastLidarTime) >= self._it_max:
            #print("Lidar!")
            numThreads=0
            self.hit_distance = []
            results = self._pybullet_client.rayTestBatch(self.rayFrom, self.rayTo, numThreads, parentObjectUniqueId=self._robot, parentLinkIndex=self._joint_id)
            for i in range(self._point_num-1, -1, -1):
                hitObjectUid=results[i][0]
                hitFraction = results[i][2]
                hitPosition = results[i][3]
                # self.hit_positions.append((hitPosition, hitFraction))
                
                if (hitFraction==1.):
                    if self._render == True:
                        self._pybullet_client.addUserDebugLine(self.rayFrom[i], self.rayTo[i], self.rayMissColor,replaceItemUniqueId=self.rayIds[i],parentObjectUniqueId=self._robot, parentLinkIndex=self._joint_id)
                    dist = np.inf
                else:
                    localHitTo = [self.rayFrom[i][0]+hitFraction*(self.rayTo[i][0]-self.rayFrom[i][0]),
                                    self.rayFrom[i][1]+hitFraction*(self.rayTo[i][1]-self.rayFrom[i][1]),
                                    self.rayFrom[i][2]+hitFraction*(self.rayTo[i][2]-self.rayFrom[i][2])]
                    if self._render == True:
                        self._pybullet_client.addUserDebugLine(self.rayFrom[i], localHitTo, self.rayHitColor, replaceItemUniqueId=self.rayIds[i], parentObjectUniqueId=self._robot, parentLinkIndex=self._joint_id)
                    dist = hitFraction * self._delta + self._range_min

                self.hit_distance.append(dist)
            self.lastLidarTime = nowLidarTime
            self.lidar_updated = True

    def get_data(self):
        return self.hit_distance
    
    def is_lidar_updated(self):
        return self.lidar_updated