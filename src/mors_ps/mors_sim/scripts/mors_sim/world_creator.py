import numpy as np
import random
import pybullet as p
from  mors_sim import gazebo_world_parser

import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

class WorldCreator():
    def __init__(self,
                pybullet_client : p,
                spinning_friction = 0.0063,
                lateral_friction = 1.0
                ) -> None:
        self._pybullet_client = pybullet_client
        self.spinning_friction = spinning_friction
        self.lateral_friction = lateral_friction

        self.models_addr = parentdir + "/models/"
        self.gazebo_world_addr = parentdir + "/worlds/"
        # print("ADDRESS")
        # print(self.models_addr)

    def create_world(self, world_name : str, lateralFriction=1.0, spinningFriction=0.0063):
        self.spinning_friction = spinningFriction
        self.lateral_friction = lateralFriction

        if world_name.find("gazebo") == -1:
            if world_name == "empty":
                self._create_empty_world()
            elif world_name == "random1":
                self._create_random1_world()
            elif world_name == "random2":
                self._create_random2_world()
            else:
                self._create_empty_world()
        else:
            
            self._create_gazebo_world(world_name.replace('gazebo_', ''))

    def _create_empty_world(self):
        planeShape = self._pybullet_client.createCollisionShape(shapeType=self._pybullet_client.GEOM_PLANE)
        ground_id = self._pybullet_client.createMultiBody(0, planeShape)
        self._pybullet_client.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
        self._pybullet_client.changeDynamics(ground_id, -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

    def _create_random1_world(self, heightPerturbationRange=0.03, numHeightfieldRows=256, numHeightfieldColumns=256):
        heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns
        for j in range(int(numHeightfieldColumns/2)):
            for i in range(int(numHeightfieldRows/2)):
                height = random.uniform(0, heightPerturbationRange)
                heightfieldData[2*i+2*j*numHeightfieldRows] = height
                heightfieldData[2*i+1+2*j*numHeightfieldRows] = height
                heightfieldData[2*i+(2*j+1)*numHeightfieldRows] = height
                heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows] = height
        terrainShape = self._pybullet_client.createCollisionShape(
            shapeType=self._pybullet_client.GEOM_HEIGHTFIELD,
            meshScale=[.1, .1, 1],
            heightfieldTextureScaling=(numHeightfieldRows-1)/2,
            heightfieldData=heightfieldData,
            numHeightfieldRows=numHeightfieldRows,
            numHeightfieldColumns=numHeightfieldColumns)
        ground_id = self._pybullet_client.createMultiBody(0, terrainShape)
        self._pybullet_client.changeVisualShape(ground_id, -1, rgbaColor=[0, 1, 0, 0.7])
        self._pybullet_client.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
        self._pybullet_client.changeDynamics(ground_id, -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

    def _create_random2_world(self):
        terrain_shape = self._pybullet_client.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                meshScale=[.5, .5, .5],
                fileName="heightmaps/ground0.txt",
                heightfieldTextureScaling=128)
        ground_id = self._pybullet_client.createMultiBody(0, terrain_shape)
        textureId = self._pybullet_client.loadTexture(self.models_addr + "grass.png")
        self._pybullet_client.changeVisualShape(ground_id, -1, textureUniqueId=textureId)
        self._pybullet_client.resetBasePositionAndOrientation(ground_id, [1, 0, 0.2], [0, 0, 0, 1])
        self._pybullet_client.changeDynamics(ground_id, -1, lateralFriction=self.lateral_friction, spinningFriction=self.spinning_friction)

    def _create_gazebo_world(self, world_name : str):
        os.chdir(parentdir)
        self._pybullet_client.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        gazebo_world_parser.parseWorld(self._pybullet_client, filepath=self.gazebo_world_addr + world_name + ".world")
        self._pybullet_client.configureDebugVisualizer(shadowMapResolution=8192)
        self._pybullet_client.configureDebugVisualizer(shadowMapWorldSize=25)
        self._pybullet_client.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    