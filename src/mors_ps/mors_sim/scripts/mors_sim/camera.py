import pybullet as p
import pcl
import numpy
import pyquaternion
from sensor_msgs.msg import PointCloud2, PointField


class Camera():
    def __init__(self, 
                 pybullet_client : p, 
                 robot_id, 
                 joint_id : int,
                 frequency = 20, 
                 base_frequency = 240,
                 width = 320,
                 height = 240,
                 pointcloud_enabled = False) -> None:
        
        self._pybullet_client = pybullet_client
        self._robot = robot_id
        self._joint_id = joint_id
        self._freq = frequency
        self._sim_freq = base_frequency
        self._it_max = int(self._sim_freq / self._freq)-1
        self._pointcloud_enabled = pointcloud_enabled

        self.lastCameraTime = 0

        self.rgbImg = 0
        self.depthImg = 0

        self._pixelWidth = width
        self._pixelHeight = height
        self.near = 0.01
        self.far = 5

        self.T3 = numpy.zeros((4, 4))
        self.pub_pointcloud = PointCloud2()

        self.camera_updated = False

        self.T1 = numpy.array(numpy.mat([[ 0,  0, 1, 0.0], 
                                    [-1,  0, 0, 0.0],
                                    [ 0, -1, 0, 0.0], 
                                    [ 0,  0, 0, 1.0]]))
        
        self.set_params(freq=self._freq,
                        width=self._pixelWidth,
                        height=self._pixelHeight,
                        pointcloud_enabled=self._pointcloud_enabled)


    def set_params(self, freq, width, height, pointcloud_enabled):
        self._freq = freq
        self._pixelWidth = width
        self._pixelHeight = height
        self._pointcloud_enabled = pointcloud_enabled
        self._it_max = int(self._sim_freq / self._freq)-1

        self.camInfo = self._pybullet_client.getDebugVisualizerCamera()
        self.aspect = float(self._pixelWidth) / float(self._pixelHeight)
        self.projMat = p.computeProjectionMatrixFOV(60, self.aspect, self.near, self.far)

        self.h = numpy.arange(self._pixelHeight)
        self.w = numpy.arange(self._pixelWidth)

        self.pub_pointcloud.header.frame_id = "camera_frame"
        self.pub_pointcloud.height = 1
        self.pub_pointcloud.point_step = 12
        self.pub_pointcloud.row_step = 12
        self.pub_pointcloud.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]


    def update(self):
        
        ls = p.getLinkState(self._robot, self._joint_id, computeForwardKinematics=True)
        self.camPos = ls[0]
        self.camOrn = ls[1]
        camMat = p.getMatrixFromQuaternion(self.camOrn)

        forwardVec = [camMat[0],camMat[3],camMat[6]]
        camUpVec =  [camMat[2],camMat[5],camMat[8]]
        self.camTarget = [self.camPos[0]+forwardVec[0]*10,self.camPos[1]+forwardVec[1]*10,self.camPos[2]+forwardVec[2]*10]
        viewMat = p.computeViewMatrix(self.camPos, self.camTarget, camUpVec)
        
        width, height, self.rgbImg, self.depthImg, _ = p.getCameraImage(
            self._pixelWidth,
            self._pixelHeight,
            viewMatrix=viewMat,
            projectionMatrix=self.projMat, 
            renderer=p.ER_BULLET_HARDWARE_OPENGL)
        
        if self._pointcloud_enabled:
            self.calc_pointcloud()

        self.camera_updated = True


    def is_camera_updated(self):
        return self.camera_updated


    def get_rgb_image(self):
        return self.rgbImg
    

    def get_depth_image(self):
        return self.depthImg


    def get_pointcloud(self):
        return self.pub_pointcloud


    def calc_pointcloud(self):
        pc_list = []
        # pcl_data = pcl.PointCloud()
        fx = (self._pixelWidth*self.projMat[0]) / 2.0
        fy = (self._pixelHeight*self.projMat[5]) / 2.0
        cx = (1-self.projMat[2]) * self._pixelWidth / 2.0
        cy = (1+self.projMat[6]) * self._pixelHeight / 2.0
        cloud_point = [0] * self._pixelWidth * self._pixelHeight * 3
        depthBuffer = numpy.reshape(self.depthImg, [self._pixelHeight, self._pixelWidth])
        depth = self.far * self.near / (self.far - (self.far - self.near) * depthBuffer)

        for h in range(0, self._pixelHeight):
            for w in range(0, self._pixelWidth):
                Z = float(depth[h][w])
                if (Z > 4 or Z < 0.01):
                    continue
                X = (w - cx) * Z / fx
                Y = (h - cy) * Z / fy
                XYZ_ = numpy.mat([[X], [Y], [Z], [1]])
                XYZ = numpy.array(self.T1*XYZ_)
                X = float(XYZ[0])
                Y = float(XYZ[1])
                Z = float(XYZ[2])
                cloud_point[h * self._pixelWidth * 3 + w * 3 + 0] = float(X)
                cloud_point[h * self._pixelWidth * 3 + w * 3 + 1] = float(Y)
                cloud_point[h * self._pixelWidth * 3 + w * 3 + 2] = float(Z)
                pc_list.append([X, Y, Z])

        self.pub_pointcloud.width = len(pc_list)
        self.pub_pointcloud.data = numpy.asarray(pc_list, numpy.float32).tostring()
