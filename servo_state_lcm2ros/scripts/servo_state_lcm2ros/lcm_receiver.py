
import lcm
from lcm_msgs import servo_state_msg

from threading import Thread

class LCMReceiver():
    def __init__(self, channel="SERVO_STATE") -> None:

        self.lc = lcm.LCM()
        subscription = self.lc.subscribe(channel, self.callback)

        th = Thread(target=self.check_msg, args=())
        th.daemon = True
        th.start()

        self.position = [0]*12
        self.velocity = [0]*12
        self.torque = [0]*12
        self.names = [b'abad_joint_R1', b'hip_joint_R1', b'knee_joint_R1', 
                      b'abad_joint_L1', b'hip_joint_L1', b'knee_joint_L1', 
                      b'abad_joint_R2', b'hip_joint_R2', b'knee_joint_R2', 
                      b'abad_joint_L2', b'hip_joint_L2', b'knee_joint_L2']

    def check_msg(self):
        print("state_converter: check_servo_state_msg")
        while True:
            self.lc.handle()

    def callback(self, channel, data : servo_state_msg):
        msg = servo_state_msg.decode(data)
        # for i in range(12):
        #     self.position[i] = msg.position[i]
        #     self.velocity[i] = msg.velocity[i]
        #     self.torque[i] = msg.torque[i]
        self.position = msg.position[:]
        self.velocity = msg.velocity[:]
        self.torque = msg.torque[:]

    def get_ros_msg(self):
        return self.position, self.velocity, self.torque, self.names
