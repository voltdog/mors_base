#!/usr/bin/env python
import rospy
import time, datetime

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState, BatteryState
from trajectory_msgs.msg import JointTrajectoryPoint

from pyrr import Quaternion
from scipy.spatial.transform import Rotation

class LogWriter():
    def __init__(self) -> None:
        rospy.init_node("log_writer")
        self.rate = rospy.Rate(0.1)#300)

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=10)
        rospy.Subscriber("cmd_pose", Twist, self.cmd_pose_callback, queue_size=10)
        rospy.Subscriber("joint_states", JointState, self.cur_joint_pos_callback, queue_size=1)
        rospy.Subscriber("lcm/servo_cmd", JointTrajectoryPoint, self.ref_joint_pos_callback, queue_size=1)
        rospy.Subscriber("imu/data", Imu, self.imu_callback, queue_size=1)
        rospy.Subscriber("bat", BatteryState, self.battery_callback, queue_size=1)

        self.cmd_vel_msg = Twist()
        self.cmd_pose_msg = Twist()
        self.cur_joints_msg = JointState()
        self.ref_joints_msg = JointTrajectoryPoint()
        self.imu_msg = Imu()
        self.bat_msg = BatteryState()

        filename = f"log_{datetime.datetime.now().strftime('%Y%m%d-%H%M%S')}.csv"
        self.file = open(f"/home/user/mors_ws/src/mors_base/log_writer/logs/{filename}", 'w')

        # cmd_vel[X], cmd_vel[Y], cmd_vel[Z], \
        #     cmd_pose_linear[X], cmd_pose_linear[Y], cmd_pose_linear[Z], cmd_pose_angular[X], cmd_pose_angular[Y], cmd_pose_angular[Z],\
        #     cur_joint[0], cur_joint[1], cur_joint[2], cur_joint[3], cur_joint[4], cur_joint[5], \
        #     cur_joint[6], cur_joint[7], cur_joint[8], cur_joint[9], cur_joint[10], cur_joint[11], \
        #     ref_joint[0], ref_joint[1], ref_joint[2], ref_joint[3], ref_joint[4], ref_joint[5], \
        #     ref_joint[6], ref_joint[7], ref_joint[8], ref_joint[9], ref_joint[10], ref_joint[11], \
        #     roll, pitch, yaw, \
        line = "datetime, t, \
            voltage"
        self.file.write(line + '\n')

        rospy.loginfo("log_writer started")


    def loop(self):
        # start_t = int(datetime.datetime.now().strftime("%H%M%S%f"))
        start_t = time.time()
        while not rospy.is_shutdown():
            # now = datetime.datetime.now()
            # t = (int(now.strftime("%H%M%S%f")) - start_t)/1000000
            now = time.time()
            t = int(now - start_t)+1
            print(t)

            cur_joint_str = ""
            ref_joint_str = ""

            for i in range(12):
                if len(self.cur_joints_msg.position) >= 12:
                    cur_joint_str += f"{self.cur_joints_msg.position[i]}, " #f"1, "#
                else:
                    cur_joint_str += f"0, "
                if len(self.ref_joints_msg.positions) >= 12:   
                    ref_joint_str += f"{self.ref_joints_msg.positions[i]}, " #f"2, "#
                else:
                    ref_joint_str += "0, "

            quat_df = [self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w]
            if quat_df[3] != 0:
                quat = Quaternion(quat_df).inverse
                rot = Rotation.from_quat(quat)
                rot_euler = rot.as_euler('xyz', degrees=False)
            else:
                rot_euler = [0, 0, 0]

            voltage = self.bat_msg.voltage

            # {self.cmd_vel_msg.linear.x}, {self.cmd_vel_msg.linear.y}, {self.cmd_vel_msg.angular.z}, \
            #     {self.cmd_pose_msg.linear.x}, {self.cmd_pose_msg.linear.y}, {self.cmd_pose_msg.linear.z}, \
            #     {self.cmd_pose_msg.angular.x}, {self.cmd_pose_msg.angular.y}, {self.cmd_pose_msg.angular.z}, \
            #     {cur_joint_str}{ref_joint_str}\
            #     {rot_euler[0]}, {rot_euler[0]}, {rot_euler[0]}\
            line = f"{now}, {t}, \
                {voltage}\
                \n"
            self.file.write(line)


            self.rate.sleep()

    def battery_callback(self, msg : BatteryState):
        self.bat_msg = msg

    def cmd_vel_callback(self, msg : Twist):
        self.cmd_vel_msg = msg

    def cmd_pose_callback(self, msg : Twist):
        self.cmd_pose_msg = msg

    def cur_joint_pos_callback(self, msg : JointState):
        self.cur_joints_msg = msg

    def ref_joint_pos_callback(self, msg : JointTrajectoryPoint):
        self.ref_joints_msg = msg

    def imu_callback(self, msg : Imu):
        self.imu_msg = msg

def main():
    lw = LogWriter()
    lw.loop()


if __name__ == '__main__':
    main()
