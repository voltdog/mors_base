import lcm
from lcm_msgs.servo_cmd_msg import servo_cmd_msg
from lcm_msgs.servo_state_msg import servo_state_msg


class LCMDataExchange():
    def __init__(self,
                 servo_cmd_cnannel = "SERVO_CMD",
                 servo_state_channel = "SERVO_STATE",
                 ) -> None:
        self.servo_state_channel = servo_state_channel
        self.servo_cmd_cnannel = servo_cmd_cnannel

        self.ref_pos = [0]*12
        self.ref_vel = [0]*12
        self.ref_torq = [0]*12
        self.ref_kp = [0]*12
        self.ref_kd = [0]*12

        self.cur_pos = [0]*12
        self.cur_vel = [0]*12
        self.cur_torq = [0]*12

        self.cmd_msg = servo_cmd_msg()
        self.lc_cmd = lcm.LCM()
        

    def servo_state_callback(self, channel, data):
        state_msg = servo_state_msg.decode(data)

        self.cur_pos = state_msg.position
        self.cur_vel = state_msg.velocity
        self.cur_torq = state_msg.torque


    def servo_state_thread(self):
        print("State thread started")
        lc = lcm.LCM()
        subscription = lc.subscribe(self.servo_state_channel, self.servo_state_callback)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass


    def send_servo_cmd(self, pos, vel, torq, kp, kd):
        
        self.cmd_msg.position = pos[:]
        self.cmd_msg.velocity = vel[:]
        self.cmd_msg.torque = torq[:]
        self.cmd_msg.kp = kp[:]
        self.cmd_msg.kd = kd[:]

        # print(f"kp: {self.cmd_msg.kp}")
        # print(f"kd: {self.cmd_msg.kd}")

        self.lc_cmd.publish(self.servo_cmd_cnannel, self.cmd_msg.encode())


    def get_servo_state(self):
        return self.cur_pos, self.cur_vel, self.cur_torq