import rospy
from sensor_msgs.msg import Joy
from diagnostic_msgs.msg import DiagnosticArray
from math import *
import random

RX = 0
RY = 1
LX = 2
LY = 3
R1 = 4
R2 = 5
R3 = 6
L1 = 7
L2 = 8
L3 = 9
UP = 10
DOWN = 11
LEFT = 12
RIGHT = 13
CROSS = 14
SQUARE = 15
TRIANGLE = 16
CIRCLE = 17
OPTIONS = 18
SHARE = 19
PS = 20
STATUS = 21
 
class JOY_CONVERTER(object):
    def __init__(self, value_topic="joy", status_topic='diagnostics'):
        rospy.Subscriber(value_topic, Joy, self.joy_callback_values, queue_size=1)
        rospy.Subscriber(status_topic, DiagnosticArray, self.joy_callback_status, queue_size=1)

        self._joy_value = [0]*22
        # self._config = rospy.get_param('~joystick_config')
    def set_joy_config(self, config:dict):
        self._config = config
        
    def get_joy_data(self)->list:
        return self._joy_value

    def joy_callback_status(self, msg:DiagnosticArray):
        self._joy_value[STATUS] = msg.status[0].message

    def joy_callback_values(self, msg:Joy):
        concat_msg = [msg.axes, msg.buttons]

        self._joy_value[RX] = -concat_msg[self._config['rstick_x'][0]][self._config['rstick_x'][1]]
        self._joy_value[RY] = concat_msg[self._config['rstick_y'][0]][self._config['rstick_y'][1]]
        self._joy_value[LX] = -concat_msg[self._config['lstick_x'][0]][self._config['lstick_x'][1]]
        self._joy_value[LY] = concat_msg[self._config['lstick_y'][0]][self._config['lstick_y'][1]]

        self._joy_value[R1] = concat_msg[self._config['rbtn_small'][0]][self._config['rbtn_small'][1]]
        self._joy_value[R2] = concat_msg[self._config['rbtn_big'][0]][self._config['rbtn_big'][1]]
        self._joy_value[R3] = concat_msg[self._config['rstick_btn'][0]][self._config['rstick_btn'][1]]
        self._joy_value[L1] = concat_msg[self._config['lbtn_small'][0]][self._config['lbtn_small'][1]]
        self._joy_value[L2] = concat_msg[self._config['lbtn_big'][0]][self._config['lbtn_big'][1]]
        self._joy_value[L3] = concat_msg[self._config['lstick_btn'][0]][self._config['lstick_btn'][1]]

        self._joy_value[UP] = 0
        self._joy_value[DOWN] = 0
        self._joy_value[LEFT] = 0
        self._joy_value[RIGHT] = 0
        if concat_msg[self._config['lpad_y'][0]][self._config['lpad_y'][1]] == 1:
            self._joy_value[UP] = 1
        elif concat_msg[self._config['lpad_y'][0]][self._config['lpad_y'][1]] == -1:
            self._joy_value[DOWN] = 1
        if concat_msg[self._config['lpad_x'][0]][self._config['lpad_x'][1]] == 1:
            self._joy_value[LEFT] = 1
        elif concat_msg[self._config['lpad_x'][0]][self._config['lpad_x'][1]] == -1:
            self._joy_value[RIGHT] = 1

        self._joy_value[CROSS] = concat_msg[self._config['rpad_down'][0]][self._config['rpad_down'][1]]
        self._joy_value[SQUARE] = concat_msg[self._config['rpad_left'][0]][self._config['rpad_left'][1]]
        self._joy_value[TRIANGLE] = concat_msg[self._config['rpad_up'][0]][self._config['rpad_up'][1]]
        self._joy_value[CIRCLE] = concat_msg[self._config['rpad_right'][0]][self._config['rpad_right'][1]]

        self._joy_value[OPTIONS] = concat_msg[self._config['start_btn'][0]][self._config['start_btn'][1]]
        self._joy_value[SHARE] = concat_msg[self._config['mode_btn'][0]][self._config['mode_btn'][1]]
        self._joy_value[PS] = concat_msg[self._config['joy_btn'][0]][self._config['joy_btn'][1]]

        # print(self._joy_value[OPTIONS])