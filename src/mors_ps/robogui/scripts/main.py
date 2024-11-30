import sys
from PyQt5 import QtGui
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QFrame, QWidget, QTableWidgetItem
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QFileDialog, QScrollArea, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QSlider, QStackedLayout, QPushButton, QTreeView, QTableWidget, QTableWidgetItem, QTreeWidgetItem, QCheckBox

from threading import Thread

from robogui.robogui_window import Ui_Dialog

from robogui.ros_connector import ROS_Connector
from robogui.ros_connector import CmdData, ROSParams
from robogui.data_containers import StateDataDict
from robogui.DnDWidgets import MyTableWidget, MyTreeWidget
from robogui.simple_widgets import QHLine, QVLine
from robogui.plot2d import Plot2D

from functools import partial

import time
import subprocess
import csv
import configparser
import os

VERSION = "0.0.1"



class MainWindow(QtWidgets.QDialog, Ui_Dialog):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        result = subprocess.run(["rospack find robogui"], shell=True, capture_output=True)
        cur_dir = result.stdout.decode("utf-8")
        self.setupUi(self)
        self.setWindowFlags(Qt.Window)
        # set window params
        self.title = "RoboGUI v." + VERSION
        self.top = 500
        self.left = 400
        self.width = 1600#3400
        self.height = 1500
        self.setWindowIcon(QtGui.QIcon(cur_dir[:-1]+"/scripts/robogui/icon.png"))
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.splitter.setSizes([600, 450, 490, 1760])
        self.walking_btn.setAutoDefault(False)
        self.leg_btn.setAutoDefault(False)
        self.body_btn.setAutoDefault(False)
        self.cute_actions_btn.setAutoDefault(False)
        self.joints_btn.setAutoDefault(False)
        
        self.kill_threads = False
        # init ros stuff
        self.cmd_data = CmdData()
        self.state_data = StateDataDict()
        self.ros_prms = ROSParams()
        
        self.ros_connector = ROS_Connector()
        self.ros_prms = self.ros_connector.get_params()
        self.ros_connector.set_mode(self.ros_prms.walk_mode)
        # start ros thread
        self.th1 = Thread(target=self.ros_connector.loop, args=())
        self.th1.daemon = True
        self.th1.start()
        
        # robot variables
        self.joint_kp = [0]*12
        self.joint_kd = [0]*12
        self.cur_device = 0

        # self.ref_body_mins = [-0.1, -0.06, -0.1, -0.5, -0.5, -0.5]
        self.ref_body_mins = [-self.ros_prms.max_lin_x, -self.ros_prms.max_lin_y, -self.ros_prms.min_lin_z,
                              -self.ros_prms.max_angle_x, -self.ros_prms.max_angle_y, -self.ros_prms.max_angle_z]
        # self.ref_body_maxes = [0.1, 0.06, 0.035, 0.5, 0.5, 0.5]
        self.ref_body_maxes = [self.ros_prms.max_lin_x, self.ros_prms.max_lin_y, self.ros_prms.max_lin_z,
                              self.ros_prms.max_angle_x, self.ros_prms.max_angle_y, self.ros_prms.max_angle_z]
        self.ref_body_inits = [0, 0, 0, 0, 0, 0]

        self.ref_joint_pos_min = [-self.ros_prms.max_abad]*12
        self.ref_joint_pos_max = [self.ros_prms.max_abad]*12
        self.ref_joint_pos_inits = [0]*12
        self.ref_joint_vel_min = [-self.ros_prms.max_joint_vel]*12
        self.ref_joint_vel_max = [self.ros_prms.max_joint_vel]*12
        self.ref_joint_vel_inits = [0]*12
        self.ref_joint_torq_min = [-self.ros_prms.max_joint_torq]*12
        self.ref_joint_torq_max = [self.ros_prms.max_joint_torq]*12
        self.ref_joint_torq_inits = [0]*12
        self.ref_joint_kp_min = [0]*12
        self.ref_joint_kp_max = [self.ros_prms.max_joint_kp]*12
        self.ref_joint_kp_inits = [0]*12
        self.ref_joint_kd_min = [0]*12
        self.ref_joint_kd_max = [self.ros_prms.max_joint_kd]*12
        self.ref_joint_kd_inits = [0]*12
        self.same_kpkd = False

        leg_init_x = 0.0
        leg_init_y = 0.0
        leg_init_z = 0.0
        leg_max_offset_x = self.ros_prms.ef_max_pos_x
        leg_min_offset_y = -self.ros_prms.ef_max_pos_y
        leg_max_offset_y = self.ros_prms.ef_max_pos_y
        leg_max_offset_z = self.ros_prms.ef_max_pos_z
        leg_min_offset_z = -self.ros_prms.ef_min_pos_z
        self.ref_legs_min = [-leg_max_offset_x, leg_min_offset_y, leg_min_offset_z,
                             -leg_max_offset_x, leg_min_offset_y, leg_min_offset_z,
                             -leg_max_offset_x, leg_min_offset_y, leg_min_offset_z,
                             -leg_max_offset_x, leg_min_offset_y, leg_min_offset_z]
        self.ref_legs_max = [ leg_init_x+leg_max_offset_x, leg_max_offset_y, leg_max_offset_z,
                              leg_init_x+leg_max_offset_x,  leg_max_offset_y, leg_max_offset_z,
                             -leg_init_x+leg_max_offset_x, leg_max_offset_y, leg_max_offset_z,
                             -leg_init_x+leg_max_offset_x,  leg_max_offset_y, leg_max_offset_z]
        self.ref_legs_init = [ leg_init_x, -leg_init_y, leg_init_z,
                               leg_init_x,  leg_init_y, leg_init_z,
                              -leg_init_x, -leg_init_y, leg_init_z,
                              -leg_init_x,  leg_init_y, leg_init_z]
        
        self.ref_robot_x_min = -self.ros_prms.max_speed_x
        self.ref_robot_x_max = self.ros_prms.max_speed_x
        self.ref_robot_x_init = 0

        self.ref_robot_y_min = -self.ros_prms.max_speed_y
        self.ref_robot_y_max = self.ros_prms.max_speed_y
        self.ref_robot_y_init = 0
        
        self.ref_robot_z_min = -self.ros_prms.max_speed_z
        self.ref_robot_z_max = self.ros_prms.max_speed_z
        self.ref_robot_z_init = 0

        self.ref_robot_height_min = 0
        self.ref_robot_height_max = 0.15
        self.ref_robot_height_init = 0
 
        # self.ref_robot_length_min = 0
        # self.ref_robot_length_max = 0.15
        # self.ref_robot_length_init = 0

        # self.ref_robot_freq_min = 0.2
        # self.ref_robot_freq_max = 3.5
        # self.ref_robot_freq_init = 2.5

        for i in range(4):
            self.cmd_data.ef_x[i] = self.ref_legs_init[3*i]
            self.cmd_data.ef_y[i] = self.ref_legs_init[3*i+1]
            self.cmd_data.ef_z[i] = self.ref_legs_init[3*i+2]
        # self.cmd_data.robot_step_freq = self.ref_robot_freq_init
        # create widgets
        self.stacked_layout = QStackedLayout()

        self.walking_widget = QWidget()
        self.legs_widget = QWidget()
        self.body_widget = QWidget()
        self.joints_widget = QWidget()
        self.cute_widget = QWidget()
        
        self.create_walking_control_widgets()
        self.create_legs_control_widgets()
        self.create_body_control_widgets()
        self.create_joints_control_widgets()
        self.create_cute_control_widgets()
        self.create_all_signals_tree()
        self.create_table()
        self.create_plot()

        # start refreshing all widgets thread
        self.th2 = Thread(target=self.refresh_widgets, args=())
        self.th2.daemon = True
        self.th2.start()

        self.verticalLayout_3.addLayout(self.stacked_layout)

        self.stacked_layout.setCurrentIndex(0)

        # other params
        self.ref_body_prm = [0]*6
        self.ref_joint_prm = [0]*12

        # set signals and slots
        self.walking_btn.clicked.connect(self.walking_btn_was_clicked)
        self.leg_btn.clicked.connect(self.leg_btn_was_clicked)
        self.body_btn.clicked.connect(self.body_btn_was_clicked)
        self.joints_btn.clicked.connect(self.joints_btn_was_clicked)
        self.cute_actions_btn.clicked.connect(self.cute_btn_was_clicked)

        for i in range(6):
            self.slider_ref_body_lst[i].valueChanged.connect(partial(self.slider_ref_body_changed, i))
            self.line_ref_body_lst[i].returnPressed.connect(partial(self.line_ref_body_changed, i))

        for i in range(12):
            self.slider_ref_joint_pos_lst[i].valueChanged.connect(partial(self.slider_ref_joint_pos_changed, i))
            self.slider_ref_joint_vel_lst[i].valueChanged.connect(partial(self.slider_ref_joint_vel_changed, i))
            self.slider_ref_joint_torq_lst[i].valueChanged.connect(partial(self.slider_ref_joint_torq_changed, i))
            self.slider_ref_joint_kp_lst[i].valueChanged.connect(partial(self.slider_ref_joint_kp_changed, i))
            self.slider_ref_joint_kd_lst[i].valueChanged.connect(partial(self.slider_ref_joint_kd_changed, i))

            self.line_ref_joint_pos_lst[i].returnPressed.connect(partial(self.line_ref_joint_pos_changed, i))
            self.line_ref_joint_vel_lst[i].returnPressed.connect(partial(self.line_ref_joint_vel_changed, i))
            self.line_ref_joint_torq_lst[i].returnPressed.connect(partial(self.line_ref_joint_torq_changed, i))
            self.line_ref_joint_kp_lst[i].returnPressed.connect(partial(self.line_ref_joint_kp_changed, i))
            self.line_ref_joint_kd_lst[i].returnPressed.connect(partial(self.line_ref_joint_kd_changed, i))

            self.slider_ref_legs_lst[i].valueChanged.connect(partial(self.slider_ref_legs_changed, i))
            self.line_ref_legs_lst[i].returnPressed.connect(partial(self.line_ref_legs_changed, i))

        self.btn_ref_joints.clicked.connect(self.btn_ref_joints_clicked)
        # self.btn_hold_servos.clicked.connect(self.btn_hold_servos_clicked)
        self.btn_ref_legs.clicked.connect(self.btn_ref_legs_clicked)
        self.btn_ref_body.clicked.connect(self.btn_ref_body_clicked)

        self.btn_stand_up.clicked.connect(self.btn_stand_up_clicked)
        self.btn_lay_down.clicked.connect(self.btn_lay_down_clicked)

        self.slider_ref_x.valueChanged.connect(self.slider_ref_x_changed)
        self.slider_ref_y.valueChanged.connect(self.slider_ref_y_changed)
        self.slider_ref_z.valueChanged.connect(self.slider_ref_z_changed)
        # self.slider_ref_freq.valueChanged.connect(self.slider_ref_freq_changed)
        self.slider_ref_height.valueChanged.connect(self.slider_ref_height_changed)
        # self.slider_ref_length.valueChanged.connect(self.slider_ref_length_changed)
        self.line_ref_x.returnPressed.connect(self.line_ref_x_changed)
        self.line_ref_y.returnPressed.connect(self.line_ref_y_changed)
        self.line_ref_z.returnPressed.connect(self.line_ref_z_changed)
        # self.line_ref_freq.returnPressed.connect(self.line_ref_freq_changed)
        self.line_ref_height.returnPressed.connect(self.line_ref_height_changed)
        # self.line_ref_length.returnPressed.connect(self.line_ref_length_changed)
        # for i in range(4):
        #     self.btn_gait_lst[i].clicked.connect(partial(self.btn_gait_clicked, i))
        self.btn_stop_walking.clicked.connect(self.btn_stop_walking_clicked)

        for i in range(8):
            self.btn_ref_cute_lst[i].clicked.connect(partial(self.btn_ref_cute_clicked, i+1))

        self.box_same_kpkd.stateChanged.connect(self.box_same_kpkd_changed)

        self.btn_save.clicked.connect(self.btn_save_clicked)
        self.btn_load.clicked.connect(self.btn_load_clicked)

        # self.treeView_chosen_signals.dataChanged.connect(self.signal_chosen)
        # self.model_chosen.dataChanged.connect(self.signal_chosen)

        # self.timer = QTimer()
        # self.timer.setInterval(25)
        # self.timer.timeout.connect(self.update_plot_data)
        # self.timer.start()

        self.read_config()
        print("RoboGUI: 100%")

    def update_plot_data(self):
        self.plt.add_data(self.state_data.data)

    def signal_chosen(self):
        print("Hey")

    def box_same_kpkd_changed(self):
        if self.box_same_kpkd.isChecked() == True:
            self.same_kpkd = True
        else:
            self.same_kpkd = False

    def walking_btn_was_clicked(self):
        # self.make_walking_control_widgets()
        self.stacked_layout.setCurrentIndex(0)
        # self.cmd_data.mode = 0
        self.ros_connector.set_mode(self.ros_prms.walk_mode)
        self.label_11.setText("Walking Control")

    def leg_btn_was_clicked(self):
        # self.make_legs_control_widgets()
        self.stacked_layout.setCurrentIndex(1)
        # self.cmd_data.mode = 1
        self.ros_connector.set_mode(self.ros_prms.ef_mode)
        self.label_11.setText("Legs Control")

    def body_btn_was_clicked(self):
        # self.make_body_control_widgets()
        self.stacked_layout.setCurrentIndex(2)
        # self.cmd_data.mode = 2
        self.ros_connector.set_mode(self.ros_prms.body_mode)
        self.label_11.setText("Body Control")

    def joints_btn_was_clicked(self):
        # self.make_joints_control_widgets()
        self.stacked_layout.setCurrentIndex(3)
        # self.cmd_data.mode = 3
        # self.ros_connector.set_joints_kp([0.0]*12)
        # self.ros_connector.set_joints_kd([0.0]*12)
        self.btn_ref_joints_clicked()
        self.ros_connector.set_mode(self.ros_prms.joint_mode)
        self.label_11.setText("Joints Control")
        

    def cute_btn_was_clicked(self):
        # self.make_cute_control_widgets()
        self.stacked_layout.setCurrentIndex(4)
        # self.cmd_data.mode = 4
        self.label_11.setText("Cute Actions Mode")

    def btn_stand_up_clicked(self):
        self.btn_stand_up.setEnabled(False)

        self.btn_lay_down.setEnabled(True)
        self.slider_ref_x.setEnabled(True)
        self.line_ref_x.setEnabled(True)
        self.slider_ref_y.setEnabled(True)
        self.line_ref_y.setEnabled(True)
        self.slider_ref_z.setEnabled(True)
        self.line_ref_z.setEnabled(True)
        # self.slider_ref_freq.setEnabled(True)
        # self.line_ref_freq.setEnabled(True)
        self.slider_ref_height.setEnabled(True)
        self.line_ref_height.setEnabled(True)
        # self.slider_ref_length.setEnabled(True)
        # self.line_ref_length.setEnabled(True)
        # for i in range(4):
        #     self.btn_gait_lst[i].setEnabled(True)
        self.btn_stop_walking.setEnabled(True)

        # self.cmd_data.start = 1
        # time.sleep(0.05)
        # self.cmd_data.start = 0
        self.leg_btn.setEnabled(True)
        self.body_btn.setEnabled(True)
        self.cute_actions_btn.setEnabled(True)
        self.joints_btn.setEnabled(False)

        self.ros_connector.set_mode(self.ros_prms.walk_mode)
        self.ros_connector.set_joints_kp([0.0]*12)
        self.ros_connector.set_joints_kd([0.0]*12)

        self.ros_connector.set_action(1)

    def btn_lay_down_clicked(self):
        self.btn_lay_down.setEnabled(False)

        self.btn_stand_up.setEnabled(True)
        self.slider_ref_x.setEnabled(False)
        self.line_ref_x.setEnabled(False)
        self.slider_ref_y.setEnabled(False)
        self.line_ref_y.setEnabled(False)
        self.slider_ref_z.setEnabled(False)
        self.line_ref_z.setEnabled(False)
        # self.slider_ref_freq.setEnabled(False)
        # self.line_ref_freq.setEnabled(False)
        self.slider_ref_height.setEnabled(False)
        self.line_ref_height.setEnabled(False)
        # self.slider_ref_length.setEnabled(False)
        # self.line_ref_length.setEnabled(False)
        # for i in range(4):
        #     self.btn_gait_lst[i].setEnabled(False)
        self.btn_stop_walking.setEnabled(False)

        # self.cmd_data.start = 1
        # time.sleep(0.05)
        # self.cmd_data.start = 0
        self.leg_btn.setEnabled(False)
        self.body_btn.setEnabled(False)
        self.cute_actions_btn.setEnabled(False)
        self.joints_btn.setEnabled(True)

        self.ros_connector.set_action(2)

    def slider_ref_x_changed(self):
        ref_x_prm = self.slider_ref_x.value() / 1000
        self.line_ref_x.setText(f"{ref_x_prm}")
        self.cmd_data.robot_dx = ref_x_prm

    def slider_ref_y_changed(self):
        ref_y_prm = self.slider_ref_y.value() / 1000
        self.line_ref_y.setText(f"{ref_y_prm}")
        self.cmd_data.robot_dy = ref_y_prm

    def slider_ref_z_changed(self):
        ref_z_prm = self.slider_ref_z.value() / 1000
        self.line_ref_z.setText(f"{ref_z_prm}")
        self.cmd_data.robot_dz = ref_z_prm

    # def slider_ref_freq_changed(self):
    #     ref_freq_prm = self.slider_ref_freq.value() / 1000
    #     self.line_ref_freq.setText(f"{ref_freq_prm}")
    #     self.cmd_data.robot_step_freq = ref_freq_prm

    def slider_ref_height_changed(self):
        ref_height_prm = self.slider_ref_height.value() / 1000
        self.line_ref_height.setText(f"{ref_height_prm}")
        # self.cmd_data.robot_step_height = ref_height_prm
        self.ros_connector.set_stride_height(ref_height_prm)

    # def slider_ref_length_changed(self):
    #     ref_length_prm = self.slider_ref_length.value() / 1000
    #     self.line_ref_length.setText(f"{ref_length_prm}")
    #     self.cmd_data.robot_step_lenght = ref_length_prm
    

    def line_ref_x_changed(self):
        text = float(self.line_ref_x.text())
        if text > self.ref_robot_x_max:
            text = self.ref_robot_x_max
        elif text < self.ref_robot_x_min:
            text = self.ref_robot_x_min
        self.slider_ref_x.setValue(int(text*1000))

    def line_ref_y_changed(self):
        text = float(self.line_ref_y.text())
        if text > self.ref_robot_y_max:
            text = self.ref_robot_y_max
        elif text < self.ref_robot_y_min:
            text = self.ref_robot_y_min
        self.slider_ref_y.setValue(int(text*1000))

    def line_ref_z_changed(self):
        text = float(self.line_ref_z.text())
        if text > self.ref_robot_z_max:
            text = self.ref_robot_z_max
        elif text < self.ref_robot_z_min:
            text = self.ref_robot_z_min
        self.slider_ref_z.setValue(int(text*1000))

    # def line_ref_freq_changed(self):
    #     text = float(self.line_ref_freq.text())
    #     if text > self.ref_robot_freq_max:
    #         text = self.ref_robot_freq_max
    #     elif text < self.ref_robot_freq_min:
    #         text = self.ref_robot_freq_min
    #     self.slider_ref_freq.setValue(int(text*1000))

    # def line_ref_length_changed(self):
    #     text = float(self.line_ref_length.text())
    #     if text > self.ref_robot_length_max:
    #         text = self.ref_robot_length_max
    #     elif text < self.ref_robot_length_min:
    #         text = self.ref_robot_length_min
    #     self.slider_ref_length.setValue(int(text*1000))

    def line_ref_height_changed(self):
        text = float(self.line_ref_height.text())
        if text > self.ref_robot_height_max:
            text = self.ref_robot_height_max
        elif text < self.ref_robot_height_min:
            text = self.ref_robot_height_min
        self.slider_ref_height.setValue(int(text*1000))
        

    def btn_stop_walking_clicked(self):
        self.slider_ref_x.setValue(int(self.ref_robot_x_init*1000))
        self.slider_ref_y.setValue(int(self.ref_robot_y_init*1000))
        self.slider_ref_z.setValue(int(self.ref_robot_z_init*1000))
        # self.slider_ref_freq.setValue(int(self.ref_robot_freq_init*1000))
        self.slider_ref_height.setValue(int(self.ref_robot_height_init*1000))
        # self.slider_ref_length.setValue(int(self.ref_robot_length_init*1000))

    # def btn_gait_clicked(self, i):
    #     self.cmd_data.robot_gait_type = i

    def slider_ref_body_changed(self, i):
        self.ref_body_prm[i] = self.slider_ref_body_lst[i].value() / 1000
        self.line_ref_body_lst[i].setText(f"{self.ref_body_prm[i]}")

        self.cmd_data.body_x = self.ref_body_prm[0]
        self.cmd_data.body_y = self.ref_body_prm[1]
        self.cmd_data.body_z = self.ref_body_prm[2]
        self.cmd_data.body_wx = self.ref_body_prm[3]
        self.cmd_data.body_wy = self.ref_body_prm[4]
        self.cmd_data.body_wz = self.ref_body_prm[5]

    def line_ref_body_changed(self, i):
        text = float(self.line_ref_body_lst[i].text())
        if text > self.ref_body_maxes[i]:
            text = self.ref_body_maxes[i]
        elif text < self.ref_body_mins[i]:
            text = self.ref_body_mins[i]
        self.slider_ref_body_lst[i].setValue(int(text*1000))

    def btn_ref_body_clicked(self):
        for i in range(6):
            self.slider_ref_body_lst[i].setValue(int(self.ref_body_inits[i]*1000))

    def slider_ref_legs_changed(self, i):
        ref_joint_prm = self.slider_ref_legs_lst[i].value() / 1000
        self.line_ref_legs_lst[i].setText(f"{ref_joint_prm}")

        if i % 3 == 0:
            self.cmd_data.ef_x[int(i/3)] = ref_joint_prm
        elif i % 3 == 1:    
            self.cmd_data.ef_y[int(i/3)] = ref_joint_prm
        elif i % 3 == 2:    
            self.cmd_data.ef_z[int(i/3)] = ref_joint_prm
        
        
        # print(f"{self.cmd_data.ef_x[0]} | {self.cmd_data.ef_y[0]} | {self.cmd_data.ef_z[0]}")
        # print(f"{self.cmd_data.ef_x[1]} | {self.cmd_data.ef_y[1]} | {self.cmd_data.ef_z[1]}")
        # print(f"{self.cmd_data.ef_x[2]} | {self.cmd_data.ef_y[2]} | {self.cmd_data.ef_z[2]}")
        # print(f"{self.cmd_data.ef_x[3]} | {self.cmd_data.ef_y[3]} | {self.cmd_data.ef_z[3]}")
        # print("===========================")

    def line_ref_legs_changed(self, i):
        text = float(self.line_ref_legs_lst[i].text())
        if text > self.ref_legs_max[i]:
            text = self.ref_legs_max[i]
        elif text < self.ref_legs_min[i]:
            text = self.ref_legs_min[i]
        self.slider_ref_legs_lst[i].setValue(int(text*1000))

    def btn_ref_legs_clicked(self):
        for i in range(12):
            self.slider_ref_legs_lst[i].setValue(int(self.ref_legs_init[i]*1000))

    def slider_ref_joint_pos_changed(self, i):
        ref_joint_prm = self.slider_ref_joint_pos_lst[i].value() / 1000
        self.line_ref_joint_pos_lst[i].setText(f"{ref_joint_prm}")
        self.cmd_data.joint_pos[i] = ref_joint_prm
    
    def slider_ref_joint_vel_changed(self, i):
        ref_joint_prm = self.slider_ref_joint_vel_lst[i].value() / 1000
        self.line_ref_joint_vel_lst[i].setText(f"{ref_joint_prm}")
        self.cmd_data.joint_vel[i] = ref_joint_prm

    def slider_ref_joint_torq_changed(self, i):
        ref_joint_prm = self.slider_ref_joint_torq_lst[i].value() / 1000
        self.line_ref_joint_torq_lst[i].setText(f"{ref_joint_prm}")
        self.cmd_data.joint_torq[i] = ref_joint_prm

    def slider_ref_joint_kp_changed(self, i):
        ref_joint_prm = self.slider_ref_joint_kp_lst[i].value() / 1000
        self.line_ref_joint_kp_lst[i].setText(f"{ref_joint_prm}")
        self.joint_kp[i] = ref_joint_prm
 
        if self.same_kpkd == True:
            if i >= 11:
                self.ros_connector.set_joints_kp(self.joint_kp)
        else:
            self.ros_connector.set_joints_kp(self.joint_kp)

    def slider_ref_joint_kd_changed(self, i):
        ref_joint_prm = self.slider_ref_joint_kd_lst[i].value() / 1000
        self.line_ref_joint_kd_lst[i].setText(f"{ref_joint_prm}")
        self.joint_kd[i] = ref_joint_prm

        if self.same_kpkd == True:
            if i >= 11:
                self.ros_connector.set_joints_kd(self.joint_kd)
        else:
            self.ros_connector.set_joints_kd(self.joint_kd)
    
    def line_ref_joint_pos_changed(self, i):
        text = float(self.line_ref_joint_pos_lst[i].text())
        if text > self.ref_joint_pos_max[i]:
            text = self.ref_joint_pos_max[i]
        elif text < self.ref_joint_pos_min[i]:
            text = self.ref_joint_pos_min[i]
        self.slider_ref_joint_pos_lst[i].setValue(int(text*1000))
    
    def line_ref_joint_vel_changed(self, i):
        text = float(self.line_ref_joint_vel_lst[i].text())
        if text > self.ref_joint_vel_max[i]:
            text = self.ref_joint_vel_max[i]
        elif text < self.ref_joint_vel_min[i]:
            text = self.ref_joint_vel_min[i]
        self.slider_ref_joint_vel_lst[i].setValue(int(text*1000))
    
    def line_ref_joint_torq_changed(self, i):
        text = float(self.line_ref_joint_torq_lst[i].text())
        if text > self.ref_joint_torq_max[i]:
            text = self.ref_joint_torq_max[i]
        elif text < self.ref_joint_torq_min[i]:
            text = self.ref_joint_torq_min[i]
        self.slider_ref_joint_torq_lst[i].setValue(int(text*1000))
    
    def line_ref_joint_kp_changed(self, i):
        text = float(self.line_ref_joint_kp_lst[i].text())
        if text > self.ref_joint_kp_max[i]:
            text = self.ref_joint_kp_max[i]
        elif text < self.ref_joint_kp_min[i]:
            text = self.ref_joint_kp_min[i]
        
        if self.same_kpkd == True:
            for k in range(12):
                self.line_ref_joint_kp_lst[k].setText(f"{text}")
                self.slider_ref_joint_kp_lst[k].setValue(int(text*1000))
        else:
            self.slider_ref_joint_kp_lst[i].setValue(int(text*1000))
    
    def line_ref_joint_kd_changed(self, i):
        text = float(self.line_ref_joint_kd_lst[i].text())
        if text > self.ref_joint_kd_max[i]:
            text = self.ref_joint_kd_max[i]
        elif text < self.ref_joint_kd_min[i]:
            text = self.ref_joint_kd_min[i]
        
        if self.same_kpkd == True:
            for k in range(12):
                self.line_ref_joint_kd_lst[k].setText(f"{text}")
                self.slider_ref_joint_kd_lst[k].setValue(int(text*1000))
        else:
            self.slider_ref_joint_kd_lst[i].setValue(int(text*1000))

    def btn_ref_joints_clicked(self):
        for i in range(12):
            self.slider_ref_joint_pos_lst[i].setValue(int(self.ref_joint_pos_inits[i]*1000))
            self.slider_ref_joint_vel_lst[i].setValue(int(self.ref_joint_vel_inits[i]*1000))
            self.slider_ref_joint_torq_lst[i].setValue(int(self.ref_joint_torq_inits[i]*1000))
            self.slider_ref_joint_kp_lst[i].setValue(int(self.ref_joint_kp_inits[i]*1000))
            self.slider_ref_joint_kd_lst[i].setValue(int(self.ref_joint_kd_inits[i]*1000))

    def btn_hold_servos_clicked(self):
        for i in range(12):
            self.slider_ref_joint_pos_lst[i].setValue(int(self.state_data.data["Joint States"][f"Joint Pos {i+1}"]*1000))
            self.slider_ref_joint_vel_lst[i].setValue(int(0))
            self.slider_ref_joint_torq_lst[i].setValue(int(0))
            self.slider_ref_joint_kp_lst[i].setValue(int(12*1000))
            self.slider_ref_joint_kd_lst[i].setValue(int(0.3*1000))
            # print(self.state_data.data["Joint States"][f"Joint Pos {i+1}"])

    def btn_ref_cute_clicked(self, i):
        # self.cmd_data.cute_action = i
        # time.sleep(0.05)
        # self.cmd_data.cute_action = 0
        self.ros_connector.set_action(i)

    def btn_save_clicked(self):
        # print("yo")
        sig_lst = []
        for row in range(self.table.rowCount()):
            it = self.table.item(row, 0)
            signal = it.text() if it is not None else ""
            sig_lst.append(signal)
        
        name = QFileDialog.getSaveFileName(self, 'Save File', filter=".csv")

        if name[0] != '':
            filename = name[0] + name[1]

            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=' ',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(sig_lst)

    def btn_load_clicked(self):
        name = QFileDialog.getOpenFileName(self, 'Open File', filter="*.csv")
        filename = name[0]

        if filename != '':
            with open(filename, 'r', newline='') as csvfile:
                spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
                # print(spamreader)
                for row in spamreader:
                    # print(row)
                    self.table_list = row
                    # print(self.table_list)

            # print(self.table_list)
            while self.table.rowCount() > 0:
                self.table.removeRow(self.table.rowCount()-1)
            for i in range(len(self.table_list)):
                self.table.insertRow(i)
                self.table.setItem(i, 0, QTableWidgetItem(self.table_list[i]))
                self.table.setItem(i, 1, QTableWidgetItem("0.000"))
                # print(self.table_list[i])

    
    def refresh_widgets(self):
        while True:
            if self.kill_threads == False:
                self.ros_connector.set_data(self.cmd_data)
                
                self.state_data, self.cur_device = self.ros_connector.get_data()
                # print(self.cur_device)

                if self.cur_device == 1:
                    self.label_control_device.setText("Dualshock 4")
                    self.set_enabled_widgets(False, True)
                elif self.cur_device == 2:
                    self.label_control_device.setText("Radiolink")
                    self.set_enabled_widgets(False, True)
                elif self.cur_device == 3:
                    self.label_control_device.setText("Navigation")
                    self.set_enabled_widgets(False, True)
                elif self.cur_device == 4:
                    self.label_control_device.setText("PC")
                    self.set_enabled_widgets(True, False)
                else:
                    self.label_control_device.setText("Unknown")
                    self.set_enabled_widgets(False, True)
                # time.sleep(0.025)
                col = 0
                # data = []
                for row in range(self.table.rowCount()):
                    it = self.table.item(row, col)
                    try:
                        signal = it.text() if it is not None else ""
                    except:
                        break
                    
                    if signal in self.state_data.data["Desired Values"]:
                        value = self.state_data.data["Desired Values"][signal]
                    elif signal in self.state_data.data["Joint States"]:
                        value = self.state_data.data["Joint States"][signal]
                    elif signal in self.state_data.data["Body States"]:
                        value = self.state_data.data["Body States"][signal]
                    elif signal in self.state_data.data["Commands to HL from LL"]:
                        value = self.state_data.data["Commands to HL from LL"][signal]
                    elif signal in self.state_data.data["Foot Positions"]:
                        value = self.state_data.data["Foot Positions"][signal]
                    
                    cell = self.table.item(row, 1)
                    try:
                        cell.setText(f"{value:.3f}")
                    except:
                        break
                
                # self.plt.add_data(self.state_data.data)
                    # data.append(text)
                # print(data)
                # print("Hey")
    def set_enabled_widgets(self, enable : bool, not_pc=False):
        self.walking_widget.setEnabled(enable)
        self.legs_widget.setEnabled(enable)
        self.body_widget.setEnabled(enable)
        self.joints_widget.setEnabled(enable)
        self.cute_widget.setEnabled(enable)
        self.walking_btn.setEnabled(enable)

        if not_pc == True:
            self.leg_btn.setEnabled(enable)
            self.body_btn.setEnabled(enable)
            self.joints_btn.setEnabled(enable)
            self.cute_actions_btn.setEnabled(enable)

    def create_walking_control_widgets(self):
        
        self.leg_btn.setEnabled(False)
        self.body_btn.setEnabled(False)
        self.cute_actions_btn.setEnabled(False)
        self.joints_btn.setEnabled(True)
        self.label_11.setText("Walking Control")

        line_size = 110

        self.btn_stand_up = QPushButton("Stand up")
        self.btn_lay_down = QPushButton("Lay down")
        self.btn_lay_down.setEnabled(False)
        layout_stand = QHBoxLayout()
        layout_stand.addWidget(self.btn_stand_up)
        layout_stand.addWidget(self.btn_lay_down)
        
        lbl_ref_x = QLabel("X vel [m/s]")
        self.slider_ref_x = QSlider(Qt.Horizontal)
        self.slider_ref_x.setMinimum(int(self.ref_robot_x_min*1000))
        self.slider_ref_x.setMaximum(int(self.ref_robot_x_max*1000))
        self.slider_ref_x.setValue(int(self.ref_robot_x_init*1000))
        self.line_ref_x = QLineEdit()
        self.line_ref_x.setMaximumWidth(line_size)
        self.line_ref_x.setText(f"{self.ref_robot_x_init}")
        self.slider_ref_x.setEnabled(False)
        self.line_ref_x.setEnabled(False)

        layout_ref_x = QHBoxLayout()
        layout_ref_x.addWidget(lbl_ref_x)
        layout_ref_x.addWidget(self.slider_ref_x)
        layout_ref_x.addWidget(self.line_ref_x)

        lbl_ref_y = QLabel("Y vel [m/s]")
        self.slider_ref_y = QSlider(Qt.Horizontal)
        self.slider_ref_y.setMinimum(int(self.ref_robot_y_min*1000))
        self.slider_ref_y.setMaximum(int(self.ref_robot_y_max*1000))
        self.slider_ref_y.setValue(int(self.ref_robot_y_init*1000))
        self.line_ref_y = QLineEdit()
        self.line_ref_y.setMaximumWidth(line_size)
        self.line_ref_y.setText(f"{self.ref_robot_y_init}")
        self.slider_ref_y.setEnabled(False)
        self.line_ref_y.setEnabled(False)

        layout_ref_y = QHBoxLayout()
        layout_ref_y.addWidget(lbl_ref_y)
        layout_ref_y.addWidget(self.slider_ref_y)
        layout_ref_y.addWidget(self.line_ref_y)

        lbl_ref_z = QLabel("Z vel [m/s]")
        self.slider_ref_z = QSlider(Qt.Horizontal)
        self.slider_ref_z.setMinimum(int(self.ref_robot_z_min*1000))
        self.slider_ref_z.setMaximum(int(self.ref_robot_z_max*1000))
        self.slider_ref_z.setValue(int(self.ref_robot_z_init*1000))
        self.line_ref_z = QLineEdit()
        self.line_ref_z.setMaximumWidth(line_size)
        self.line_ref_z.setText(f"{self.ref_robot_z_init}")
        self.slider_ref_z.setEnabled(False)
        self.line_ref_z.setEnabled(False)

        layout_ref_z = QHBoxLayout()
        layout_ref_z.addWidget(lbl_ref_z)
        layout_ref_z.addWidget(self.slider_ref_z)
        layout_ref_z.addWidget(self.line_ref_z)

        # lbl_ref_freq = QLabel("Stride freq")
        # self.slider_ref_freq = QSlider(Qt.Horizontal)
        # self.slider_ref_freq.setMinimum(int(self.ref_robot_freq_min*1000))
        # self.slider_ref_freq.setMaximum(int(self.ref_robot_freq_max*1000))
        # self.slider_ref_freq.setValue(int(self.ref_robot_freq_init*1000))
        # self.line_ref_freq = QLineEdit()
        # self.line_ref_freq.setMaximumWidth(line_size)
        # self.line_ref_freq.setText(f"{self.ref_robot_freq_init}")
        # self.slider_ref_freq.setEnabled(False)
        # self.line_ref_freq.setEnabled(False)

        # layout_ref_freq = QHBoxLayout()
        # layout_ref_freq.addWidget(lbl_ref_freq)
        # layout_ref_freq.addWidget(self.slider_ref_freq)
        # layout_ref_freq.addWidget(self.line_ref_freq)

        lbl_ref_height = QLabel("Stride height [m]")
        self.slider_ref_height = QSlider(Qt.Horizontal)
        self.slider_ref_height.setMinimum(int(self.ref_robot_height_min*1000))
        self.slider_ref_height.setMaximum(int(self.ref_robot_height_max*1000))
        self.slider_ref_height.setValue(int(self.ref_robot_height_init*1000))
        self.line_ref_height = QLineEdit()
        self.line_ref_height.setText(f"{self.ref_robot_height_init}")
        self.line_ref_height.setMaximumWidth(line_size)
        self.slider_ref_height.setEnabled(False)
        self.line_ref_height.setEnabled(False)

        layout_ref_height = QHBoxLayout()
        layout_ref_height.addWidget(lbl_ref_height)
        layout_ref_height.addWidget(self.slider_ref_height)
        layout_ref_height.addWidget(self.line_ref_height)

        # lbl_ref_length = QLabel("Stride width")
        # self.slider_ref_length = QSlider(Qt.Horizontal)
        # self.slider_ref_length.setMinimum(int(self.ref_robot_length_min*1000))
        # self.slider_ref_length.setMaximum(int(self.ref_robot_length_max*1000))
        # self.slider_ref_length.setValue(int(self.ref_robot_length_init*1000))
        # self.line_ref_length = QLineEdit()
        # self.line_ref_length.setMaximumWidth(line_size)
        # self.line_ref_length.setText(f"{self.ref_robot_length_init}")
        # self.slider_ref_length.setEnabled(False)
        # self.line_ref_length.setEnabled(False)

        # layout_ref_length = QHBoxLayout()
        # layout_ref_length.addWidget(lbl_ref_length)
        # layout_ref_length.addWidget(self.slider_ref_length)
        # layout_ref_length.addWidget(self.line_ref_length)

        # lbl_ref_gait = QLabel("Gait type")
        # layout_ref_gait = QHBoxLayout()
        # layout_ref_gait.addWidget(lbl_ref_gait)
        # self.btn_gait_lst = []
        # gait_types = ["Walk", "Trot", "Pace", "Gallop"]
        # for i in range(4):
        #     self.btn_gait_lst.append(QPushButton(f"{gait_types[i]}"))
        #     layout_ref_gait.addWidget(self.btn_gait_lst[i])
        #     self.btn_gait_lst[i].setEnabled(False)

        self.btn_stop_walking = QPushButton("Stop Walking")
        self.btn_stop_walking.setEnabled(False)

        verticalSpacer = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding) 

        walking_layout = QVBoxLayout()
        walking_layout.addLayout(layout_stand)
        walking_layout.addLayout(layout_ref_height)
        walking_layout.addLayout(layout_ref_x)
        walking_layout.addLayout(layout_ref_y)
        walking_layout.addLayout(layout_ref_z)
        # walking_layout.addLayout(layout_ref_freq)
        # walking_layout.addLayout(layout_ref_length)
        # walking_layout.addLayout(layout_ref_gait)
        walking_layout.addWidget(self.btn_stop_walking)
        walking_layout.addItem(verticalSpacer)

        # self.walking_widget = QWidget()
        self.walking_widget.setLayout(walking_layout)
        walking_scroll = QScrollArea()
        walking_scroll.setWidgetResizable(True)
        walking_scroll.setWidget(self.walking_widget)
        self.stacked_layout.addWidget(walking_scroll)

        print("RoboGUI: 15%")

    def create_legs_control_widgets(self):
        
        lbl_ref_legs_lst = []
        self.slider_ref_legs_lst = []
        self.line_ref_legs_lst = []
        layout_ref_legs_lst = []
        lbl_names = ["R1 X [rad]", "R1 Y [rad]", "R1 Z [rad]", 
                     "L1 X [rad]", "L1 Y [rad]", "L1 Z [rad]", 
                     "R2 X [rad]", "R2 Y [rad]", "R2 Z [rad]", 
                     "L2 X [rad]", "L2 Y [rad]", "L2 Z [rad]"]
        scroll_area_layout = QVBoxLayout()

        for i in range(12):
            lbl_ref_legs_lst.append(QLabel(lbl_names[i]))
            self.slider_ref_legs_lst.append(QSlider(Qt.Horizontal))
            self.slider_ref_legs_lst[i].setMinimum(int(self.ref_legs_min[i]*1000))
            self.slider_ref_legs_lst[i].setMaximum(int(self.ref_legs_max[i]*1000))
            self.slider_ref_legs_lst[i].setValue(int(self.ref_legs_init[i]*1000))
            self.line_ref_legs_lst.append(QLineEdit())
            self.line_ref_legs_lst[i].setMaximumWidth(110)
            self.line_ref_legs_lst[i].setText(f"{self.ref_legs_init[i]}")

            layout_ref_legs_lst.append(QHBoxLayout())
            layout_ref_legs_lst[i].addWidget(lbl_ref_legs_lst[i])
            layout_ref_legs_lst[i].addWidget(self.slider_ref_legs_lst[i])
            layout_ref_legs_lst[i].addWidget(self.line_ref_legs_lst[i])

            scroll_area_layout.addLayout(layout_ref_legs_lst[i])

            if i == 2 or i == 5 or i == 8:
                scroll_area_layout.addWidget(QHLine())

        self.btn_ref_legs = QPushButton("Set to Zero")
        scroll_area_layout.addWidget(self.btn_ref_legs)
        
        verticalSpacer = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding) 
        scroll_area_layout.addItem(verticalSpacer)

        # w = QWidget()
        self.legs_widget.setLayout(scroll_area_layout)
        body_scroll = QScrollArea()
        body_scroll.setWidgetResizable(True)
        body_scroll.setWidget(self.legs_widget)
        self.stacked_layout.addWidget(body_scroll)

        for i in range(1, 12):
            self.setTabOrder(self.line_ref_legs_lst[i-1], self.line_ref_legs_lst[i])
            self.setTabOrder(self.slider_ref_legs_lst[i-1], self.slider_ref_legs_lst[i])
        print("RoboGUI: 30%")

    def create_body_control_widgets(self):
        
        lbl_ref_body_lst = []
        self.slider_ref_body_lst = []
        self.line_ref_body_lst = []
        layout_ref_body_lst = []
        lbl_names = ["X pos [m]", "Y pos [m]", "Z pos [m]", 
                     "X ang [rad]", "Y ang [rad]", "Z ang [rad]"]
        
 
        scroll_area_layout = QVBoxLayout()

        for i in range(6):
            lbl_ref_body_lst.append(QLabel(lbl_names[i]))
            self.slider_ref_body_lst.append(QSlider(Qt.Horizontal))
            self.slider_ref_body_lst[i].setMinimum(int(self.ref_body_mins[i]*1000))
            self.slider_ref_body_lst[i].setMaximum(int(self.ref_body_maxes[i]*1000))
            self.slider_ref_body_lst[i].setValue(int(self.ref_body_inits[i]*1000))
            self.line_ref_body_lst.append(QLineEdit())
            self.line_ref_body_lst[i].setMaximumWidth(100)
            self.line_ref_body_lst[i].setText(f"{self.ref_body_inits[i]}")

            layout_ref_body_lst.append(QHBoxLayout())
            layout_ref_body_lst[i].addWidget(lbl_ref_body_lst[i])
            layout_ref_body_lst[i].addWidget(self.slider_ref_body_lst[i])
            layout_ref_body_lst[i].addWidget(self.line_ref_body_lst[i])

            scroll_area_layout.addLayout(layout_ref_body_lst[i])

            if i == 2:
                scroll_area_layout.addWidget(QHLine())

        self.btn_ref_body = QPushButton("Set to Zero")
        scroll_area_layout.addWidget(self.btn_ref_body)
        
        verticalSpacer = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding) 
        scroll_area_layout.addItem(verticalSpacer)

        # w = QWidget()
        self.body_widget.setLayout(scroll_area_layout)
        body_scroll = QScrollArea()
        body_scroll.setWidgetResizable(True)
        body_scroll.setWidget(self.body_widget)
        self.stacked_layout.addWidget(body_scroll)

        for i in range(1, 12):
            if i == 6:
                self.setTabOrder(self.line_ref_body_lst[i-1], self.slider_ref_body_lst[i-6])
            elif i < 6:
                self.setTabOrder(self.line_ref_body_lst[i-1], self.line_ref_body_lst[i])
            else:
                self.setTabOrder(self.slider_ref_body_lst[i-7], self.slider_ref_body_lst[i-6])
        print("RoboGUI: 45%")

    def create_joints_control_widgets(self):
        
        lbl_ref_pos_lst = []
        self.slider_ref_joint_pos_lst = []
        self.line_ref_joint_pos_lst = []
        layout_ref_pos_lst = []

        lbl_ref_vel_lst = []
        self.slider_ref_joint_vel_lst = []
        self.line_ref_joint_vel_lst = []
        layout_ref_vel_lst = []

        lbl_ref_joint_torq_lst = []
        self.slider_ref_joint_torq_lst = []
        self.line_ref_joint_torq_lst = []
        layout_ref_torq_lst = []

        lbl_ref_kp_lst = []
        self.slider_ref_joint_kp_lst = []
        self.line_ref_joint_kp_lst = []
        layout_ref_kp_lst = []

        lbl_ref_kd_lst = []
        self.slider_ref_joint_kd_lst = []
        self.line_ref_joint_kd_lst = []
        layout_ref_kd_lst = []

        scroll_area_layout = QVBoxLayout()

        lbl_min_width = 50
        line_min_width = 110

        # self.btn_hold_servos = QPushButton("Hold cur pos")
        # scroll_area_layout.addWidget(self.btn_hold_servos)

        self.box_same_kpkd = QCheckBox("Same KpKd")
        self.box_same_kpkd.setChecked(False)
        scroll_area_layout.addWidget(self.box_same_kpkd)
        

        for i in range(12):
            # Joint positions
            lbl_ref_pos_lst.append(QLabel("Joint {0} pos [rad]".format(i+1)))
            lbl_ref_pos_lst[i].setMinimumWidth(lbl_min_width)
            self.slider_ref_joint_pos_lst.append(QSlider(Qt.Horizontal))
            self.slider_ref_joint_pos_lst[i].setMinimum(int(self.ref_joint_pos_min[i]*1000))
            self.slider_ref_joint_pos_lst[i].setMaximum(int(self.ref_joint_pos_max[i]*1000))
            self.slider_ref_joint_pos_lst[i].setValue(int(self.ref_joint_pos_inits[i]*1000))
            self.line_ref_joint_pos_lst.append(QLineEdit())
            self.line_ref_joint_pos_lst[i].setMaximumWidth(line_min_width)
            self.line_ref_joint_pos_lst[i].setText(f"{self.ref_joint_pos_inits[i]}")


            layout_ref_pos_lst.append(QHBoxLayout())
            layout_ref_pos_lst[i].addWidget(lbl_ref_pos_lst[i])
            layout_ref_pos_lst[i].addWidget(self.slider_ref_joint_pos_lst[i])
            layout_ref_pos_lst[i].addWidget(self.line_ref_joint_pos_lst[i])

            # joint velocities
            lbl_ref_vel_lst.append(QLabel("Joint {0} vel [rad/s]".format(i+1)))
            lbl_ref_vel_lst[i].setMinimumWidth(lbl_min_width)
            self.slider_ref_joint_vel_lst.append(QSlider(Qt.Horizontal))
            self.slider_ref_joint_vel_lst[i].setMinimum(int(self.ref_joint_vel_min[i]*1000))
            self.slider_ref_joint_vel_lst[i].setMaximum(int(self.ref_joint_vel_max[i]*1000))
            self.slider_ref_joint_vel_lst[i].setValue(int(self.ref_joint_vel_inits[i]*1000))
            self.line_ref_joint_vel_lst.append(QLineEdit())
            self.line_ref_joint_vel_lst[i].setMaximumWidth(line_min_width)
            self.line_ref_joint_vel_lst[i].setText(f"{self.ref_joint_vel_inits[i]}")

            layout_ref_vel_lst.append(QHBoxLayout())
            layout_ref_vel_lst[i].addWidget(lbl_ref_vel_lst[i])
            layout_ref_vel_lst[i].addWidget(self.slider_ref_joint_vel_lst[i])
            layout_ref_vel_lst[i].addWidget(self.line_ref_joint_vel_lst[i])

            # joint torques
            lbl_ref_joint_torq_lst.append(QLabel("Joint {0} torq [Nm]".format(i+1)))
            lbl_ref_joint_torq_lst[i].setMinimumWidth(lbl_min_width)
            self.slider_ref_joint_torq_lst.append(QSlider(Qt.Horizontal))
            self.slider_ref_joint_torq_lst[i].setMinimum(int(self.ref_joint_torq_min[i]*1000))
            self.slider_ref_joint_torq_lst[i].setMaximum(int(self.ref_joint_torq_max[i]*1000))
            self.slider_ref_joint_torq_lst[i].setValue(int(self.ref_joint_torq_inits[i]*1000))
            self.line_ref_joint_torq_lst.append(QLineEdit())
            self.line_ref_joint_torq_lst[i].setMaximumWidth(line_min_width)
            self.line_ref_joint_torq_lst[i].setText(f"{self.ref_joint_torq_inits[i]}")

            layout_ref_torq_lst.append(QHBoxLayout())
            layout_ref_torq_lst[i].addWidget(lbl_ref_joint_torq_lst[i])
            layout_ref_torq_lst[i].addWidget(self.slider_ref_joint_torq_lst[i])
            layout_ref_torq_lst[i].addWidget(self.line_ref_joint_torq_lst[i])

            # joint kp
            lbl_ref_kp_lst.append(QLabel("Joint {0} kp".format(i+1)))
            lbl_ref_kp_lst[i].setMinimumWidth(lbl_min_width)
            self.slider_ref_joint_kp_lst.append(QSlider(Qt.Horizontal))
            self.slider_ref_joint_kp_lst[i].setMinimum(int(self.ref_joint_kp_min[i]*1000))
            self.slider_ref_joint_kp_lst[i].setMaximum(int(self.ref_joint_kp_max[i]*1000))
            self.slider_ref_joint_kp_lst[i].setValue(int(self.ref_joint_kp_inits[i]*1000))
            self.line_ref_joint_kp_lst.append(QLineEdit())
            self.line_ref_joint_kp_lst[i].setMaximumWidth(line_min_width)
            self.line_ref_joint_kp_lst[i].setText(f"{self.ref_joint_kp_inits[i]}")

            layout_ref_kp_lst.append(QHBoxLayout())
            layout_ref_kp_lst[i].addWidget(lbl_ref_kp_lst[i])
            layout_ref_kp_lst[i].addWidget(self.slider_ref_joint_kp_lst[i])
            layout_ref_kp_lst[i].addWidget(self.line_ref_joint_kp_lst[i])

            # joint kd
            lbl_ref_kd_lst.append(QLabel("Joint {0} kd".format(i+1)))
            lbl_ref_kd_lst[i].setMinimumWidth(lbl_min_width)
            self.slider_ref_joint_kd_lst.append(QSlider(Qt.Horizontal))
            self.slider_ref_joint_kd_lst[i].setMinimum(int(self.ref_joint_kd_min[i]*1000))
            self.slider_ref_joint_kd_lst[i].setMaximum(int(self.ref_joint_kd_max[i]*1000))
            self.slider_ref_joint_kd_lst[i].setValue(int(self.ref_joint_kd_inits[i]*1000))
            self.line_ref_joint_kd_lst.append(QLineEdit())
            self.line_ref_joint_kd_lst[i].setMaximumWidth(line_min_width)
            self.line_ref_joint_kd_lst[i].setText(f"{self.ref_joint_kd_inits[i]}")

            layout_ref_kd_lst.append(QHBoxLayout())
            layout_ref_kd_lst[i].addWidget(lbl_ref_kd_lst[i])
            layout_ref_kd_lst[i].addWidget(self.slider_ref_joint_kd_lst[i])
            layout_ref_kd_lst[i].addWidget(self.line_ref_joint_kd_lst[i])

            # add all horizontal layouts to one vertical
            scroll_area_layout.addLayout(layout_ref_pos_lst[i])
            scroll_area_layout.addLayout(layout_ref_vel_lst[i])
            scroll_area_layout.addLayout(layout_ref_torq_lst[i])
            scroll_area_layout.addLayout(layout_ref_kp_lst[i])
            scroll_area_layout.addLayout(layout_ref_kd_lst[i])
            scroll_area_layout.addWidget(QHLine())

        self.btn_ref_joints = QPushButton("Set to Zero")
        scroll_area_layout.addWidget(self.btn_ref_joints)

        verticalSpacer = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding) 
        scroll_area_layout.addItem(verticalSpacer)

        # w = QWidget()
        self.joints_widget.setLayout(scroll_area_layout)
        joints_scroll = QScrollArea()
        joints_scroll.setWidgetResizable(True)
        joints_scroll.setWidget(self.joints_widget)
        self.stacked_layout.addWidget(joints_scroll)

        print("RoboGUI: 60%")
    
    def create_cute_control_widgets(self):
        
        self.btn_ref_cute_lst = []
        scroll_area_layout = QVBoxLayout()
        cute_actions_lst = ["Stand Up", "Lay Down", "Give Right Paw", "Side Roll", "Wave Paw", "Sit", "Give Left Paw", "Action 8"]

        for i in range(len(cute_actions_lst)):
            self.btn_ref_cute_lst.append(QPushButton(cute_actions_lst[i]))
            self.btn_ref_cute_lst[i].setMaximumWidth(320)

            scroll_area_layout.addWidget(self.btn_ref_cute_lst[i])
        
        verticalSpacer = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding) 
        scroll_area_layout.addItem(verticalSpacer)

        # w = QWidget()
        self.cute_widget.setLayout(scroll_area_layout)
        body_scroll = QScrollArea()
        body_scroll.setWidgetResizable(True)
        body_scroll.setWidget(self.cute_widget)
        self.stacked_layout.addWidget(body_scroll)

        print("RoboGUI: 75%")

    def create_all_signals_tree(self):
        self.tree = MyTreeWidget()
        self.verticalLayout_2.addWidget(self.tree)

        self.tree.setColumnCount(1)
        self.tree.setHeaderLabels([' '])
        parents_cnt = 0

        # states = StateData()
        # parents_cnt = 0
        # for i in range(len(states.data)):
        #     if states.data[i][0] == parents_cnt:
        #         parent = QTreeWidgetItem(self.tree)
        #         parent.setText(0, f'{states.data[i][1]}')
        #         parents_cnt += 1
        #     child = QTreeWidgetItem(parent)
        #     child.setText(0, f'{states.data[i][2]}')

        # self.states = StateDataDict()
        self.tree.fillItems(self.tree, self.state_data.data)


    def create_table(self):
        self.table = MyTableWidget()
        self.table.setColumnCount(2)
        self.table.setColumnWidth(0, 150)
        self.table.setColumnWidth(1, 100)
        self.table.setHorizontalHeaderLabels(['Name', 'Value'])
        self.verticalLayout_4.addWidget(self.table)

        self.btn_load = QPushButton("Load")
        self.btn_save = QPushButton("Save")
        self.btn_load.setAutoDefault(False)
        self.btn_save.setAutoDefault(False)
        hor_lay = QHBoxLayout()
        hor_lay.addWidget(self.btn_load)
        hor_lay.addWidget(self.btn_save)
        self.verticalLayout_4.addLayout(hor_lay)

    def create_plot(self):

        # self.plt = Plot2D()
        # self.plt.set_dt(0.025)
        # page_lay = QHBoxLayout()
        # page_lay.addWidget(self.plt)

        # # self.tab_1.setLayout(page_lay)
        # w = QWidget()
        # self.tabWidget_plots.addTab(self.plt, "Plot1")
        # self.tabWidget_plots.addTab(w, "+")

        self.tabWidget_plots.setVisible(False)

    def read_config(self):
        # pass
        path = "config.ini"
        if not os.path.exists(path):
            self.write_config()
        
        config = configparser.ConfigParser()
        config.read(path)

        win_width = int(config.get("Window", "width"))
        win_height = int(config.get("Window", "height")) - 74
        win_pos_x = int(config.get("Window", "pos_x"))
        win_pos_y = int(config.get("Window", "pos_y")) + 74

        if config.has_option("Window", "column_w1"):
            columnW1 = int(config.get("Window", "column_w1"))
        else:
            columnW1 = 150
        if config.has_option("Window", "column_w2"):
            columnW2 = int(config.get("Window", "column_w2"))
        else:
            columnW2 = 100

        sig_lst = []
        for key in config["Table"]:
            sig_lst.append(config["Table"][key])
        
        # print(f"height: {win_height}")
        self.setGeometry(win_pos_x, win_pos_y, win_width, win_height)
        self.table.setColumnWidth(0, columnW1)
        self.table.setColumnWidth(1, columnW2)

        while self.table.rowCount() > 0:
            self.table.removeRow(self.table.rowCount()-1)
        for i in range(len(sig_lst)):
            self.table.insertRow(i)
            self.table.setItem(i, 0, QTableWidgetItem(sig_lst[i]))
            self.table.setItem(i, 1, QTableWidgetItem("0.000"))

    def write_config(self):
        win_width = self.frameGeometry().width()
        win_height = self.frameGeometry().height()
        win_pos = self.pos()

        sig_lst = []
        for row in range(self.table.rowCount()):
            it = self.table.item(row, 0)
            signal = it.text() if it is not None else ""
            sig_lst.append(signal)

        columnW1 = self.table.columnWidth(0)
        columnW2 = self.table.columnWidth(1)
        # print(win_width)
        # print(f"height: {win_height}")
        # print(win_pos.x())
        # print(win_pos.y())
        # print(sig_lst)

        config = configparser.ConfigParser()
        config.add_section("Window")
        config.set("Window", "width", str(win_width))
        config.set("Window", "height", str(win_height))
        config.set("Window", "pos_x", str(win_pos.x()))
        config.set("Window", "pos_y", str(win_pos.y()))
        config.set("Window", "column_w1", str(columnW1))
        config.set("Window", "column_w2", str(columnW2))

        config.add_section("Table")
        for i in range(len(sig_lst)):
            config.set("Table", f"{i}", str(sig_lst[i]))
        # config.set("Settings", "test_lst", str(lst))

        with open("config.ini", "w") as config_file:
            config.write(config_file)

    def closeEvent(self, event):
        
        self.kill_threads = True

        self.write_config()

        directory = os.getcwd()
        print(f"RoboGUI: Config saved to directory {directory}")
        print("RoboGUI: closed")


app = QtWidgets.QApplication(sys.argv)

window = MainWindow()
window.show()
app.exec()
