from PyQt5 import QtWidgets
# from PyQt5 import QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
# import os
# from PyQt5 import QtCore
from random import randint
from functools import partial

import numpy as np
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from robogui.simple_widgets import QHLine, QVLine

class MyPlotWidget(pg.PlotWidget):

    sigMouseClicked = pyqtSignal(object)
    sigDataDropped = pyqtSignal(int, str)

    def __init__(self, id=0, *args, **kwargs):
        super(MyPlotWidget, self).__init__(*args, **kwargs)
        self.setAcceptDrops(True)
        self.id = id

    # Events

    def mousePressEvent(self, ev):
        super().mousePressEvent(ev)
        if ev.button() == Qt.LeftButton:
            self.sigMouseClicked.emit(ev)

    def dragEnterEvent(self, ev):
        super().dragEnterEvent(ev)
        if ev.mouseButtons() == Qt.LeftButton:
            print("enter")
            ev.accept()
    
    def dragMoveEvent(self, e):
        pass
    
    def dropEvent(self, event : QDropEvent):
        dropped_item = event.source().currentItem().text()
        # print(f"id: {self.id}; dropped: {dropped_item}")
        self.sigDataDropped.emit(self.id, dropped_item)

    def get_id(self):
        return self.id
        


class Plot2D(QtWidgets.QWidget):
    def __init__(self, *args, **kwargs):
        super(Plot2D, self).__init__(*args, **kwargs)

        self.data_line = [[]]
        self.colors = [(0, 0, 0), (0,0,255), (3, 166, 3), (255,0,0), (217, 117, 0), (148, 7, 219), (204, 4, 164), (255, 255, 0), (255, 0, 255), (0,255,255), (5, 176, 99), (3, 150, 146)]

        self.initUI()
        self.data_dict = {}
        self.disp_data = {}
        self.disp_data_lst = []
        self.time_data = []
        self.it = 0
        self.dt = 0.01
        self.ptr = 0
        self.refresh_time = 0.02
        self.plot_display_time = 2
        self.stored_data = self.plot_display_time/self.dt

        self.timer = QTimer()
        self.timer.setInterval(int(self.refresh_time*1000))
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def new_plot(self, id):
        self.plt.append(MyPlotWidget(id=id))
        self.plt[id].addLegend()
        self.plt[id].setBackground('w')
        styles = {'color':'b', 'font-size':'10pt'}
        # self.plt.setLabel('left', 'Temperature (°C)', **styles)
        self.plt[id].setLabel('bottom', f'Time', units="s", **styles)
        self.plt[id].showGrid(x=True, y=True)

        self.plt[id].sigMouseClicked.connect(partial(self.plt_clicked, id))
        self.plt[id].sigDataDropped.connect(self.plt_data_dropped)

        for i in range(len(self.plt)-1):
            self.plt[i].setLabel('bottom', f'', units="", **styles)
            # self.plt[i].hideAxis('bottom')

    def del_plot(self, id):
        del self.plt[id]

    def initUI(self):
        # init main widgets
        self.main_lay = QVBoxLayout()

        # init plot
        self.plot_lay = QVBoxLayout()
        self.cur_id = 0
        self.new_id = 0
        self.plt = []
        self.vsplitter = []
        self.hsplitter = []
        self.new_plot(self.cur_id)
        self.vsplitter = QSplitter(Qt.Vertical)
        self.plot_lay.addWidget(self.vsplitter)
        self.vsplitter.addWidget(self.plt[0])

        # upper buttons
        buttons_lay = QHBoxLayout()
        self.btn_new_plt = QPushButton("+")
        self.btn_new_plt.setAutoDefault(False)
        buttons_lay.addWidget(self.btn_new_plt)
        self.btn_close_plt = QPushButton("-")
        self.btn_close_plt.setAutoDefault(False)
        buttons_lay.addWidget(self.btn_close_plt)
        hSpacer = QSpacerItem(20, 40, QSizePolicy.Expanding, QSizePolicy.Minimum)
        buttons_lay.addItem(hSpacer)

        self.btn_rec = QPushButton("Rec")
        self.btn_rec.setCheckable(True)
        self.btn_rec.setAutoDefault(False)
        buttons_lay.addWidget(self.btn_rec)
        self.btn_pause = QPushButton("Pause")
        self.btn_pause.setAutoDefault(False)
        buttons_lay.addWidget(self.btn_pause)
        self.btn_save = QPushButton("Save")
        self.btn_save.setAutoDefault(False)
        buttons_lay.addWidget(self.btn_save)

        # lower control elements
        # vertical scale
        self.vert_ctrl_lay = QVBoxLayout()
        lbl_vert_scale = QLabel("Vertical")
        self.vert_ctrl_lay.addWidget(lbl_vert_scale)

        self.dial_y_scale = QDial()
        self.dial_y_scale.setRange(1, 100)
        self.dial_y_scale.setSingleStep(1)
        self.dial_y_scale.setFixedSize(170, 170)
        self.vert_ctrl_lay.addWidget(self.dial_y_scale)

        self.btn_y_auto = QPushButton("Auto")
        self.btn_y_auto.setAutoDefault(False)
        self.vert_ctrl_lay.addWidget(self.btn_y_auto)
        self.vert_ctrl_lay.setAlignment(lbl_vert_scale, Qt.AlignHCenter)
        self.vert_ctrl_lay.setAlignment(self.dial_y_scale, Qt.AlignHCenter)

        # horizontal scale
        self.hor_ctrl_lay = QVBoxLayout()
        
        lbl_hor_scale = QLabel("Horizontal")
        self.hor_ctrl_lay.addWidget(lbl_hor_scale)

        self.dial_x_scale = QDial()
        self.dial_x_scale.setRange(1, 100)
        self.dial_x_scale.setSingleStep(1)
        self.dial_x_scale.setFixedSize(170, 170)
        self.hor_ctrl_lay.addWidget(self.dial_x_scale)

        self.btn_x_auto = QPushButton("Auto")
        self.btn_x_auto.setAutoDefault(False)
        self.hor_ctrl_lay.addWidget(self.btn_x_auto)
        self.hor_ctrl_lay.setAlignment(lbl_hor_scale, Qt.AlignHCenter)
        self.hor_ctrl_lay.setAlignment(self.dial_x_scale, Qt.AlignHCenter)

        # signal control
        group_x_scale = QScrollArea()
        group_x_scale.setMaximumHeight(280)
        group_x_lay = QHBoxLayout()

        lbl_x_sig = []
        slider_x_level = []
        dial_x_scale = []
        btn_x_default = []

        for i in range(12):
            lbl_x_sig.append(QLabel(f"Signal{i+1}"))
            lbl_x_sig[i].setVisible(False)
            slider_x_level.append(QSlider(Qt.Vertical))
            slider_x_level[i].setVisible(False)
            dial_x_scale.append(QDial())
            dial_x_scale[i].setVisible(False)
            btn_x_default.append(QPushButton("Default"))
            btn_x_default[i].setAutoDefault(False)
            btn_x_default[i].setVisible(False)

            sig_ctr_lay = QHBoxLayout()
            sig_ctr_lay.addWidget(slider_x_level[i])
            sig_ctr_lay.addWidget(dial_x_scale[i])

            sig_lay = QVBoxLayout()
            sig_lay.addWidget(lbl_x_sig[i])
            sig_lay.addLayout(sig_ctr_lay)
            sig_lay.addWidget(btn_x_default[i])

            group_x_lay.addLayout(sig_lay)
        
        group_x_lay.addItem(hSpacer)
        w = QWidget()
        w.setLayout(group_x_lay)

        group_x_scale.setWidget(w)

        # make all widgets together
        scale_lay = QHBoxLayout()
        scale_lay.addLayout(self.vert_ctrl_lay)
        scale_lay.addWidget(QVLine())
        scale_lay.addLayout(self.hor_ctrl_lay)
        scale_lay.addWidget(group_x_scale)
        control_lay = QVBoxLayout()
        control_lay.addLayout(buttons_lay)
        control_lay.addLayout(scale_lay)

        self.main_lay.addLayout(self.plot_lay)
        self.main_lay.addLayout(scale_lay)
        self.main_lay.addLayout(control_lay)
        
        self.setLayout(self.main_lay)

        # test curve
        # pen = pg.mkPen(color=(0, 0, 255), width=3, style=Qt.SolidLine)
        # self.data_line =  self.plt[0].plot([x*x for x in range(-10,11)], pen=pen, name="Signal 1")

        self.btn_new_plt.clicked.connect(self.btn_new_plt_clicked)
        self.btn_close_plt.clicked.connect(self.btn_close_plt_clicked)

    def plt_clicked(self, id):
        print(f"clicked {id}")
        self.cur_id = id

    def plt_data_dropped(self, id, data):
        print(f"id: {id}; dropped: {data}") 
        lst = self.disp_data.get(id)
        if lst == None:
            self.disp_data[id] = [data]
        else:
            lst.append(data)
            self.disp_data[id] = lst

        if (data in self.disp_data_lst) == False:
            self.disp_data_lst.append(data)
            self.data_dict[data] = []
            print(f"yo {data}")


        pen = pg.mkPen(color=self.colors[len(self.disp_data[id])-1], width=3, style=Qt.SolidLine)

        if len(self.data_line)-1 < id:
            self.data_line.append([])
        
        self.data_line[id].append(self.plt[id].plot([], pen=pen, name=data))
        
        print(self.disp_data)

    def btn_new_plt_clicked(self):
        print("new plot")
        self.new_id += 1
        self.new_plot(self.new_id)
        self.vsplitter.addWidget(self.plt[self.new_id])

    def btn_close_plt_clicked(self):
        print("close")
        self.del_plot(self.new_id)
        self.vsplitter.widget(self.new_id).deleteLater()
        self.new_id -= 1

    def addPoint(self, X, Y, signal_name: str):
        pass

    def add_data(self, data : dict):
        # self.data = data
        # if len(self.data_dict) < self.stored_data:
        #     self.data_dict.append(data.copy())
        # else:
        #     self.data_dict = self.data_dict[1:]
        #     self.data_dict.append(data.copy())
        #     # разобраться с проблемо дублирования времение
            # узнать, почему так много данных пытается сохраниться
        if len(self.disp_data_lst) > 0:
            self.it += self.dt
            if len(self.data_dict[self.disp_data_lst[0]]) < self.stored_data:
                self.time_data.append(self.it)
            else:
                self.time_data = self.time_data[1:]
                self.time_data.append(self.it)
            
        for x in self.disp_data_lst:
            if len(self.data_dict[x]) < self.stored_data:
                for y in data:
                    if x in data[y]:
                        self.data_dict[x].append(data[y][x])
                        break
            else:
                for y in data:
                    if x in data[y]:
                        self.data_dict[x] = self.data_dict[x][1:]
                        self.data_dict[x].append(data[y][x])
                        break
                
            
        # print(len(self.time_data))
        # print(self.data_lst[0]['square'])
    
    def set_dt(self, dt):
        self.dt = dt
        self.stored_data = self.plot_display_time/self.dt

    def update_plot_data(self):

        for x, y in self.disp_data.items():
            for i in range(len(y)):
                self.data_line[x][i].setData(self.time_data[-len(self.data_dict[y[i]]):], self.data_dict[y[i]])  #[-len(self.data_dict[y[i]]):]

        if len(self.time_data) > 0:       
            if self.time_data[-1] > self.plot_display_time:
                # self.ptr += self.refresh_time
                self.ptr = self.time_data[0]
                for i in range(len(self.plt)):
                    self.plt[i].setXRange(self.ptr, self.ptr+self.plot_display_time, padding=0)
            else:
                for i in range(len(self.plt)):
                    self.plt[i].setXRange(0, self.plot_display_time, padding=0)


                    


class MainWindow(QDialog):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.plt = Plot2D()
        # plt.setGeometry(1300, 600, 1500, 1200)
        # plt.setMinimumSize(1500, 1200)
        listWidget = QListWidget()
        listWidget.setMinimumWidth(400)
        listWidget.addItems(["sin11", "sin12", "sin13", "sin21", "sin22", "sin23", "sawtooth", "square"])
        listWidget.setDragEnabled(True)

        lay = QHBoxLayout()
        lay.addWidget(listWidget)
        lay.addWidget(self.plt)
        self.setLayout(lay)

        self.setGeometry(1300, 600, 1600, 1400)

        listWidget.itemClicked.connect(self.lst_clicked)

        # cretae data variables
        self.data = {'data': {'sin11': 0, 'sin12': 0, 'sin13': 0, 
                     'sin21': 0, 'sin22': 0, 'sin23': 0, 
                     'sawtooth': 0, 'square': 0, }}
        self.it = 0
        self.dt = 0.005
        self.plt.set_dt(self.dt)


        self.timer = QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def lst_clicked(self, item):
        print(f"Item: {item.text()}")

    def update_plot_data(self):
        self.data['data']['sin11'] = self.sin(1, 1, self.it)
        self.data['data']['sin12'] = self.sin(1, 2, self.it)
        self.data['data']['sin13'] = self.sin(1, 3, self.it)
        self.data['data']['sin21'] = self.sin(2, 1, self.it)
        self.data['data']['sin22'] = self.sin(2, 2, self.it)
        self.data['data']['sin23'] = self.sin(2, 3, self.it)
        self.data['data']['sawtooth'] = self.saw(1, 1, self.it)
        self.data['data']['square'] = self.square(1, 1, self.it)

        # print(saw)
        self.plt.add_data(self.data)

        self.it += self.dt

    def sin(self, a, omega, it):
        return a*np.sin(2*np.pi*omega*it)
    
    def saw(self, a, omega, it):
        return a*(it%(1/omega))*omega
    
    def square(self, a, omega, it):
        return a*np.sign(np.sin(2*np.pi*omega*it))

    

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())