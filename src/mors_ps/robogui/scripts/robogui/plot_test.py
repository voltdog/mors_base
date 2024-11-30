from PyQt5 import QtWidgets
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
from PyQt5 import QtCore
from random import randint

import numpy as np
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *



class MainWindow(QtWidgets.QDialog):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setTitle("Your Title Here", color="k", size="15pt")
        self.graphWidget.addLegend()
        pg.setConfigOptions(antialias=True)

        btn_y_plus = QPushButton("Y+")
        btn_y_minus = QPushButton("Y-")
        btn_x_plus = QPushButton("X+")
        btn_x_minus = QPushButton("X-")
        hSpacer = QSpacerItem(20, 40, QSizePolicy.Expanding, QSizePolicy.Minimum)
        lay1 = QHBoxLayout()
        lay1.addWidget(btn_y_minus)
        lay1.addWidget(btn_y_plus)
        lay1.addWidget(btn_x_minus)
        lay1.addWidget(btn_x_plus)
        lay1.addItem(hSpacer)

        lay2 = QVBoxLayout()
        lay2.addWidget(self.graphWidget)
        lay2.addLayout(lay1)

        self.setLayout(lay2)

        hour = [1,2,3,4,5,6,7,8,9,10]
        temperature_1 = [30,32,34,32,33,31,29,32,35,45]
        temperature_2 = [50,35,44,22,38,32,27,38,32,44]

        self.graphWidget.setBackground('w')
        pen = pg.mkPen(color=(255, 0, 0), width=3, style=QtCore.Qt.SolidLine)

        styles = {'color':'b', 'font-size':'10pt'}
        self.graphWidget.setLabel('left', 'Temperature (°C)', **styles)
        self.graphWidget.setLabel('bottom', 'Hour (H)', **styles)
        self.graphWidget.showGrid(x=True, y=True)

        # self.graphWidget.setXRange(0, 100, padding=0)
        self.graphWidget.setYRange(-1.1, 1.1, padding=0)

        # self.plot(hour, temperature_1, "Sensor1", 'r')
        # self.plot(hour, temperature_2, "Sensor2", 'b')

        self.x = list(range(1000))  # 100 time points
        # self.y = [randint(0,100) for _ in range(100)]  # 100 data points
        self.t = [i/20 for i in self.x]
        self.y = [np.sin(x) for x in self.t]

        self.data_line =  self.graphWidget.plot(self.x, self.y, pen=pen, name="Signal")

        self.timer = QtCore.QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

        self.ptr = -1

        # set signals and slots
        btn_x_plus.clicked.connect(self.x_plus)
        btn_x_minus.clicked.connect(self.x_minus)
        self.x_scale = 0
        btn_y_plus.clicked.connect(self.y_plus)
        btn_y_minus.clicked.connect(self.y_minus)
        self.y_scale = 1.1
    
    def x_plus(self):
        self.x_scale += 50
        self.graphWidget.setXRange(self.x_scale, 1000, padding=0)

    def x_minus(self):
        self.x_scale -= 50
        self.graphWidget.setXRange(self.x_scale, 1000, padding=0)

    def y_plus(self):
        self.y_scale += 0.1
        self.graphWidget.setYRange(-self.y_scale, self.y_scale, padding=0)

    def y_minus(self):
        self.y_scale -= 0.1
        self.graphWidget.setYRange(-self.y_scale, self.y_scale, padding=0)

    def update_plot_data(self):

        self.x = self.x[1:]  # Remove the first y element.
        self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last.
        self.ptr += 1

        self.y = self.y[1:]  # Remove the first
        # self.y.append( randint(0,100)/100)  # Add a new random value.
        self.y.append(np.sin(self.x[-1]/20))

        self.data_line.setData(self.x, self.y)  # Update the data.
        # self.data_line.setPos(-self.ptr, 0)
        self.graphWidget.setXRange(self.ptr, self.ptr+1000, padding=0)

    def plot(self, x, y, plotname, color):
        pen = pg.mkPen(color=color, width=3, style=QtCore.Qt.SolidLine)
        self.graphWidget.plot(x, y, name=plotname, pen=pen)


def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()