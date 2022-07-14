from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import math
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QComboBox, QWidget, QApplication, QPushButton, QGridLayout, QLabel, QCheckBox, QInputDialog, QLineEdit, QFileDialog
import sys
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import matplotlib.animation as animation
import random
import rospy
import rosnode
from infrastructure_msgs.msg import DoorSensors
from sensor_msgs.msg import JointState
from sensor import Sensors
from items import *
import os
import atexit
import signal
import subprocess
import multiprocessing
import threading
import pandas as pd
import rosbag
import roslaunch
import uuid
import rviz
from time import sleep





# Wrapper class for generating canvas in the application
class CanvasFigure(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=4, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super(CanvasFigure, self).__init__(self.fig)
        self.axes = self.fig.add_subplot(111)


# class for openning a graph in a sep window
class DifferentWindows(QtWidgets.QMainWindow):
    def __init__(self, p, t, status = [], index = -1, num = 0 ):
        super(QWidget, self).__init__()
        self.showMaximized()
        self.setWindowTitle('Distance Graph')
        w = Wrapper(p, status, t, index, num)
        self.setCentralWidget(w)

# class wrapper class with the selection of what graph to open and layout structure.
# Used to setup interface
class Wrapper(QtWidgets.QWidget):
    def __init__(self, p, status, t, index, num):
        super(QWidget, self).__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout1 = QtWidgets.QHBoxLayout(self)

        self.setLayout(layout)
        layout.addStretch()
        layout.addLayout(layout1)
        layout.addStretch()
        if (t == 0):
            self.g = GraphDistance(p, index, num, p.main_page)
        if (t == 1):
            self.g = GraphFSR(p, status, index, num, p.main_page)
        if (t == 2):
            self.g = GraphImage(p, status, index, num, p.main_page)
        if (t == 3):
            self.g = RvizWidget(p, index, num, p.main_page)
        layout1.addStretch()
        layout1.addWidget(self.g)
        layout1.addStretch()

# Widget to select item placed on the testbed
class Items(QtWidgets.QWidget):
    def __init__(self, p, statusArray, index, num):
        super(QWidget, self).__init__()
        self.statusArray = statusArray
        self.parent = p
        self.index = index
        if num == 1:
            self.height = 700
            self.width = 1000
        elif num == 2:
            self.height = 550
            self.width = 600
        elif num == 0:
            self.width = 1200
            self.height = 800
        else:
            self.width = 500
            self.height = 350
        self.setFixedWidth(self.width)
        self.setFixedHeight(self.height)
        button = QPushButton()
        button.setText("Export Path")
        button.setFixedWidth(120)
        button.setFixedHeight(30)
        buttonBack = QPushButton()
        buttonBack.setText("Remove")
        buttonBack.setStyleSheet("background-color: red;")
        buttonBack.setFixedWidth(120)
        buttonBack.setFixedHeight(30)
        buttonBack.clicked.connect(lambda:self.parent.goBackToSelection(self.index))
        buttonNewWindow = QPushButton()
        buttonNewWindow.setText("New Window")
        buttonNewWindow.setFixedWidth(120)
        buttonNewWindow.setFixedHeight(30)
        buttonNewWindow.clicked.connect(lambda:self.openInNewWindow(self.parent))
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout1 =  QtWidgets.QHBoxLayout(self)

        if num != 0:
            self.layout.addLayout(self.layout1)
        self.layout.addWidget(ObjectChoice(self.parent))
        self.layout1.addStretch()
        self.layout1.addWidget(buttonNewWindow)
        self.layout1.addWidget(buttonBack)
        self.setLayout(self.layout)

# Graph fsr data against time using canvas
class GraphFSR(QtWidgets.QWidget):
    def __init__(self, p, statusArray, index, num, main_page):
        self.statusArray = statusArray
        self.parent = p
        self.main_page = main_page
        super(QWidget, self).__init__()
        self.index = index
        if num == 1:
            self.height = 700
            self.width = 1000
        elif num == 2:
            self.height = 550
            self.width = 600
        elif num == 0:
            self.width = 1200
            self.height = 800
        else:
            self.width = 500
            self.height = 350
        self.setFixedWidth(self.width)
        self.setFixedHeight(self.height)
        button = QPushButton()
        button.setText("Export Path")

        button.setFixedWidth(120)
        button.setFixedHeight(30)

        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout1 =  QtWidgets.QHBoxLayout(self)
        buttonBack = QPushButton()
        buttonBack.setText("Remove")
        buttonBack.setStyleSheet("background-color: red;")
        buttonBack.setFixedWidth(120)
        buttonBack.setFixedHeight(30)

        buttonBack.clicked.connect(lambda:self.parent.goBackToSelection(self.index))
        buttonNewWindow = QPushButton()
        buttonNewWindow.setText("New Window")
        buttonNewWindow.setFixedWidth(120)
        buttonNewWindow.setFixedHeight(30)
        buttonNewWindow.clicked.connect(lambda:self.openInNewWindow())

        self.canvas = CanvasFigure(self, width=5, height=4, dpi=100)
        self.canvas.fig.suptitle('FSR Graph', fontsize=14)
        #self.layout1.addWidget(button)
        if num != 0:
            self.layout.addLayout(self.layout1)
        self.layout.addWidget(self.canvas)
        self.layout1.addStretch()
        self.layout1.addWidget(buttonNewWindow)
        self.layout1.addWidget(buttonBack)
        self.setLayout(self.layout)
        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()



    def openInNewWindow(self):
        self.main_page.global_vars["otherWindows"].append(DifferentWindows(self.parent, 1, self.statusArray))



    def update_plot(self):
        a = []
        time = []
        names = []
        i = 0
        if not self.main_page.global_vars["live"]:
            self.main_page.global_vars["sensorBuffer"] = []
            if len(self.parent.bagData) != 0:
                while i <= len(self.parent.bagData) -1 and (self.parent.bagData[i][1]-self.parent.t_start).to_sec() < self.parent.currentValue:
                    d = self.parent.bagData[i][0]
                    self.main_page.global_vars["sensorBuffer"] = self.main_page.global_vars["sensorBuffer"] + [Sensors(d.fsr1, d.fsr2, d.fsr3, d.fsr4, d.fsr5, d.fsr6, d.fsr7,
                                            d.fsr8, d.fsr9, d.fsr10, d.fsr11, d.fsr12, d.current_time)]
                    i = i+1


        else:
            while self.main_page.global_vars["queue"].empty() == 0:
                d = self.main_page.global_vars["queue"].get()
                self.main_page.global_vars["sensorBuffer"] = self.main_page.global_vars["sensorBuffer"] + [Sensors(d.fsr1, d.fsr2, d.fsr3, d.fsr4, d.fsr5, d.fsr6, d.fsr7,
                                        d.fsr8, d.fsr9, d.fsr10, d.fsr11, d.fsr12, d.current_time)]


        l = len(self.main_page.global_vars["sensorBuffer"])
        bound = min(l, 20)

        inner = []
        for i in range (bound):
            inner = []
            time.append(self.main_page.global_vars["sensorBuffer"][l-bound+i].time.to_sec())
            if self.statusArray[0] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr1]
            if self.statusArray[1] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr2]
            if self.statusArray[2] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr3]
            if self.statusArray[3] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr4]
            if self.statusArray[4] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr5]
            if self.statusArray[5] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr6]
            if self.statusArray[6] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr7]
            if self.statusArray[7] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr8]
            if self.statusArray[8] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr9]
            if self.statusArray[9] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr10]
            if self.statusArray[10] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr11]
            if self.statusArray[11] == 1:
                inner = inner + [self.main_page.global_vars["sensorBuffer"][l-bound+i].fsr12]
            a.append(inner)
        if self.statusArray[0] == 1:
            names = names + ["FSR1"]
        if self.statusArray[1] == 1:
            names = names + ["FSR2"]
        if self.statusArray[2] == 1:
            names = names + ["FSR3"]
        if self.statusArray[3] == 1:
            names = names + ["FSR4"]
        if self.statusArray[4] == 1:
            names = names + ["FSR5"]
        if self.statusArray[5] == 1:
            names = names + ["FSR6"]
        if self.statusArray[6] == 1:
            names = names + ["FSR7"]
        if self.statusArray[7] == 1:
            names = names + ["FSR8"]
        if self.statusArray[8] == 1:
            names = names + ["FSR9"]
        if self.statusArray[9] == 1:
            names = names + ["FSR10"]
        if self.statusArray[10] == 1:
            names = names + ["FSR11"]
        if self.statusArray[11] == 1:
            names = names + ["FSR12"]
        self.canvas.axes.cla()

        if  len(inner) != 0:
            df = pd.DataFrame(a, time, columns = names)
            df.plot(ax = self.canvas.axes)
        self.canvas.draw()

# canvas and controls for graping Distance against Time
class GraphDistance(QtWidgets.QWidget):
    def __init__(self, p, index, num, main_page):
        super(QWidget, self).__init__()
        self.parent = p
        self.index = index
        self.main_page = main_page
        if num == 1:
            self.height = 700
            self.width = 1000
        elif num == 2:
            self.height = 550
            self.width = 600
        elif num == 0:
            self.width = 1200
            self.height = 800
        else:
            self.width = 500
            self.height = 350

        self.setFixedWidth(self.width)
        self.setFixedHeight(self.height)
        button = QPushButton()
        button.setText("Export Path")

        button.setFixedWidth(120)
        button.setFixedHeight(30)
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout1 =  QtWidgets.QHBoxLayout(self)
        buttonBack = QPushButton()
        buttonBack.setText("Remove")
        buttonBack.setStyleSheet("background-color: red;")
        buttonBack.setFixedWidth(120)
        buttonBack.setFixedHeight(30)
        buttonNewWindow = QPushButton()
        buttonNewWindow.setText("New Window")
        buttonNewWindow.setFixedWidth(120)
        buttonNewWindow.setFixedHeight(30)
        buttonBack.clicked.connect(lambda:self.parent.goBackToSelection(self.index))
        buttonNewWindow.clicked.connect(lambda:self.openInNewWindow())
        self.canvas = CanvasFigure(self, width=5, height=4, dpi=100)
        self.canvas.fig.suptitle('Distance Graph', fontsize=14)
        #self.layout1.addWidget(button)
        if num != 0:
            self.layout.addLayout(self.layout1)
        self.layout.addWidget(self.canvas)
        self.layout1.addStretch()
        self.layout1.addWidget(buttonNewWindow)
        self.layout1.addWidget(buttonBack)

        self.setLayout(self.layout)




        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    # update plot in either  live - 0 or recorded - 1 mode
    def update_plot(self):
        if not self.main_page.global_vars["live"]:
            self.main_page.global_vars["buffer"] = []
            self.main_page.global_vars["timeBuffer"] = []
            i = 0
            if len(self.parent.bagData) != 0:
                while  i <= len(self.parent.bagData) -1 and (self.parent.bagData[i][1]-self.parent.t_start).to_sec() < self.parent.currentValue:
                    d = self.parent.bagData[i][0]
                    self.main_page.global_vars["buffer"] = self.main_page.global_vars["buffer"] + [d.tof]
                    self.main_page.global_vars["timeBuffer"] = self.main_page.global_vars["timeBuffer"] + [d.current_time.to_sec()]
                    i = i+1

        else:
            while not self.main_page.global_vars["distanceQueue"].empty():
                d = self.main_page.global_vars["distanceQueue"].get()
                self.main_page.global_vars["buffer"] = self.main_page.global_vars["buffer"] + [d.tof]
                self.main_page.global_vars["timeBuffer"] = self.main_page.global_vars["timeBuffer"] + [d.current_time.to_sec()]


        self.canvas.axes.cla()
        if len(self.main_page.global_vars["buffer"]) != 0:
            df = pd.DataFrame(self.main_page.global_vars["buffer"], self.main_page.global_vars["timeBuffer"], columns = ["Distance"])
            df.plot(ax = self.canvas.axes)
        self.canvas.draw()


    # open the widget in a new window
    def openInNewWindow(self):
        self.main_page.global_vars["otherWindows"].append(DifferentWindows(self.parent, 0))


class GraphImage(QWidget):
    def __init__(self, p, statusArray, index, num, main_page):
        super(QWidget, self).__init__()
        self.visibleButtons = statusArray
        self.index = index
        self.parent = p
        self.main_page = main_page
        if num == 0:
            self.parent.otherWindows.append(self)
        if num == 1:
            self.height = 700
            self.width = 1000
        elif num == 2:
            self.height = 550
            self.width = 600
        elif num == 0:
            self.width = 1200
            self.height = 800
        else:
            self.width = 500
            self.height = 350
        self.setFixedWidth(self.width)
        self.setFixedHeight(self.height)
        self.fig = Figure()
        buttonBack = QPushButton()
        buttonBack.setText("Remove")
        buttonBack.setStyleSheet("background-color: red;")
        buttonBack.setFixedWidth(120)
        buttonBack.setFixedHeight(30)
        buttonBack.clicked.connect(lambda:self.parent.goBackToSelection(self.index))
        buttonNewWindow = QPushButton()
        buttonNewWindow.setText("New Window")
        buttonNewWindow.setFixedWidth(120)
        buttonNewWindow.setFixedHeight(30)
        buttonNewWindow.clicked.connect(lambda:self.openInNewWindow())
        self.canvas = FigureCanvas(self.fig)
        self.axes = self.fig.add_subplot(111, projection='3d')
        self.layout = QtWidgets.QVBoxLayout(self)
        buttonLaout = QtWidgets.QHBoxLayout(self)
        self.controlLayot = QtWidgets.QHBoxLayout(self)
        self.text = QCheckBox("Display Labels",self)
        self.text.setChecked(True)
        self.text.stateChanged.connect(self.toggleText)
        self.controlLayot.addStretch()
        self.controlLayot.addWidget(buttonNewWindow)
        self.controlLayot.addWidget(buttonBack)
        if num != 0:
            self.layout.addLayout(self.controlLayot)
        self.layout.addWidget(self.canvas)
        self.layout.addLayout(buttonLaout)
        buttonLaout.addStretch()
        buttonLaout.addWidget(self.text)
        self.marks = QCheckBox("Display Marks",self)
        buttonLaout.addStretch()
        self.marks.setChecked(True)
        self.marks.stateChanged.connect(self.toggleMarks)
        buttonLaout.addWidget(self.marks)
        buttonLaout.addStretch()
        self.your_mesh = mesh.Mesh.from_file(self.main_page.global_vars["package_dir"] + '/stl_meshes/Main.stl')
        self.meshArray = []
        self.sensors = []
        self.texts = []

        for i in range (12):
            self.meshArray.append(mesh.Mesh.from_file(self.main_page.global_vars["package_dir"] + '/stl_meshes/sensor' + str(i+1) + '.stl'))
            self.sensors.append(mplot3d.art3d.Line3DCollection(self.meshArray[i].vectors, colors=(1,0,0,1), facecolors=(1,0,0,1), alpha=1, zorder=1, visible=False))
            self.axes.add_collection3d(self.sensors[i])
            self.texts.append(self.axes.text3D(self.meshArray[i].vectors[0][1][0],self.meshArray[i].vectors[0][1][1],self.meshArray[i].vectors[0][1][2],  "sensor #" + str(i+1),  visible=False))

        self.mainOjbect = mplot3d.art3d.Line3DCollection(self.your_mesh.vectors,colors=(0,0,0,1),  facecolors='gray', alpha=0.15, linewidths=0.2, zorder=2, visible=True)
        self.axes.add_collection3d(self.mainOjbect)
        scale = self.your_mesh.points.flatten()
        self.axes.auto_scale_xyz(scale*0.7, scale*0.7, scale*0.7)
        self.axes.set_axis_off()

        self.toggleMarks(QtCore.Qt.Checked)
        self.toggleText(QtCore.Qt.Checked)


    def setvisibleButtons(self, v):
        self.visibleButtons = v


    def toggleText(self, state):
        if (state == QtCore.Qt.Checked):
            for i in range (12):
                if (self.visibleButtons[i] == 1):
                    self.showMesh(i)
        else:
            for i in range (12):
                self.hideText(i)
                self.canvas.draw()


    def toggleMarks(self, state):
        if (state == QtCore.Qt.Checked):
            for i in range (12):
                if (self.visibleButtons[i] == 1):
                    self.showMesh(i)
        else:
            for i in range (12):
                 self.sensors[i].set_visible(False)
                 self.canvas.draw()


    def showMesh(self, index):
        if (self.marks.isChecked()):
            self.sensors[index].set_visible(True)
        if (self.text.isChecked()):
            self.showText(index)
        self.canvas.draw()


    def hideText(self, index):
        self.texts[index].set_visible(False)


    def showText(self,index):
        self.texts[index].set_visible(True)


    def hideMesh(self, index):
        self.sensors[index].set_visible(False)
        self.hideText(index)
        self.canvas.draw()


    def openInNewWindow(self):
        self.main_page.global_vars["otherWindows"].append(DifferentWindows(self.parent, 2, self.visibleButtons))

class RvizWidget(QtWidgets.QWidget):
    def __init__(self, p, index, num, main_page):
        super(QWidget, self).__init__()
        self.parent = p
        self.main_page = main_page
        self.index = index
        if num == 1:
            self.height = 700
            self.width = 1000
        elif num == 2:
            self.height = 550
            self.width = 600
        elif num == 0:
            self.width = 1200
            self.height = 800
        else:
            self.width = 500
            self.height = 350

        self.setFixedWidth(self.width)
        self.setFixedHeight(self.height)
        button = QPushButton()
        button.setText("Export Path")

        button.setFixedWidth(120)
        button.setFixedHeight(30)
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout1 =  QtWidgets.QHBoxLayout(self)
        buttonBack = QPushButton()
        buttonBack.setText("Remove")
        buttonBack.setStyleSheet("background-color: red;")
        buttonBack.setFixedWidth(120)
        buttonBack.setFixedHeight(30)
        buttonNewWindow = QPushButton()
        buttonNewWindow.setText("New Window")
        buttonNewWindow.setFixedWidth(120)
        buttonNewWindow.setFixedHeight(30)
        buttonLoadConfig = QPushButton()
        buttonLoadConfig.setText("Load Config")
        buttonLoadConfig.setFixedWidth(120)
        buttonLoadConfig.setFixedHeight(30)
        buttonBack.clicked.connect(lambda:self.parent.goBackToSelection(self.index))
        buttonNewWindow.clicked.connect(lambda:self.openInNewWindow())
        buttonLoadConfig.clicked.connect(self.changeConfig)
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()

        if self.main_page.global_vars["arm"] is "kinova":
            reader.readFile( config, self.main_page.global_vars["package_dir"] + "/rviz/kinova_arm_basic.rviz" )
        else:
            reader.readFile( config, self.main_page.global_vars["package_dir"] + "/rviz/blankconfig.rviz" )

        self.frame.load( config )


        #Please read before uncommenting:
        #Attempting to use the "Load Config" button in its current form
        #with the menu bar hidden results in a segfault. I don't know
        #what specifically within the rviz source code causes this.
        #If you really want to try to fix this, I suggest diving more into
        #the frame manager. I hope to fix this at some point myself, but at
        #present there are various things that take priority over not having the
        #menu bar at the top of the widget.
        #self.frame.setMenuBar( None )

        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )
        if num != 0:
            self.layout.addLayout(self.layout1)
        self.layout.addWidget(self.frame)
        self.layout1.addStretch()
        self.layout1.addWidget(buttonLoadConfig)
        self.layout1.addWidget(buttonNewWindow)
        self.layout1.addWidget(buttonBack)

        self.setLayout(self.layout)

        if self.main_page.global_vars["arm"] is "kinova":
            self.initializeJaco2()

    def openInNewWindow(self):
        self.main_page.global_vars["otherWindows"].append(DifferentWindows(self.parent, 3))

    def changeConfig(self):

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fname, _ = QFileDialog.getOpenFileName(self, "QFileDialog.getOpenFileName()", self.main_page.global_vars["package_dir"] + '/rviz',"All Files (*);;rviz configuration files (*.rviz *.myviz)", options=options)
        print(fname)
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, fname )
        print("Read config")
        self.frame.load( config )
        print("Loaded config")

    def initializeJaco2(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [self.main_page.global_vars["package_dir"] + "/launch/display_jaco_2.launch"])
        launch.start()
        #pub = rospy.Publisher('/j2s7s300_driver/out/joint_state', JointState, queue_size=1)

        #initial_message = JointState()
        #initial_message.header.stamp = rospy.Time.now()
        #initial_message.header.frame_id = ''
        #initial_message.name = ["j2s7s300_joint_1",
        #                        "j2s7s300_joint_2",
        #                        "j2s7s300_joint_3",
        #                        "j2s7s300_joint_4",
        #                        "j2s7s300_joint_5",
        #                        "j2s7s300_joint_6",
        #                        "j2s7s300_joint_7",
        #                        "j2s7s300_joint_finger_1",
        #                        "j2s7s300_joint_finger_tip_1",
        #                        "j2s7s300_joint_finger_2",
        #                        "j2s7s300_joint_finger_tip_2",
        #                        "j2s7s300_joint_finger_3",
        #                        "j2s7s300_joint_finger_tip_3"]
        #initial_message.position = [0.0, 3.1415, 3.1415, 3.1415, 0.0, 3.14159265359, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #initial_message.velocity = []
        #initial_message.effort = []

        #rospy.sleep(0.5)

        #pub.publish(initial_message)
        #rospy.spin()

    def update_joints(self):
        pass
