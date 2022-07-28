#!/usr/bin/env python

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QGridLayout, QLabel, QCheckBox, QAction,  QInputDialog, QLineEdit, QFileDialog
import sys
import numpy as np
import multiprocessing
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from interface import Window
from sensor import Sensors
from sensor_msgs.msg import JointState
from infrastructure_msgs.msg import DoorSensors
import rospy
import os
import json


#Sets up the PyQt widget for the target object/setup selection
class ApparatusSelection(QWidget):
    def __init__(self, main_page):
        super(QWidget, self).__init__()
        self.main_page = main_page
        layout = QtWidgets.QHBoxLayout(self)
        self.button1 = QPushButton("Drawer", self)
        self.button1.clicked.connect(lambda:self.select("drawer"))
        self.button1.setFixedHeight(150)
        self.button1.setFixedWidth(150)
        self.button2 = QPushButton("Door", self)
        self.button2.clicked.connect(lambda:self.select("door"))
        self.button2.setFixedHeight(150)
        self.button2.setFixedWidth(150)
        self.button3 = QPushButton("Test Bed", self)
        self.button3.clicked.connect(lambda:self.select("testBed"))
        self.button3.setFixedHeight(150)
        self.button3.setFixedWidth(150)
        layout.addStretch(1)
        layout.addStretch(1)
        layout.addWidget(self.button1)
        layout.addStretch(1)
        layout.addWidget(self.button2)
        layout.addStretch(1)
        layout.addWidget(self.button3)
        layout.addStretch(1)
        layout.addStretch(1)
        self.setFixedHeight(200)


    def select(self, app):
        self.main_page.global_vars["apparatus"] = app
        self.changeColor(app)

    def changeColor(self, app):
        if (app == "drawer"):
            self.button1.setStyleSheet("background-color : lightgreen")
            self.button2.setStyleSheet("background-color : light gray")
            self.button3.setStyleSheet("background-color : light gray")
        elif (app == "door"):
            self.button1.setStyleSheet("background-color : light gray")
            self.button2.setStyleSheet("background-color : lightgreen")
            self.button3.setStyleSheet("background-color : light gray")
        else:
            self.button3.setStyleSheet("background-color : lightgreen")
            self.button1.setStyleSheet("background-color : light gray")
            self.button2.setStyleSheet("background-color : light gray")


#Sets up the PyQt widget for the specific robot being used
class ArmSelection(QWidget):
    def __init__(self, main_page):
        super(QWidget, self).__init__()
        self.main_page = main_page
        layout = QtWidgets.QHBoxLayout(self)
        self.button1 = QPushButton("Kinova Jaco2", self)
        self.button1.clicked.connect(lambda:self.select("kinova"))
        self.button1.setFixedHeight(150)
        self.button1.setFixedWidth(150)
        self.button2 = QPushButton("Thor Arm", self)
        self.button2.clicked.connect(lambda:self.select("thor"))
        self.button2.setFixedHeight(150)
        self.button2.setFixedWidth(150)
        layout.addStretch(1)
        layout.addStretch(1)
        layout.addWidget(self.button1)
        layout.addStretch(1)
        layout.addWidget(self.button2)
        layout.addStretch(1)
        layout.addStretch(1)
        self.setFixedHeight(200)

    def select(self, arm):
        self.main_page.global_vars["arm"] = arm
        self.changeColor(arm)

    def changeColor(self, arm):
        if (arm == "kinova"):
            self.button1.setStyleSheet("background-color : lightgreen")
            self.button2.setStyleSheet("background-color : light gray")
        elif (arm == "thor"):
            self.button1.setStyleSheet("background-color : light gray")
            self.button2.setStyleSheet("background-color : lightgreen")


#Creates the PyQt widget to select whether to run the visualization on
#live data or on recorded rosbags
class ModeSelection(QtWidgets.QWidget):
    def __init__(self, main_page):
        super(QWidget, self).__init__()
        self.main_page = main_page
        layout = QtWidgets.QHBoxLayout(self)
        self.button1 = QPushButton("Live Mode", self)
        self.button1.clicked.connect(lambda:self.select(True))
        self.button1.setFixedHeight(50)
        self.button1.setFixedWidth(150)
        self.button2 = QPushButton("Import Mode", self)
        self.button2.clicked.connect(lambda:self.select(False))
        self.button2.setFixedHeight(50)
        self.button2.setFixedWidth(150)
        layout.addStretch(1)
        layout.addStretch(1)
        layout.addWidget(self.button1)
        layout.addStretch(1)
        layout.addWidget(self.button2)
        layout.addStretch(1)
        layout.addStretch(1)
        self.setFixedHeight(200)

    def select(self, is_live):
        self.main_page.global_vars["live"] = is_live
        self.changeColor(is_live)

    def changeColor(self, is_live):
        if (is_live):
            self.button1.setStyleSheet("background-color : lightgreen")
            self.button2.setStyleSheet("background-color : light gray")
        else:
            self.button1.setStyleSheet("background-color : light gray")
            self.button2.setStyleSheet("background-color : lightgreen")


#Sets up the initial GUI page
class FirstPage(QWidget):
    def __init__(self, parent, main_page):
        super(QWidget, self).__init__(parent)
        self.main_page = main_page
        self.layout = QtWidgets.QVBoxLayout(self)
        self.p = parent
        self.apparatusSelect = ApparatusSelection(self.main_page)
        self.armSelect = ArmSelection(self.main_page)
        self.modeSelect = ModeSelection(self.main_page)
        title = self.createLabel("Equipment Setup", 24)
        wr1Layout = self.createLabel("Select Your Apparatus", 16)
        wr2Layout = self.createLabel("Select Your Arm", 16)
        wr3Layout = self.createLabel("Select Data Mode", 16)
        fButton = self.configRow()
        self.layout.addLayout(title)
        self.layout.addStretch()
        self.layout.addLayout(wr1Layout)
        self.layout.addWidget(self.apparatusSelect)
        self.layout.addStretch()
        self.layout.addLayout(wr2Layout)
        self.layout.addWidget(self.armSelect)
        self.layout.addStretch()
        self.layout.addLayout(wr3Layout)
        self.layout.addWidget(self.modeSelect)
        self.layout.addStretch()
        self.layout.addLayout(fButton)
        self.layout.addStretch()


    def getLayout(self):
        return self.layout

    def createLabel(self, text, fontSize):
        layout1 = QtWidgets.QHBoxLayout()
        label = QLabel(text, self)
        label.setFont(QFont('Times', fontSize))
        layout1.addStretch(1)
        layout1.addWidget(label)
        layout1.addStretch(1)
        return layout1

    def configRow(self):
        layout1 = QtWidgets.QHBoxLayout()
        button1 = QPushButton("Load Config", self)
        button1.clicked.connect(lambda:self.main_page.changeConfig())
        button1.setFixedHeight(30)
        button1.setFixedWidth(150)
        button2 = QPushButton("Finish Setup", self)
        button2.clicked.connect(lambda:self.main_page.goToSecondScreen())
        button2.setStyleSheet("background-color: green;")
        button2.setFixedHeight(30)
        button2.setFixedWidth(150)
        layout1.addStretch(1)
        layout1.addWidget(button1)
        layout1.addWidget(button2)
        layout1.addStretch(1)
        return layout1


class MainPage(QtWidgets.QMainWindow):
    def __init__(self):
        super(QWidget, self).__init__()
        self.global_vars = {
            "queue" : multiprocessing.Queue(),
            "distanceQueue" : multiprocessing.Queue(),
            "bagQueue" : multiprocessing.Queue(),
            "sensorBuffer" : [],
            "buffer" : [],
            "timeBuffer" : [],
            "otherWindows" : [],
            "rviz_instances" : [],
            "sensor_subscriber" : None,
            "joints_subscriber" : None,
            "sensor_publisher" : rospy.Publisher("infrastructure_gui/sensor_data", DoorSensors, queue_size=10),
            "joints_publisher" : rospy.Publisher("infrastructure_gui/joint_states", JointState, queue_size=10),
            "sensor_data_topic" : "sensor_data",
            "joint_state_topic" : "joint_states",
            "apparatus" : "",
            "arm" : "",
            "live" : None,
            "item_count" : 2,
            "robot_state_publisher" : None,
            "items" : {
                "item_1" : {
                    "type" : "none"
                },
                "item_2" : {
                    "type" : "none"
                },
                "item_3" : {
                    "type" : "none"
                },
                "item_4" : {
                    "type" : "none"
                }
            },
            "sensors_enabled" : [],
            "script_path" : os.path.dirname(os.path.realpath(__file__)),
            "package_dir" : os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        }
        self.imPath = None
        self.exPath = None
        bar = self.menuBar()

        file = bar.addMenu("File")
        self.actionExport = QAction("Export")
        self.actionImport = QAction("Import")
        file.addAction("New")
        file.addAction("Save Setup")
        file.addAction(self.actionExport)
        file.addAction(self.actionImport)

        view = bar.addMenu("View")
        self.action1 = QAction("1 Element")
        self.action2 = QAction("2 Elements")
        self.action3 = QAction("4 Elements")
        view.addAction(self.action1)
        view.addAction(self.action2)
        view.addAction(self.action3)
        self.actionExport.triggered.connect(lambda:self.setExportPath())
        self.actionImport.triggered.connect(lambda:self.setImportPath())
        self.action1.triggered.connect(lambda:self.changeGrid(1))
        self.action2.triggered.connect(lambda:self.changeGrid(2))
        self.action3.triggered.connect(lambda:self.changeGrid(4))

        self.showMaximized()
        self.firstPage = FirstPage(self, self)
        self.setCentralWidget(self.firstPage)

    def setExportPath(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName = QFileDialog.getExistingDirectory(self, "Select Directory")
        if fileName:
            self.exPath = fileName
            print(self.exPath)

    def setImportPath(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","All Files (*);;Python Files (*.py)", options=options)
        if fileName:
            self.imPath = fileName

    def changeGrid(self, i):
        self.global_vars["item_count"] = i
        if self.global_vars["apparatus"] != ""  and self.global_vars["arm"] != "" and self.global_vars["live"] != None:
            self.setCentralWidget(Window(self.global_vars["apparatus"], self.global_vars["arm"], self.global_vars["live"], self.global_vars["item_count"], self, self))

    def changeWidget(self):
        if self.global_vars["apparatus"] != ""  and self.global_vars["arm"] != "" and self.global_vars["live"] != None:
            self.setCentralWidget(Window(self.global_vars["apparatus"], self.global_vars["arm"], self.global_vars["live"], self.global_vars["item_count"], self, self))

    def changeMode(self, is_live):
        self.global_vars["live"] = is_live
        self.setCentralWidget(Window(self.global_vars["apparatus"], self.global_vars["arm"], self.global_vars["live"], self.global_vars["item_count"], self, self))

    def changeApparatus(self, app_number):
        if app_number == 0:
            self.global_vars["apparatus"] = "drawer"
        elif app_number == 1:
            self.global_vars["apparatus"] = "door"
        else:
            self.global_vars["apparatus"] = "testBed"

        self.setCentralWidget(Window(self.global_vars["apparatus"], self.global_vars["arm"], self.global_vars["live"], self.global_vars["item_count"], self, self))

    def changeArm(self, arm_number):
        if arm_number == 0:
            self.global_vars["arm"] = "kinova"
        else:
            self.global_vars["arm"] = "thor"
        self.setCentralWidget(Window(self.global_vars["apparatus"], self.global_vars["arm"], self.global_vars["live"], self.global_vars["item_count"], self, self))

    def goToSecondScreen(self):
            self.changeWidget()

    def changeConfig(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fname, _ = QFileDialog.getOpenFileName(self, "QFileDialog.getOpenFileName()", self.global_vars["package_dir"] + '/config',"All Files (*);;JSON files (*.json)", options=options)
        with open(fname) as f:
            config = json.load(f)
        for key in config.keys():
            if key is "mode":
                if config["mode"] is "live":
                    self.global_vars["live"] = True
                else:
                    self.global_vars["live"] = False
            else:
                self.global_vars[key] = config[key]

        self.setCentralWidget(Window(self.global_vars["apparatus"], self.global_vars["arm"], self.global_vars["live"], self.global_vars["item_count"], self, self))


    def saveConfig(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fname, _ = QFileDialog.getSaveFileName(self, "QFileDialog.getSaveFileName()", self.global_vars["package_dir"] + '/config',"All Files (*);;JSON files (*.json)", options=options)
        config = {}
        config["apparatus"] = self.global_vars["apparatus"]
        config["arm"] = self.global_vars["arm"]
        if self.global_vars["live"]:
            config["mode"] = "live"
        else:
            config["mode"] = "rosbag"
        config["item_count"] = self.global_vars["item_count"]
        config["items"] = self.global_vars["items"]
        config["sensor_data_topic"] = self.global_vars["sensor_data_topic"]
        config["joint_state_topic"] = self.global_vars["joint_state_topic"]
        config["sensors_enabled"] = self.global_vars["sensors_enabled"]
        with open(fname, "w+") as f:
            f.write(json.dumps(config, indent=4))

    def resetRvizTimes(self):
        for instance in self.global_vars["rviz_instances"]:
            instance.reset_time()


if __name__ == '__main__':
    rospy.init_node('infrastructure_project_gui_node', anonymous=True)
    app = QApplication(sys.argv)
    main = MainPage()
    main.show()
    sys.exit(app.exec_())
