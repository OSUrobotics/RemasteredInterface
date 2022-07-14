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
import rospy
import os



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
        fButton = self.outButton()
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

    def outButton(self):
        layout1 = QtWidgets.QHBoxLayout()
        button1 = QPushButton("Finish Setup", self)
        button1.clicked.connect(lambda:self.main_page.goToSecondScreen())
        button1.setFixedHeight(30)
        button1.setFixedWidth(150)
        layout1.addStretch(1)
        layout1.addWidget(button1)
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
            "apparatus" : "",
            "arm" : "",
            "live" : None,
            "item_count" : 2,
            "script_path" : os.path.dirname(os.path.realpath(__file__)),
            "package_dir" : os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        }
        self.imPath = None
        self.exPath = None
        bar = self.menuBar()
        self.actionExport = QAction("Export")
        self.actionImport = QAction("Import")
        file = bar.addMenu("File")
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



if __name__ == '__main__':
    rospy.init_node('infrastructure_project_gui_node', anonymous=True)
    app = QApplication(sys.argv)
    main = MainPage()
    main.show()
    sys.exit(app.exec_())
