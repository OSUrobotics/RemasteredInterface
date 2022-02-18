from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QGridLayout, QLabel, QCheckBox, QAction
import sys
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtCore import *
from PyQt5.QtGui import * 
from interface import Window




class FirstChoice(QWidget):
    def __init__(self):
        self.selection = 0
        super(QWidget, self).__init__() 
        layout = QtWidgets.QHBoxLayout(self)
        self.button1 = QPushButton("Drawer", self)
        self.button1.clicked.connect(lambda:self.select(1))
        self.button1.setFixedHeight(150)
        self.button1.setFixedWidth(150)
        self.button2 = QPushButton("Door", self)
        self.button2.clicked.connect(lambda:self.select(2))
        self.button2.setFixedHeight(150)
        self.button2.setFixedWidth(150)
        self.button3 = QPushButton("Test Bed", self)
        self.button3.clicked.connect(lambda:self.select(3))
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
        self.setFixedHeight(250)

    
    def select(self, i):
       
        self.selection = i
        self.changeColor(i)

    def changeColor(self, i):
        if (i==1):
            self.button1.setStyleSheet("background-color : lightgreen")
            self.button2.setStyleSheet("background-color : light gray")
            self.button3.setStyleSheet("background-color : light gray")
        elif (i==2):
            self.button1.setStyleSheet("background-color : light gray")
            self.button2.setStyleSheet("background-color : lightgreen")
            self.button3.setStyleSheet("background-color : light gray")
        else:
            self.button3.setStyleSheet("background-color : lightgreen")
            self.button1.setStyleSheet("background-color : light gray")
            self.button2.setStyleSheet("background-color : light gray")

    

   


class SecondChoice(QWidget):
    def __init__(self):
        super(QWidget, self).__init__() 
        self.selection = 0
        layout = QtWidgets.QHBoxLayout(self)
        self.button1 = QPushButton("Kinova Jaco2", self)
        self.button1.clicked.connect(lambda:self.select(1))
        self.button1.setFixedHeight(150)
        self.button1.setFixedWidth(150)
        self.button2 = QPushButton("Thor Arm", self)
        self.button2.clicked.connect(lambda:self.select(2))
        self.button2.setFixedHeight(150)
        self.button2.setFixedWidth(150)
        layout.addStretch(1)
        layout.addStretch(1)
        layout.addWidget(self.button1)
        layout.addStretch(1)
        layout.addWidget(self.button2)
        layout.addStretch(1)
        layout.addStretch(1)
        self.setFixedHeight(250)

    def select(self, i):
       
        self.selection = i
        self.changeColor(i)

    def changeColor(self, i):
        if (i==1):
            self.button1.setStyleSheet("background-color : lightgreen")
            self.button2.setStyleSheet("background-color : light gray")
        elif (i==2):
            self.button1.setStyleSheet("background-color : light gray")
            self.button2.setStyleSheet("background-color : lightgreen")
        


        
       

class FirstPage(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent) 
        self.layout = QtWidgets.QVBoxLayout(self)
        self.p = parent
        
        self.firstPage()
        

    def firstPage(self):
        
        self.F = FirstChoice()
        self.S = SecondChoice()
        title = self.createLabel("Equipment Setup", 24)
        wr1Layout = self.createLabel("Select Your Apparatus", 16)
        wr2Layout = self.createLabel("Select Your Arm", 16)
        fButton = self.outButton()
        self.layout.addLayout(title)
        self.layout.addStretch()
        self.layout.addLayout(wr1Layout)
        self.layout.addWidget(self.F)
        self.layout.addStretch()
        self.layout.addLayout(wr2Layout)
        self.layout.addWidget(self.S)
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
        button1.clicked.connect(lambda:goToSecondScreen(self.p))
        button1.setFixedHeight(30)
        button1.setFixedWidth(150)
        layout1.addStretch(1)
        layout1.addWidget(button1)
        layout1.addStretch(1)
        return layout1

    


    
class MainPage(QtWidgets.QMainWindow):
    def __init__(self):
        super(QWidget, self).__init__()
        bar = self.menuBar()
        file = bar.addMenu("File")
        file.addAction("New")
        file.addAction("Save Setup")
        file.addAction("Export")
        view = bar.addMenu("View")
        self.action1 = QAction("1 Element")
        self.action2 = QAction("2 Elements")
        self.action3 = QAction("4 Elements")
        view.addAction(self.action1)
        view.addAction(self.action2)
        view.addAction(self.action3)
        self.action1.triggered.connect(lambda:self.changeGrid(1))
        self.action2.triggered.connect(lambda:self.changeGrid(2))
        self.action3.triggered.connect(lambda:self.changeGrid(4))
        self.showMaximized()
        self.f = FirstPage(self)
        self.setCentralWidget(self.f)

    def changeGrid(self, i):
        if (self.f.F.selection != 0 and self.f.S.selection != 0):
            self.setCentralWidget(Window(self.f.F.selection, self.f.S.selection, i))

    def changeWidget(self):
      
        if (self.f.F.selection != 0 and self.f.S.selection != 0):
            self.setCentralWidget(Window(self.f.F.selection, self.f.S.selection, 2))

def goToSecondScreen(p):
        p.changeWidget()



if __name__ == '__main__':
    app = QApplication(sys.argv)
    main = MainPage()
    main.show()
    sys.exit(app.exec_())