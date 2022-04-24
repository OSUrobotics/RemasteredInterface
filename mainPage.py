from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QGridLayout, QLabel, QCheckBox, QAction,  QInputDialog, QLineEdit, QFileDialog
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
        self.setFixedHeight(200)

    
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
        self.setFixedHeight(200)

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
        


class ModeSlection(QtWidgets.QWidget):
    def __init__(self):
        super(QWidget, self).__init__() 
        self.selection = 0
        layout = QtWidgets.QHBoxLayout(self)
        self.button1 = QPushButton("Live Mode", self)
        self.button1.clicked.connect(lambda:self.select(1))
        self.button1.setFixedHeight(50)
        self.button1.setFixedWidth(150)
        self.button2 = QPushButton("Import Mode", self)
        self.button2.clicked.connect(lambda:self.select(2))
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
        
        self.initFirstPage()
        

    def initFirstPage(self):
        self.F = FirstChoice()
        self.S = SecondChoice()
        self.C = ModeSlection()
        title = self.createLabel("Equipment Setup", 24)
        wr1Layout = self.createLabel("Select Your Apparatus", 16)
        wr2Layout = self.createLabel("Select Your Arm", 16)
        wr3Layout = self.createLabel("Select Data Mode", 16)
        fButton = self.outButton()
        self.layout.addLayout(title)
        self.layout.addStretch()
        self.layout.addLayout(wr1Layout)
        self.layout.addWidget(self.F)
        self.layout.addStretch()
        self.layout.addLayout(wr2Layout)
        self.layout.addWidget(self.S)
        self.layout.addStretch()
        self.layout.addLayout(wr3Layout)
        self.layout.addWidget(self.C)
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
        self.f = FirstPage(self)
        self.setCentralWidget(self.f)


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
            print(fileName)
            self.imPath = fileName
            print(self.imPath)


    def changeGrid(self, i):
        if (self.f.F.selection != 0  and self.f.S.selection != 0  and self.f.C.selection != 0 ):
            self.setCentralWidget(Window(self.f.F.selection, self.f.S.selection, self.f.C.selection, i, self))

    def changeWidget(self):
        if (self.f.F.selection != 0 and self.f.S.selection != 0 and self.f.C.selection != 0 ):
            self.setCentralWidget(Window(self.f.F.selection, self.f.S.selection, self.f.C.selection, 2, self))

    def changeMode(self, mode):
        self.f.C.selection = mode
        self.setCentralWidget(Window(self.f.F.selection, self.f.S.selection, self.f.C.selection, 2, self))

    def changeApparatus(self, mode):
        self.f.F.selection = mode
        self.setCentralWidget(Window(self.f.F.selection, self.f.S.selection, self.f.C.selection, 2, self))

    def changeArm(self, mode):
        self.f.S.selection = mode
        self.setCentralWidget(Window(self.f.F.selection, self.f.S.selection, self.f.C.selection, 2, self))
    

def goToSecondScreen(p):
        p.changeWidget()



if __name__ == '__main__':
    app = QApplication(sys.argv)
    main = MainPage()
    main.show()
    sys.exit(app.exec_())