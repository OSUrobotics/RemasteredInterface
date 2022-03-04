from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import math
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QComboBox, QWidget, QApplication, QPushButton, QGridLayout, QLabel, QCheckBox
from PyQt5 import QtCore, QtGui
import sys
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import matplotlib.animation as animation
import random

arrayX = [1, 2, 3 , 4 , 5 , 6 , 7 , 8]
arrayY = [3, 4, 5 , 1 , 6 , 9,  4 ,6]


class CanvasFigure(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=4, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super(CanvasFigure, self).__init__(self.fig)
        self.axes = self.fig.add_subplot(111)
              


class DifferentWindows(QtWidgets.QMainWindow):
    def __init__(self, p, t, status = [], index = -1, num = 0 ):
        super(QWidget, self).__init__()
        self.showMaximized()
        self.setWindowTitle('Distance Graph')
        w = Wrapper(p, t, status, index, num)
        self.setCentralWidget(w)

class Wrapper(QtWidgets.QWidget):
    def __init__(self, p, t, status, index, num):
        super(QWidget, self).__init__()
        layout = QtWidgets.QVBoxLayout(self) 
        layout1 = QtWidgets.QHBoxLayout(self)

        self.setLayout(layout)
        layout.addStretch()
        layout.addLayout(layout1)
        layout.addStretch()
        if (t == 0):
            self.g = GraphDistance(p, index, num)
        if (t == 1):
            self.g = Graph(p, index, num)
        if (t == 2):
            print("sdfdsf")
            self.g = graphImage(p, index, status,  num)
        layout1.addStretch()
        layout1.addWidget(self.g)
        layout1.addStretch()
        

class GraphDistance(QtWidgets.QWidget):
    def __init__(self, p, index, num):
        self.parent = p
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
            self.height = 400
        
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
        n_data = 50
        self.xdata = list(range(n_data))
        self.ydata = [random.randint(0, 10) for i in range(n_data)]
        self.update_plot()
        self.show()

        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    
   


    def update_plot(self):
        self.ydata = self.ydata[1:] + [random.randint(0, 10)]
        self.canvas.axes.cla()  
        self.canvas.axes.plot(self.xdata, self.ydata, 'b')
        self.canvas.draw()


    def openInNewWindow(self):
        self.w1 = DifferentWindows(self.parent, 0)


class Graph(QtWidgets.QWidget):
    def __init__(self, p, index, num):
        self.parent = p
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
            self.height = 400
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
        n_data = 50
        self.xdata = list(range(n_data))
        self.ydata = [random.randint(0, 10) for i in range(n_data)]
        self.update_plot()
        

        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()
        


    def openInNewWindow(self):
        self.w1 = DifferentWindows(self.parent, 1)

    
    


    def update_plot(self):
        self.ydata = self.ydata[1:] + [random.randint(0, 10)]
        self.canvas.axes.cla()  
        self.canvas.axes.plot(self.xdata, self.ydata, 'r')
        self.canvas.draw()


class graphImage(QWidget):
    def __init__(self, p, index, statusArray, num):
        super(QWidget, self).__init__()
        self.visibleButtons = statusArray
        self.index = index 
        self.parent = p 
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
            self.height = 400
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
        self.your_mesh = mesh.Mesh.from_file('./stl_meshes/Main.stl')
        self.meshArray = [] 
        self.sensors = []
        self.texts = []
        
        for i in range (12):
            self.meshArray.append(mesh.Mesh.from_file('./stl_meshes/sensor' + str(i+1) + '.stl'))
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
        print("dsfdsf")
        self.w1 = DifferentWindows(self.parent, 2, self.visibleButtons)

    
class Menu(QWidget): 
    def __init__(self, ap = 1, arm = 1, mode = 1):
        self.mode = mode
        super(QWidget, self).__init__()
        self.ap = ap 
        self.arm = arm
        self.comboBox = QComboBox(self)
        self.comboBox.addItem("Mode: Live")
        self.comboBox.addItem("Mode: Recorded")

        if mode == 1:
            self.comboBox.setCurrentText("Mode: Live")
        else:
            self.comboBox.setCurrentText("Mode: Recorded")

        self.comboBox.currentIndexChanged.connect(lambda: self.change())

        filler = QLabel('', self)
        filler.resize(200, 70)
        self.setFixedHeight(650)
        self.setFixedWidth(250)
        layout = QtWidgets.QVBoxLayout(self) 
        sublayout = QtWidgets.QVBoxLayout(self) 
        self.createApLabel()
        self.createArmLabel()
        layout.addWidget(self.comboBox)
        layout.addWidget(self.appar)
        layout.addWidget(self.armL)
        
        layout.addStretch()
        
        self.buttonArray = []
        self.statusArray = []
        for i in range (12):
            self.buttonArray.append(QPushButton("FSR " + str(i+1)))
            self.statusArray.append(0)
        self.buttonArray.append(QPushButton("Distance Sensor"))
        self.statusArray.append(0)
        for i in range (13):
            layout.addWidget(self.buttonArray[i])
            layout.addStretch()
        layout.addWidget(filler)
        
    def change(self):
        content = self.comboBox.currentText()
        if content == 'Mode: Live':
            self.mode = 1 
        else:
            self.mode = 2

    def createApLabel(self):
        if (self.ap == 1):
            self.appar = QLabel('Apparatus: Drawer', self)
        elif(self.ap == 2):
            self.appar = QLabel('Apparatus: Door', self)
        elif(self.ap == 3):
            self.appar = QLabel('Apparatus: Test Bed', self)
        else: 
            self.appar = QLabel('Apparatus: Failed', self)
        self.appar.setFont(QFont('Times', 14))
        self.appar.resize(200,20)


    def createArmLabel(self):
        if (self.arm == 1):
            self.armL = QLabel('Arm: Kinova Jaco2', self)
        elif(self.arm == 2):
            self.armL = QLabel('Arm: Thor Arm', self)
        else: 
            self.armL = QLabel('Arm: Failed', self)
        self.armL.setFont(QFont('Times', 14))
        self.armL.resize(200,20)

    
class Add(QWidget):
    def __init__(self, p, index, num):
        super(QWidget, self).__init__()
        self.parent = p
        self.index = index
        if num == 1: 
            self.height = 700
            self.width = 1000
        elif num == 2:
            self.height = 550
            self.width = 600 
        else:
            self.width = 500
            self.height = 400
        self.setFixedWidth(self.width)
        self.setFixedHeight(self.height)
        self.c1 = QPushButton("Distance Graph")
        self.c2 = QPushButton("FSR Graph")
        self.c3 = QPushButton("3D Model")
        self.c4 = QPushButton("RViz Visualization")
        self.c5 = QPushButton("Back")
        self.c5.setFixedWidth(80)
        self.c5.setFixedHeight(25)
        self.c5.setStyleSheet("background-color: red;")
        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout1 = QtWidgets.QVBoxLayout()
        self.layout2 = QtWidgets.QHBoxLayout(self)
        self.setLayout(self.layout)
        self.layout.addStretch()
        self.layout.addLayout(self.layout1)
        self.button = QPushButton("", self)
        self.button.setStyleSheet("border-image: url(./src/plus.png);")
        self.button.setFixedWidth(64)
        self.button.setFixedHeight(64)
        self.button.clicked.connect(lambda:self.switchToS())
        self.c1.clicked.connect(lambda:self.parent.addDistanceGraph(self.index))
        self.c2.clicked.connect(lambda:self.parent.addFSRGraph(self.index))
        self.c3.clicked.connect(lambda:self.parent.addModel(self.index))
        self.c4.clicked.connect(lambda:self.parent.addRviz(self.index))
        self.c5.clicked.connect(lambda:self.parent.goBack(self.index))
        self.layout1.addWidget(self.button)
        self.layout.addStretch()


    def switchToS(self): 
        self.button.deleteLater()
        self.layout1.addStretch()
        self.layout1.addWidget(self.c1)
        self.layout1.addWidget(self.c2)
        self.layout1.addWidget(self.c3)
        self.layout1.addWidget(self.c4)
        self.layout1.addLayout(self.layout2)
        self.layout2.addWidget(self.c5)
        self.layout1.addStretch()


class Window(QWidget):
    def __init__(self, ap, arm, mode, num = 4):
        super(QWidget, self).__init__()
        #self.showMaximized()
        self.num = num
        self.otherWindows = [] 
        self.widgetArray = [0,0,0,0]
        self.layoutArray =  [0,0,0,0]
        self.apparatus = ap 
        self.arm = arm
        self.mode = mode 
        self.setWindowTitle('3dMesh')
        self.Main = QtWidgets.QVBoxLayout(self)
        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout2 = QtWidgets.QHBoxLayout(self)
        self.layout3 = QtWidgets.QHBoxLayout(self)
        if self.num == 4:
            self.vLayout1 = QtWidgets.QVBoxLayout(self)
            self.vLayout2 = QtWidgets.QVBoxLayout(self)
        for i in range(num):
            self.layoutArray[i] = QtWidgets.QHBoxLayout(self)
        self.setLayout(self.Main)
        self.ap = ap 
        self.arm = arm
        self.menu = Menu(ap, arm, mode)
        for i in range(num):
            self.widgetArray[i] = Add(self, i, self.num)
        self.Main.addLayout(self.layout)
        self.layout.addWidget(self.menu) 
        if self.num == 4:
            self.layout.addLayout(self.vLayout1)
            self.layout.addLayout(self.vLayout2)
            self.vLayout1.addLayout(self.layoutArray[0])
            self.vLayout1.addLayout(self.layoutArray[1])
            self.vLayout2.addLayout(self.layoutArray[2])
            self.vLayout2.addLayout(self.layoutArray[3])
        elif (self.num == 2): 
            self.layout.addLayout(self.layoutArray[0])
            self.layout.addLayout(self.layoutArray[1])
        else: 
            self.layout.addLayout(self.layoutArray[0])
        self.Main.addLayout(self.layout2)
        
        for i in range(num):
            self.layoutArray[i].addWidget(self.widgetArray[i])
        
       
       # self.initLabels()
        self.menu.buttonArray[0].clicked.connect(lambda:self.triggerAnimation(0))
        self.menu.buttonArray[1].clicked.connect(lambda:self.triggerAnimation(1))
        self.menu.buttonArray[2].clicked.connect(lambda:self.triggerAnimation(2))
        self.menu.buttonArray[3].clicked.connect(lambda:self.triggerAnimation(3))
        self.menu.buttonArray[4].clicked.connect(lambda:self.triggerAnimation(4))
        self.menu.buttonArray[5].clicked.connect(lambda:self.triggerAnimation(5))
        self.menu.buttonArray[6].clicked.connect(lambda:self.triggerAnimation(6))
        self.menu.buttonArray[7].clicked.connect(lambda:self.triggerAnimation(7))
        self.menu.buttonArray[8].clicked.connect(lambda:self.triggerAnimation(8))
        self.menu.buttonArray[9].clicked.connect(lambda:self.triggerAnimation(9))
        self.menu.buttonArray[10].clicked.connect(lambda:self.triggerAnimation(10))
        self.menu.buttonArray[11].clicked.connect(lambda:self.triggerAnimation(11))
        self.menu.buttonArray[12].clicked.connect(lambda:self.triggerAnimation(12))
        
        if self.menu.mode == 1: 
            Start = QPushButton(self)
            Start.setText("Run")
            Stop = QPushButton(self)
            Stop.setText("Stop")
            Start.setFixedHeight(30)
            Start.setFixedWidth(150)
            Stop.setFixedHeight(30)
            Stop.setFixedWidth(150)
            self.layout2.addStretch()
            self.layout2.addWidget(Start)
            self.layout2.addWidget(Stop)
            self.layout2.addStretch()
        else: 
            self.progress = QtWidgets.QProgressBar(self)
            self.progress.setMaximum(100)
            self.progress.setValue(33)
            self.layout2.addStretch()
            self.layout2.addWidget(self.progress)
            self.layout2.addStretch()
            self.Main.addLayout(self.layout3)
            self.Start = QPushButton(self)
            self.Start.setText("Play")
            self.Start.clicked.connect(lambda:self.swapButtonText())
            self.playStatus = 0 
            self.layout3.addStretch()
            self.layout3.addWidget(self.Start)
            self.layout3.addStretch()


    def swapButtonText(self):
        if self.playStatus == 0:
            self.playStatus = 1
            self.Start.setText("Stop")
        else: 
            self.playStatus = 0
            self.Start.setText("Play")

    def goBack(self, index):
        self.widgetArray[index].deleteLater()
        self.widgetArray[index] = Add(self, index, self.num)
        self.layoutArray[index].addWidget(self.widgetArray[index])


    def addDistanceGraph(self, index):
        self.widgetArray[index].deleteLater()
        self.widgetArray[index] = GraphDistance(self, index, self.num)
        self.layoutArray[index].addWidget(self.widgetArray[index])


    def addFSRGraph(self, index):
        self.widgetArray[index].deleteLater()
        self.widgetArray[index] = Graph(self, index, self.num)
        self.layoutArray[index].addWidget(self.widgetArray[index])


    def addRviz(self, index):
        pass


    def addModel(self, index):
        self.widgetArray[index].deleteLater()
        self.widgetArray[index] = graphImage(self, index, self.menu.statusArray, self.num)
        self.layoutArray[index].addWidget(self.widgetArray[index])


    def goBackToSelection(self, index):
        self.widgetArray[index].deleteLater()
        self.widgetArray[index] = Add(self, index, self.num)
        self.layoutArray[index].addWidget(self.widgetArray[index])


    def triggerAnimation(self, index):
        if index == 12: 
            if self.menu.statusArray[index] == 0:   
                self.menu.buttonArray[index].setStyleSheet("background-color : lightgreen")
                self.menu.statusArray[index] = 1
            else: 
                self.menu.statusArray[index] = 0
                self.menu.buttonArray[index].setStyleSheet("background-color : light gray")
            return 

        
        if self.menu.statusArray[index] == 0:
            for i in range(len(self.widgetArray)):
                if isinstance(self.widgetArray[i], graphImage):
                    self.widgetArray[i].showMesh(index) 
            self.menu.statusArray[index] = 1
            
            self.menu.buttonArray[index].setStyleSheet("background-color : lightgreen")
        else: 
            for i in range(len(self.widgetArray)):
                if isinstance(self.widgetArray[i], graphImage):
                    self.widgetArray[i].hideMesh(index) 
            self.menu.buttonArray[index].setStyleSheet("background-color : light gray")
            self.menu.statusArray[index] = 0
        for i in range(len(self.widgetArray)):
            if isinstance(self.widgetArray[i], graphImage): 
                self.widgetArray[i].setvisibleButtons(self.menu.statusArray)


    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self.image.showPoly()
        elif event.key() == Qt.Key_Enter:
            self.image.clear()

  
     
    
  
# driver code

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main = Window(1,1)
    main.resize(860, 640)
    main.show()
    sys.exit(app.exec_())

