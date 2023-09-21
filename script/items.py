import rviz
from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QComboBox, QWidget, QApplication, QPushButton, QGridLayout, QLabel, QCheckBox, QInputDialog, QLineEdit, QFileDialog
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from infrastructure_msgs.msg import DoorSensors
from sensor_msgs.msg import JointState




# Wrapper class for generating canvas in the application
class CanvasFigure(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=4, height=4, dpi=100, line=False):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super(CanvasFigure, self).__init__(self.fig)
        self.axes = self.fig.add_subplot(111)
        self.legend = None
        #For distance graph
        if line:
            self.line, = self.axes.plot([0],[0])
        #For FSR graph
        self.line_arr = []
        for i in range(12):
            self.line_arr.append(self.axes.plot([0],[0])[0])
            self.line_arr[i].set_label("FSR" + str(i + 1))


        #self.legend = self.fig.legend()


# class for openning a graph in a sep window
class DifferentWindows(QtWidgets.QMainWindow):
    def __init__(self, p, t, status = [], index = -1, num = 0 ):
        super(QWidget, self).__init__()
        self.showMaximized()
        self.setWindowTitle('Distance Graph')
        self.wrapper = Wrapper(p, status, t, index, num)
        self.setCentralWidget(self.wrapper)

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
            self.item = GraphDistance(p, index, num, p.main_page)
        if (t == 1):
            self.item = GraphFSR(p, status, index, num, p.main_page)
        if (t == 2):
            self.item = GraphImage(p, status, index, num, p.main_page)
        if (t == 3):
            self.item = RvizWidget(p, index, num, p.main_page)
        layout1.addStretch()
        layout1.addWidget(self.item)
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
        self.rolling_window_size = 5
        self.viewing_mode = "rolling"
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

        self.rolling_selection = QComboBox()
        self.rolling_selection.addItems(["1s", "5s", "10s", "30s", "All"])
        self.rolling_selection.setFixedHeight(30)
        self.rolling_selection.setFixedWidth(120)
        self.rolling_selection.setCurrentIndex(1)
        self.rolling_selection.currentIndexChanged.connect(self.rollingIndexChanged)


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
        self.canvas.legend = self.canvas.fig.legend()
        #self.layout1.addWidget(button)
        if num != 0:
            self.layout.addLayout(self.layout1)
        self.layout.addWidget(self.canvas)
        self.layout1.addWidget(self.rolling_selection)
        self.layout1.addStretch()
        self.layout1.addWidget(buttonNewWindow)
        self.layout1.addWidget(buttonBack)
        self.setLayout(self.layout)

        self.update()


        # Setup a timer to trigger the redraw by calling update_plot.
        #self.timer = QtCore.QTimer()
        #self.timer.setInterval(self.main_page.global_vars["timestep"] * 1000)
        #self.timer.timeout.connect(self.update)
        #self.timer.start()



    def openInNewWindow(self):
        self.main_page.global_vars["other_windows"].append(DifferentWindows(self.parent, 1, self.statusArray))


    def rollingIndexChanged(self, idx):
        if idx == 0:
            self.viewing_mode = "rolling"
            self.rolling_window_size = 1
        elif idx == 1:
            self.viewing_mode = "rolling"
            self.rolling_window_size = 5
        elif idx == 2:
            self.viewing_mode = "rolling"
            self.rolling_window_size = 10
        elif idx == 3:
            self.viewing_mode = "rolling"
            self.rolling_window_size = 30
        elif idx == 4:
            self.viewing_mode = "total"
            self.rolling_window_size = -1
        self.update()


    def update(self):
        if len(self.main_page.global_vars["sensor_buffer"]) < 1:
            return
        graph_data = []
        for i in range(12):
            graph_data.append([])

        names = []

        if self.viewing_mode == "rolling":
            bound = self.parent.getRollingWindowBound(self.rolling_window_size)
        elif self.viewing_mode == "total":
            bound = 0

        inner = []
        time_buffer = []
        max_val = 0
        for i in range(bound, self.main_page.global_vars["sensor_index"] + 1):
            inner = []
            time_buffer.append(self.main_page.global_vars["sensor_buffer"][i].current_time.to_sec() - self.main_page.global_vars["sensor_buffer"][0].current_time.to_sec())
            if self.statusArray[0] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr1 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr1
                graph_data[0].append(self.main_page.global_vars["sensor_buffer"][i].fsr1)
            if self.statusArray[1] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr2 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr2
                graph_data[1].append(self.main_page.global_vars["sensor_buffer"][i].fsr2)
            if self.statusArray[2] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr3 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr3
                graph_data[2].append(self.main_page.global_vars["sensor_buffer"][i].fsr3)
            if self.statusArray[3] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr4 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr4
                graph_data[3].append(self.main_page.global_vars["sensor_buffer"][i].fsr4)
            if self.statusArray[4] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr5 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr5
                graph_data[4].append(self.main_page.global_vars["sensor_buffer"][i].fsr5)
            if self.statusArray[5] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr6 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr6
                graph_data[5].append(self.main_page.global_vars["sensor_buffer"][i].fsr6)
            if self.statusArray[6] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr7 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr7
                graph_data[6].append(self.main_page.global_vars["sensor_buffer"][i].fsr7)
            if self.statusArray[7] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr8 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr8
                graph_data[7].append(self.main_page.global_vars["sensor_buffer"][i].fsr8)
            if self.statusArray[8] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr9 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr9
                graph_data[8].append(self.main_page.global_vars["sensor_buffer"][i].fsr9)
            if self.statusArray[9] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr10 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr10
                graph_data[9].append(self.main_page.global_vars["sensor_buffer"][i].fsr10)
            if self.statusArray[10] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr11 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr11
                graph_data[10].append(self.main_page.global_vars["sensor_buffer"][i].fsr11)
            if self.statusArray[11] == 1:
                if self.main_page.global_vars["sensor_buffer"][i].fsr12 > max_val:
                    max_val = self.main_page.global_vars["sensor_buffer"][i].fsr12
                graph_data[11].append(self.main_page.global_vars["sensor_buffer"][i].fsr12)
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

        plot_indices = []
        for name in names:
            plot_indices.append(int(name[3:]) - 1)
        with open('/root/infrastructure_ws/src/RemasteredInterface/script/test.txt', 'w') as f:
            f.write(str(plot_indices))

        self.canvas.legend.remove()

        self.canvas.legend = self.canvas.fig.legend(names)

        self.canvas.axes.set_ybound(lower=0)
        if self.canvas.axes.get_ybound()[1] < max_val:
            self.canvas.axes.set_ybound(upper=max_val)

        if len(time_buffer) > 0:
            self.canvas.axes.set_xbound(lower=time_buffer[0], upper=time_buffer[-1])

        for i in range(12):
            if i in plot_indices:
                self.canvas.line_arr[i].set_data(time_buffer, graph_data[i])
            else:
                self.canvas.line_arr[i].set_data([0],[0])

        self.canvas.draw()

# canvas and controls for graping Distance against Time
class GraphDistance(QtWidgets.QWidget):
    def __init__(self, p, index, num, main_page):
        super(QWidget, self).__init__()
        self.parent = p
        self.index = index
        self.main_page = main_page
        self.viewing_mode = "rolling"
        self.rolling_window_size = 5
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

        self.rolling_selection = QComboBox()
        self.rolling_selection.addItems(["1s", "5s", "10s", "30s", "All"])
        self.rolling_selection.setFixedHeight(30)
        self.rolling_selection.setFixedWidth(120)
        self.rolling_selection.setCurrentIndex(1)
        self.rolling_selection.currentIndexChanged.connect(self.rollingIndexChanged)

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
        self.canvas = CanvasFigure(self, width=5, height=4, dpi=100, line=True)
        self.canvas.fig.suptitle('Distance Graph', fontsize=14)
        #self.layout1.addWidget(button)
        if num != 0:
            self.layout.addLayout(self.layout1)
        self.layout.addWidget(self.canvas)
        self.layout1.addWidget(self.rolling_selection)
        self.layout1.addStretch()
        self.layout1.addWidget(buttonNewWindow)
        self.layout1.addWidget(buttonBack)

        self.setLayout(self.layout)

        self.update()

        # Setup a timer to trigger the redraw by calling update_plot.
        #self.timer = QtCore.QTimer()
        #self.timer.setInterval(self.main_page.global_vars["timestep"] * 1000)
        #self.timer.timeout.connect(self.update)
        #self.timer.start()

    def rollingIndexChanged(self, idx):
        if idx == 0:
            self.viewing_mode = "rolling"
            self.rolling_window_size = 1
        elif idx == 1:
            self.viewing_mode = "rolling"
            self.rolling_window_size = 5
        elif idx == 2:
            self.viewing_mode = "rolling"
            self.rolling_window_size = 10
        elif idx == 3:
            self.viewing_mode = "rolling"
            self.rolling_window_size = 30
        elif idx == 4:
            self.viewing_mode = "total"
            self.rolling_window_size = -1
        self.update()

    # update plot in either  live - 0 or recorded - 1 mode
    def update(self):
        if len(self.main_page.global_vars["sensor_buffer"]) < 1:
            return
        if not self.main_page.global_vars["live"]:
            if len(self.main_page.global_vars["sensor_buffer"]) != 0:
                buffer = []
                time_buffer = []
                if self.viewing_mode == "rolling":
                    bound = self.parent.getRollingWindowBound(self.rolling_window_size)
                elif self.viewing_mode == "total":
                    bound = 0
                min_dist = 0
                max_dist = 0
                for d in self.main_page.global_vars["sensor_buffer"][bound:self.main_page.global_vars["sensor_index"]+1]:
                    buffer.append(d.tof)
                    min_dist = min(min_dist, d.tof)
                    max_dist = max(max_dist, d.tof)
                    time_buffer.append(d.current_time.to_sec() - self.main_page.global_vars["sensor_buffer"][0].current_time.to_sec())

        else:
            buffer = []
            time_buffer = []
            if self.viewing_mode == "rolling":
                bound = self.parent.getRollingWindowBound(self.rolling_window_size)
            elif self.viewing_mode == "total":
                bound = 0
            min_dist = 0
            max_dist = 0
            for d in self.main_page.global_vars["sensor_buffer"][bound:]:
                min_dist = min(min_dist, d.tof)
                max_dist = max(max_dist, d.tof)
                buffer.append(d.tof)
                time_buffer.append(d.current_time.to_sec() - self.main_page.global_vars["sensor_buffer"][0].current_time.to_sec())

        if self.canvas.axes.get_ybound()[0] > min_dist:
            self.canvas.axes.set_ybound(lower=min_dist)
        if self.canvas.axes.get_ybound()[1] < max_dist:
            self.canvas.axes.set_ybound(upper=max_dist)
        if len(time_buffer) > 0:
            self.canvas.axes.set_xbound(lower=time_buffer[0], upper=time_buffer[-1])
        if len(buffer) > 0:
            self.canvas.line.set_data(time_buffer, buffer)
        else:
            self.canvas.line.set_data(time_buffer, buffer)
        self.canvas.draw()
        self.canvas.show()


    # open the widget in a new window
    def openInNewWindow(self):
        self.main_page.global_vars["other_windows"].append(DifferentWindows(self.parent, 0))


class GraphImage(QWidget):
    def __init__(self, p, statusArray, index, num, main_page):
        super(QWidget, self).__init__()
        self.visibleButtons = statusArray
        self.index = index
        self.parent = p
        self.main_page = main_page
        if num == 0:
            self.parent.other_windows.append(self)
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
        self.main_page.global_vars["other_windows"].append(DifferentWindows(self.parent, 2, self.visibleButtons))

    def update(self):
        pass

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
        buttonBack.clicked.connect(self.removeSelf)
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

        #if self.main_page.global_vars["arm"] is "kinova":
            #self.initializeJaco2()

    def openInNewWindow(self):
        new_window = DifferentWindows(self.parent, 3)
        self.main_page.global_vars["other_windows"].append(new_window)
        self.main_page.global_vars["rviz_instances"].append(new_window)

    def changeConfig(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fname, _ = QFileDialog.getOpenFileName(self, "Open RVIZ Config File", self.main_page.global_vars["package_dir"] + '/rviz',"All Files (*);;rviz configuration files (*.rviz *.myviz)", options=options)
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, fname )
        self.frame.load( config )

    def reset_time(self):
        manager = self.frame.getManager()
        manager.resetTime()

    def removeSelf(self):
        self.main_page.global_vars["rviz_instances"].remove(self)
        self.parent.goBackToSelection(self.index)

    def update(self):
        pass
