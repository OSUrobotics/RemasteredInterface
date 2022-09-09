import sys
import os
import atexit
import multiprocessing
import rospy
import rosnode
import rosbag
import roslaunch
import rviz
from datetime import datetime
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QComboBox, QWidget, QApplication, QPushButton, QGridLayout, QLabel, QCheckBox, QInputDialog, QLineEdit, QFileDialog, QStackedWidget, QMessageBox
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from std_msgs.msg import Header
from infrastructure_msgs.msg import DoorSensors
from sensor_msgs.msg import JointState
from items import *

# kill other threads when on exit
def clean():
    children = multiprocessing.active_children()
    for child_thread in children:
        child_thread.terminate()
#    global pid
#    os.kill(pid, signal.SIGKILL)


atexit.register(clean)




class Menu(QWidget):
    def __init__(self, ap, arm, live, parent = None, main_page = None):
        self.parent = parent
        self.live = live
        self.main_page = main_page
        super(QWidget, self).__init__()
        self.ap = ap
        self.arm = arm
        self.comboBox = QComboBox(self)
        self.comboBox.addItem("Mode: Live")
        self.comboBox.addItem("Mode: Recorded")

        if live:
            self.comboBox.setCurrentIndex(0)
        else:
            self.comboBox.setCurrentIndex(1)

        self.comboBox.currentIndexChanged.connect(self.change)

        filler = QLabel('', self)
        filler.resize(200, 70)
        self.setFixedHeight(650)
        self.setFixedWidth(250)
        layout = QtWidgets.QVBoxLayout(self)
        sublayout = QtWidgets.QVBoxLayout(self)
        self.createApLabel()
        self.createArmLabel()
        layout.addWidget(self.comboBox)
        layout.addWidget(self.ALabel)
        layout.addWidget(self.Arm)

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

        #Temporary workaround for lack of topic remapping options
        self.topicBox = QLineEdit(self)
        self.topicBox.textChanged.connect(self.sensorTopicChange)
        self.topicBox.setText(self.main_page.global_vars["sensor_data_topic"])
        self.topicLabel = QLabel(self)
        self.topicLabel.setText("Sensor topic:")
        #Temporary workaround for lack of topic remapping options
        self.jointTopicBox = QLineEdit(self)
        self.jointTopicBox.textChanged.connect(self.jointsTopicChange)
        self.jointTopicBox.setText(self.main_page.global_vars["joint_state_topic"])
        self.jointTopicLabel = QLabel(self)
        self.jointTopicLabel.setText("Joint states topic:")

        layout.addWidget(self.topicLabel)
        layout.addWidget(self.topicBox)
        layout.addWidget(self.jointTopicLabel)
        layout.addWidget(self.jointTopicBox)

        self.refreshButton = QPushButton("Refresh Displayed Topic")
        self.refreshButton.clicked.connect(self.parent.refreshTopic)
        layout.addWidget(self.refreshButton)

        self.saveConfigButton = QPushButton("Save Configuration")
        self.saveConfigButton.clicked.connect(self.main_page.saveConfig)
        layout.addWidget(self.saveConfigButton)

        self.loadRosbagButton = QPushButton("Load Rosbag")
        self.loadRosbagButton.clicked.connect(self.parent.loadRosbag)
        layout.addWidget(self.loadRosbagButton)

    def sensorTopicChange(self, topic):
        self.main_page.global_vars["sensor_data_topic"] = topic

    def jointsTopicChange(self, topic):
        self.main_page.global_vars["joint_state_topic"] = topic

    def change(self, index):
        if index == 0:
            self.live = True
            self.main_page.global_vars["live"] = True
            self.parent.timer.start()
            self.parent.controlWidgetStack.setCurrentIndex(0)
        else:
            self.live = False
            self.main_page.global_vars["live"] = False
            self.parent.timer.stop()
            self.parent.controlWidgetStack.setCurrentIndex(1)


    def createApLabel(self):
        self.ALabel = QComboBox(self)
        self.ALabel.addItem("Apparatus: Drawer")
        self.ALabel.addItem("Apparatus: Door")
        self.ALabel.addItem("Apparatus: Test Bed")
        if self.ap == 1:
            self.ALabel.setCurrentText("Apparatus: Drawer")
        elif self.ap == 2:
            self.ALabel.setCurrentText("Apparatus: Door")
        else:
            self.ALabel.setCurrentText("Apparatus: Test Bed")
        self.ALabel.currentIndexChanged.connect(self.changeApparatus)


    def changeApparatus(self, index):
        self.main_page.changeApparatus(index)

    def changeArm(self, index):
        self.main_page.changeArm(index)


    def createArmLabel(self):
        self.Arm = QComboBox(self)
        self.Arm.addItem("Arm: Kinova Jaco2")
        self.Arm.addItem("Arm: Thor Arm")
        if self.arm == 1:
            self.Arm.setCurrentText("Arm: Kinova Jaco2")
        else:
            self.Arm.setCurrentText("Arm: Thor Arm")
        self.Arm.currentIndexChanged.connect(self.changeArm)



class Add(QWidget):
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
        else:
            self.width = 500
            self.height = 350
        self.setFixedWidth(self.width)
        self.setFixedHeight(self.height)
        self.c1 = QPushButton("Distance Graph")
        self.c2 = QPushButton("FSR Graph")
        self.c3 = QPushButton("3D Model")
        self.c4 = QPushButton("RViz Visualization")
        self.c6 = QPushButton("Select Test Item")
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
        self.button.setStyleSheet("border-image: url(" + self.main_page.global_vars["package_dir"] + "/src/plus.png);")
        self.button.setFixedWidth(64)
        self.button.setFixedHeight(64)
        self.button.clicked.connect(lambda:self.switchToS())
        self.c1.clicked.connect(lambda:self.parent.addDistanceGraph(self.index))
        self.c2.clicked.connect(lambda:self.parent.addFSRGraph(self.index))
        self.c3.clicked.connect(lambda:self.parent.addModel(self.index))
        self.c4.clicked.connect(lambda:self.parent.addRviz(self.index))
        self.c5.clicked.connect(lambda:self.parent.goBack(self.index))
        self.c6.clicked.connect(lambda:self.parent.addItems(self.index))
        self.layout1.addWidget(self.button)
        self.layout.addStretch()


    def switchToS(self):
        self.button.deleteLater()
        self.layout1.addStretch()
        self.layout1.addWidget(self.c1)
        self.layout1.addWidget(self.c2)
        self.layout1.addWidget(self.c3)
        self.layout1.addWidget(self.c4)
        if self.parent.apparatus == 3:
            self.layout1.addWidget(self.c6)
        self.layout1.addLayout(self.layout2)

        self.layout2.addWidget(self.c5)
        self.layout1.addStretch()


# all possible objects on the testbed
class ObjectChoice(QtWidgets.QWidget):
    def __init__(self, p):
        super(QWidget, self).__init__()
        self.parent = p
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout1 =  QtWidgets.QHBoxLayout(self)
        self.layout2 =  QtWidgets.QHBoxLayout(self)
        self.setLayout(self.layout)
        self.layout.addLayout(self.layout1)
        self.layout.addLayout(self.layout2)
        self.initItems()
        self.layout1.addWidget(self.current_items[0])
        self.layout1.addWidget(self.current_items[1])
        self.layout2.addWidget(self.current_items[2])
        self.layout2.addWidget(self.current_items[3])


    def initItems(self):
        b1 = QPushButton("objectSelection")
        b1.setFixedWidth(150)
        b1.setFixedHeight(150)
        b1.setText("Object 1")
        b1.setStyleSheet("background-color: white;")
        b2 = QPushButton("objectSelection")
        b2.setFixedWidth(150)
        b2.setFixedHeight(150)
        b2.setText("Object 1")
        b2.setStyleSheet("background-color: white;")
        b3 = QPushButton("objectSelection")
        b3.setFixedWidth(150)
        b3.setFixedHeight(150)
        b3.setText("Object 3")
        b3.setStyleSheet("background-color: white;")
        b4 = QPushButton("objectSelection")
        b4.setFixedWidth(150)
        b4.setFixedHeight(150)
        b4.setText("Object 4")
        b4.setStyleSheet("background-color: white;")
        self.current_items = []
        self.current_items.append(b1)
        self.current_items.append(b2)
        self.current_items.append(b3)
        self.current_items.append(b4)


        b1.clicked.connect(lambda:self.changeSelection(b1))
        b2.clicked.connect(lambda:self.changeSelection(b2))
        b3.clicked.connect(lambda:self.changeSelection(b3))
        b4.clicked.connect(lambda:self.changeSelection(b4))



    def changeSelection(self, button):
        for i in self.current_items:
            i.setStyleSheet("background-color: white;")
        button.setStyleSheet("background-color: lightgreen;")


class Window(QWidget):
    def __init__(self, ap, arm, mode, num = 4, parent = None, main_page = None):
        super(QWidget, self).__init__()
        self.main_page = main_page
        self.started = 0
        #self.showMaximized()
        self.bag  = None
        self.bag_data = []
        self.parent = parent
        self.max_value = 100
        self.current_value = 0
        self.num = num
        self.other_windows = []
        self.widget_array = [0,0,0,0]
        self.current_items = []
        self.layout_array =  [0,0,0,0]
        self.apparatus = ap
        self.arm = arm
        self.last_update = rospy.Time.now()

        if mode == 1:
            self.main_page.global_vars["live"] = True
        else:
            self.main_page.global_vars["live"] = False

        self.mode = mode
        self.setWindowTitle('3D Mesh')
        self.Main = QtWidgets.QVBoxLayout(self)
        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout2 = QtWidgets.QHBoxLayout(self)
        self.layout3 = QtWidgets.QHBoxLayout(self)
        if self.num == 4:
            self.vLayout1 = QtWidgets.QVBoxLayout(self)
            self.vLayout2 = QtWidgets.QVBoxLayout(self)
        for i in range(num):
            self.layout_array[i] = QtWidgets.QHBoxLayout(self)
        self.setLayout(self.Main)
        self.menu = Menu(self.main_page.global_vars["apparatus"], self.main_page.global_vars["arm"], self.main_page.global_vars["live"], self, self.main_page)
        for i in range(num):
            self.widget_array[i] = Add(self, i, self.num, self.main_page)
        self.Main.addLayout(self.layout)
        self.layout.addWidget(self.menu)
        if self.num == 4:
            self.layout.addLayout(self.vLayout1)
            self.layout.addLayout(self.vLayout2)
            self.vLayout1.addLayout(self.layout_array[0])
            self.vLayout1.addLayout(self.layout_array[1])
            self.vLayout2.addLayout(self.layout_array[2])
            self.vLayout2.addLayout(self.layout_array[3])
        elif (self.num == 2):
            self.layout.addLayout(self.layout_array[0])
            self.layout.addLayout(self.layout_array[1])
        else:
            self.layout.addLayout(self.layout_array[0])
        self.Main.addLayout(self.layout2)

        for i in range(num):
            self.layout_array[i].addWidget(self.widget_array[i])


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

        self.controlWidgetStack = QStackedWidget(self)

        Start = QPushButton(self)
        Start.setText("Record")
        Start.clicked.connect(lambda:self.startReading())
        Stop = QPushButton(self)
        Stop.setText("Save")
        Stop.clicked.connect(lambda:self.stopReading())
        Start.setFixedHeight(30)
        Start.setFixedWidth(150)
        Stop.setFixedHeight(30)
        Stop.setFixedWidth(150)

        controlLayout1 = QtWidgets.QHBoxLayout()
        controlLayout1.addStretch()
        controlLayout1.addWidget(Start)
        controlLayout1.addWidget(Stop)
        controlLayout1.addStretch()
        controlWidget1 = QWidget()
        controlWidget1.setLayout(controlLayout1)

        self.controlWidgetStack.addWidget(controlWidget1)

        controlWidgetLayout2 = QtWidgets.QHBoxLayout()
        controlWidgetLayout3 = QtWidgets.QHBoxLayout()

        controlWidget2 = QWidget()

        self.progress = QtWidgets.QProgressBar(self)
        self.progress.setMaximum(self.main_page.global_vars["bag_end_time"])
        self.progress.setFormat("%v" + "/" + str(round(self.main_page.global_vars["bag_end_time"])) + "s")
        self.progress.setValue(self.main_page.global_vars["current_time"])
        controlWidgetLayout2.addStretch()
        controlWidgetLayout2.addWidget(self.progress)
        controlWidgetLayout2.addStretch()
        self.Start = QPushButton(self)
        self.Start.setText("Play")
        self.Start.clicked.connect(lambda:self.swapButtonText())
        self.Left = QPushButton(self)
        self.Left.setText("<<")
        self.Left.clicked.connect(lambda:self.decreasebyFive())
        self.Right = QPushButton(self)
        self.Right.setText(">>")
        self.Right.clicked.connect(lambda:self.increaseByFive())
        self.playStatus = 0
        controlWidgetLayout3.addStretch()
        controlWidgetLayout3.addWidget(self.Left)
        controlWidgetLayout3.addWidget(self.Start)
        controlWidgetLayout3.addWidget(self.Right)
        controlWidgetLayout3.addStretch()

        controlWidgetLayout4 = QtWidgets.QVBoxLayout()

        controlWidgetLayout4.addLayout(controlWidgetLayout2)
        controlWidgetLayout4.addLayout(controlWidgetLayout3)

        controlWidget2.setLayout(controlWidgetLayout4)

        self.controlWidgetStack.addWidget(controlWidget2)

        controlLayout = QtWidgets.QVBoxLayout()
        controlLayout.addWidget(self.controlWidgetStack)

        self.Main.addLayout(controlLayout)

        self.timer = QtCore.QTimer()
        self.timer.setInterval(self.main_page.global_vars["timestep"] * 1000) #have to convert timestep to milliseconds
        self.timer.timeout.connect(self.startTimer)

        self.createSensorSubscriber()
        self.createJointsSubscriber()
        self.initializeJaco2()

        if self.main_page.global_vars["live"]:
            self.menu.change(0)
        else:
            self.menu.change(1)

    def startReading(self):
        if self.main_page.global_vars["sensor_subscriber"] != None:
            if not self.main_page.global_vars["sensor_data_topic"] == self.main_page.global_vars["sensor_subscriber"].resolved_name:
                self.main_page.global_vars["sensor_subscriber"].unregister()
                self.createSensorSubscriber()
        else:
            self.createSensorSubscriber()

        if self.main_page.global_vars["joints_subscriber"] != None:
            if not self.main_page.global_vars["joint_state_topic"] == self.main_page.global_vars["joints_subscriber"].resolved_name:
                self.main_page.global_vars["joints_subscriber"].unregister()
                self.createJointsSubscriber()
        else:
            self.createJointsSubscriber()

    def stopReading(self):
        #self.started = 0
        #print(bag_queue)

        datetime_string = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        default_filename = "infrastructure_trial_" + datetime_string + ".bag"
        default_filepath = os.getcwd() + "/" + default_filename

        export_filename, _ = QFileDialog.getSaveFileName(self, "Save Recorded Rosbag", default_filepath, "Rosbag files (*.bag)")

        self.bag = rosbag.Bag(export_filename, 'w')
        while not self.main_page.global_vars["bag_queue"].empty():
            sensor_data = self.main_page.global_vars["bag_queue"].get()
            self.writeSensorDataToBag(sensor_data)

        self.bag.close()

    def republishJoints(self, data):
        old_header = data.header
        new_header = Header()
        new_header.stamp = rospy.Time.now()
        data.header = new_header
        self.main_page.global_vars["joints_publisher"].publish(data)
        data.header = old_header

    def onRead(self, data):
        if self.main_page.global_vars["live"]:
            #new_header = Header()
            #new_header.stamp = rospy.Time.now()
            #data.header = new_header
            data.current_time = rospy.Time.now()
            self.main_page.global_vars["sensor_publisher"].publish(data)
            self.main_page.global_vars["queue"].put(data)
            self.main_page.global_vars["distance_queue"].put(data)
            self.main_page.global_vars["bag_queue"].put(data)
            self.main_page.global_vars["sensor_buffer"].append(data)

    def writeJointsDataToBag(self, data):
        self.bag.write(self.main_page.global_vars["joint_state_topic"], data, data.header.stamp)

    def writeSensorDataToBag(self, data):
        msg = DoorSensors()
        msg.current_time = data.current_time
        msg.tof  = data.tof
        msg.fsr1 = data.fsr1
        msg.fsr2 = data.fsr2
        msg.fsr3 = data.fsr3
        msg.fsr4 = data.fsr4
        msg.fsr5 = data.fsr5
        msg.fsr6 = data.fsr6
        msg.fsr7 = data.fsr7
        msg.fsr8 = data.fsr8
        msg.fsr9 = data.fsr9
        msg.fsr10 = data.fsr10
        msg.fsr11 = data.fsr11
        msg.fsr12 = data.fsr12
        msg.fsr_contact_1
        msg.fsr_contact_2
        self.bag.write(self.main_page.global_vars["sensor_data_topic"],  msg, msg.current_time)

    def createSensorSubscriber(self):
        if len(self.main_page.global_vars["sensor_data_topic"]) > 0 and self.main_page.global_vars["sensor_data_topic"][0] == '/':
            topic = self.main_page.global_vars["sensor_data_topic"][1:]
        else:
            topic = self.main_page.global_vars["sensor_data_topic"]
        if self.main_page.global_vars["sensor_subscriber"] != None:
            self.main_page.global_vars["sensor_subscriber"].unregister()
            self.main_page.global_vars["sensor_subscriber"] = None
        self.main_page.global_vars["sensor_subscriber"] = rospy.Subscriber(topic, DoorSensors, self.onRead)

    def createJointsSubscriber(self):
        if len(self.main_page.global_vars["joint_state_topic"]) > 0 and self.main_page.global_vars["joint_state_topic"][0] == '/':
            topic = self.main_page.global_vars["joint_state_topic"][1:]
        else:
            topic = self.main_page.global_vars["joint_state_topic"]
        if self.main_page.global_vars["joints_subscriber"] != None:
            self.main_page.global_vars["joints_subscriber"].unregister()
            self.main_page.global_vars["joints_subscriber"] = None
        self.main_page.global_vars["joints_subscriber"] = rospy.Subscriber(topic, JointState, self.liveJointHandling)

    def liveJointHandling(self, data):
        if self.main_page.global_vars["live"]:
            self.republishJoints(data)

    def initializeJaco2(self):
        if self.main_page.global_vars["robot_state_publisher"] != None:
            return
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.main_page.global_vars["robot_state_publisher"] = roslaunch.parent.ROSLaunchParent(uuid, [self.main_page.global_vars["package_dir"] + "/launch/display_jaco_2.launch"])
        self.main_page.global_vars["robot_state_publisher"].start()

    def increaseByFive(self):
        #self.main_page.resetRvizTimes()
        increase = 5
        self.main_page.global_vars["current_time"] = self.main_page.global_vars["current_time"] + increase
        if self.main_page.global_vars["current_time"] > self.main_page.global_vars["bag_end_time"]:
            self.main_page.global_vars["current_time"] = self.main_page.global_vars["bag_end_time"]
        self.progress.setValue(self.main_page.global_vars["current_time"])
        self.progress.setFormat("%v" + "/" + str(round(self.main_page.global_vars["bag_end_time"])) + "s")
        #Doubled on purpose to make sure that changes go through when paused. Kind of a stopgap solution
        self.updateItems()
        self.updateItems()

    def decreasebyFive(self):
        #self.main_page.resetRvizTimes()
        decrease = 5
        self.main_page.global_vars["current_time"] = self.main_page.global_vars["current_time"] - decrease
        if (self.main_page.global_vars["current_time"] < 0):
            self.main_page.global_vars["current_time"] = 0
        self.progress.setValue(self.main_page.global_vars["current_time"])
        self.progress.setFormat("%v" + "/" + str(round(self.main_page.global_vars["bag_end_time"])) + "s")
        #Doubled on purpose to make sure that changes go through when paused. Kind of a stopgap solution
        self.updateItems()
        self.updateItems()

    def resetTimer(self):
        self.main_page.global_vars["current_time"] = 0

    def startTimer(self):
        if not self.main_page.global_vars["live"]:
            if (self.main_page.global_vars["current_time"] >= self.main_page.global_vars["bag_end_time"]):
                self.swapButtonText()
            self.progress.setValue(self.main_page.global_vars["current_time"])
            self.progress.setFormat("%v" + "/" + str(round(self.main_page.global_vars["bag_end_time"])) + "s")
            self.main_page.global_vars["current_time"] += self.main_page.global_vars["timestep"]
        self.updateItems()

    def refreshTopic(self):
        if self.main_page.global_vars["sensor_subscriber"] != None:
            if not self.main_page.global_vars["sensor_data_topic"] == self.main_page.global_vars["sensor_subscriber"].resolved_name:
                self.main_page.global_vars["sensor_subscriber"].unregister()
                self.createSensorSubscriber()
        else:
            self.createSensorSubscriber()

        if self.main_page.global_vars["joints_subscriber"] != None:
            if not self.main_page.global_vars["joint_state_topic"] == self.main_page.global_vars["joints_subscriber"].resolved_name:
                self.main_page.global_vars["joints_subscriber"].unregister()
                self.createJointsSubscriber()
        else:
            self.createJointsSubscriber()

        if self.main_page.global_vars["rosbag_file"] and os.path.exists(self.main_page.global_vars["rosbag_file"]):
            try:
                self.bag  = rosbag.Bag(self.main_page.global_vars["rosbag_file"], 'r')
            except rosbag.bag.ROSBagException:
                error = QMessageBox()
                error.setWindowTitle("Error")
                error.setIcon(QMessageBox.Critical)
                error.setText("Error: Selected file does not appear to be a Rosbag file!")
                error.exec_()
                return



            self.t_start = rospy.Time(self.bag.get_start_time())
            t_end  = rospy.Time(self.bag.get_end_time())

            self.main_page.global_vars["bag_end_time"] = (t_end - self.t_start).to_sec()
            self.bag_data = []
            self.jointMsgs = []

            self.main_page.global_vars["sensor_buffer"] = []
            self.main_page.global_vars["joints_buffer"] = []

            self.main_page.global_vars["sensor_index"] = 0
            self.main_page.global_vars["joints_index"] = 0

            if len(self.main_page.global_vars["sensor_data_topic"]) > 0 and not self.main_page.global_vars["sensor_data_topic"][0] == '/':
                sensor_topic = '/' + self.main_page.global_vars["sensor_data_topic"]
            else:
                sensor_topic = self.main_page.global_vars["sensor_data_topic"]

            if len(self.main_page.global_vars["joint_state_topic"]) > 0 and not self.main_page.global_vars["joint_state_topic"][0] == '/':
                joints_topic = '/' + self.main_page.global_vars["joint_state_topic"]
            else:
                joints_topic = self.main_page.global_vars["joint_state_topic"]

            for topic, msg, t in self.bag.read_messages():
                if topic == sensor_topic:
                    self.bag_data.append([msg, t])
                    self.main_page.global_vars["sensor_buffer"].append(msg)
                    #print("Added message to sensor_buffer")
                if topic == joints_topic:
                    self.jointMsgs.append([msg, t])
                    self.main_page.global_vars["joints_buffer"].append(msg)
                    #print("Added message to joints_buffer")

            self.main_page.global_vars["current_time"] = 0
            self.progress.setMaximum(self.main_page.global_vars["bag_end_time"])
            self.progress.setValue(0)

    def openFileNameDialog(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        self.main_page.global_vars["rosbag_file"], _ = QFileDialog.getOpenFileName(self,"Select Bag File", "","All Files (*);;Rosbag Files (*.bag)", options=options)
        if self.main_page.global_vars["rosbag_file"]:
            #self.bag  = rosbag.Bag(os.path.dirname(script_path) + '/rosbag_records/test.bag', 'r')
            self.bag  = rosbag.Bag(self.main_page.global_vars["rosbag_file"], 'r')
            self.t_start = rospy.Time(self.bag.get_start_time())
            t_end   = rospy.Time(self.bag.get_end_time())
            self.main_page.global_vars["bag_end_time"] = (t_end - self.t_start).to_sec()
            self.bag_data = []
            if len(self.main_page.global_vars["sensor_data_topic"]) > 0 and not self.main_page.global_vars["sensor_data_topic"][0] == '/':
                sensor_topic = '/' + self.main_page.global_vars["sensor_data_topic"]
            else:
                sensor_topic = self.main_page.global_vars["sensor_data_topic"]

            if len(self.main_page.global_vars["joint_state_topic"]) > 0 and not self.main_page.global_vars["joint_state_topic"][0] == '/':
                joints_topic = '/' + self.main_page.global_vars["joint_state_topic"]
            else:
                joints_topic = self.main_page.global_vars["joint_state_topic"]

            for topic, msg, t in self.bag.read_messages():
                if topic == sensor_topic:
                    self.bag_data.append([msg, t])
                    self.main_page.global_vars["sensor_buffer"].append(msg)
                    #print("Added message to sensor_buffer")
                if topic == joints_topic:
                    self.jointMsgs.append([msg, t])
                    self.main_page.global_vars["joints_buffer"].append(msg)
                    #print("Added message to joints_buffer")


            self.progress.setMaximum(self.main_page.global_vars["bag_end_time"])
            self.progress.setValue(0)
            return 1
        return 0

    def loadRosbag(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        self.main_page.global_vars["rosbag_file"], _ = QFileDialog.getOpenFileName(self,"Select Bag File", os.getcwd(),"All Files (*);;Rosbag files (*.bag)", options=options)

        if self.main_page.global_vars["rosbag_file"] and os.path.exists(self.main_page.global_vars["rosbag_file"]):
            try:
                self.bag  = rosbag.Bag(self.main_page.global_vars["rosbag_file"], 'r')
            except rosbag.bag.ROSBagException:
                error = QMessageBox()
                error.setWindowTitle("Error")
                error.setIcon(QMessageBox.Critical)
                error.setText("Error: Selected file does not appear to be a Rosbag file!")
                error.exec_()
                return



            self.t_start = rospy.Time(self.bag.get_start_time())
            t_end  = rospy.Time(self.bag.get_end_time())

            self.main_page.global_vars["bag_end_time"] = (t_end - self.t_start).to_sec()
            self.bag_data = []
            self.jointMsgs = []

            self.main_page.global_vars["sensor_buffer"] = []
            self.main_page.global_vars["joints_buffer"] = []

            self.main_page.global_vars["sensor_index"] = 0
            self.main_page.global_vars["joints_index"] = 0

            if len(self.main_page.global_vars["sensor_data_topic"]) > 0 and not self.main_page.global_vars["sensor_data_topic"][0] == '/':
                sensor_topic = '/' + self.main_page.global_vars["sensor_data_topic"]
            else:
                sensor_topic = self.main_page.global_vars["sensor_data_topic"]

            if len(self.main_page.global_vars["joint_state_topic"]) > 0 and not self.main_page.global_vars["joint_state_topic"][0] == '/':
                joints_topic = '/' + self.main_page.global_vars["joint_state_topic"]
            else:
                joints_topic = self.main_page.global_vars["joint_state_topic"]

            for topic, msg, t in self.bag.read_messages():
                if topic == sensor_topic:
                    self.bag_data.append([msg, t])
                    self.main_page.global_vars["sensor_buffer"].append(msg)
                    #print("Added message to sensor_buffer")
                if topic == joints_topic:
                    self.jointMsgs.append([msg, t])
                    self.main_page.global_vars["joints_buffer"].append(msg)
                    #print("Added message to joints_buffer")

            self.main_page.global_vars["current_time"] = 0
            self.progress.setMaximum(self.main_page.global_vars["bag_end_time"])
            self.progress.setValue(0)

    def swapButtonText(self):
        #print("Hit swap button. Play status is " + str(self.playStatus))
        if self.bag == None:
            warning = QMessageBox()
            warning.setWindowTitle("Missing Rosbag Warning")
            warning.setIcon(QMessageBox.Warning)
            ret = warning.question(self, "", "Warning: No Rosbag loaded. Load Rosbag now?", QMessageBox.Yes | QMessageBox.No)
            if ret == QMessageBox.Yes:
                self.loadRosbag()

        elif self.playStatus == 0:
            self.playStatus = 1
            self.timer.start()
            self.Start.setText("Pause")

        elif self.playStatus == 1:
            self.playStatus = 0
            self.timer.stop()
            self.Start.setText("Play")


    def goBack(self, index):
        self.widget_array[index].deleteLater()
        self.widget_array[index] = Add(self, index, self.num, self.main_page)
        self.layout_array[index].addWidget(self.widget_array[index])

    def addDistanceGraph(self, index):
        self.widget_array[index].deleteLater()
        self.widget_array[index] = GraphDistance(self, index, self.num, self.main_page)
        self.layout_array[index].addWidget(self.widget_array[index])

    def addFSRGraph(self, index):
        self.widget_array[index].deleteLater()
        self.widget_array[index] = GraphFSR(self, self.menu.statusArray, index, self.num, self.main_page)
        self.layout_array[index].addWidget(self.widget_array[index])

    def addRviz(self, index):
        self.widget_array[index].deleteLater()
        self.widget_array[index] = RvizWidget(self, index, self.num, self.main_page)
        self.layout_array[index].addWidget(self.widget_array[index])
        self.main_page.global_vars["rviz_instances"].append(self.widget_array[index])

    def addItems(self, index):
        self.widget_array[index].deleteLater()
        for i in range (len(self.widget_array)):
            if isinstance(self.widget_array[i], Items):
                self.widget_array[i].deleteLater()
                self.widget_array[i] = Add(self, i, self.num, self.main_page)
                self.layout_array[i].addWidget(self.widget_array[i])
        self.widget_array[index] = Items(self, self.menu.statusArray, index, self.num)
        self.layout_array[index].addWidget(self.widget_array[index])

    def addModel(self, index):
        self.widget_array[index].deleteLater()
        self.widget_array[index] = GraphImage(self, self.menu.statusArray, index, self.num, self.main_page)
        self.layout_array[index].addWidget(self.widget_array[index])

    def goBackToSelection(self, index):
        self.widget_array[index].deleteLater()
        self.widget_array[index] = Add(self, index, self.num, self.main_page)
        self.layout_array[index].addWidget(self.widget_array[index])

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
            for i in range(len(self.widget_array)):
                if isinstance(self.widget_array[i], GraphImage):
                    self.widget_array[i].showMesh(index)
            self.menu.statusArray[index] = 1
            self.menu.buttonArray[index].setStyleSheet("background-color : lightgreen")

            self.main_page.global_vars["sensors_enabled"].append(index)


        else:
            for i in range(len(self.widget_array)):
                if isinstance(self.widget_array[i], GraphImage):
                    self.widget_array[i].hideMesh(index)
            self.menu.buttonArray[index].setStyleSheet("background-color : light gray")
            self.menu.statusArray[index] = 0
            self.main_page.global_vars["sensors_enabled"].remove(index)

        for i in range(len(self.widget_array)):
            if isinstance(self.widget_array[i], GraphImage):
                self.widget_array[i].setvisibleButtons(self.menu.statusArray)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self.image.showPoly()
        elif event.key() == Qt.Key_Enter:
            self.image.clear()

    def updateItems(self):
        for item in self.widget_array:
            if type(item) != type(int()):
                item.update()
        for window in self.main_page.global_vars["other_windows"]:
            window.wrapper.item.update()

        if not self.main_page.global_vars["live"]:
            sensor_index = self.main_page.global_vars["sensor_index"]
            joints_index = self.main_page.global_vars["joints_index"]
            if len(self.main_page.global_vars["sensor_buffer"]) > 0:
                while self.main_page.global_vars["sensor_buffer"][sensor_index].current_time.to_sec() - self.main_page.global_vars["sensor_buffer"][0].current_time.to_sec() < self.main_page.global_vars["current_time"] and sensor_index < len(self.main_page.global_vars["sensor_buffer"]) - 1:
                    sensor_index += 1
                while self.main_page.global_vars["sensor_buffer"][sensor_index].current_time.to_sec() - self.main_page.global_vars["sensor_buffer"][0].current_time.to_sec() > self.main_page.global_vars["current_time"] and sensor_index > 0:
                    sensor_index -= 1
                self.main_page.global_vars["sensor_index"] = sensor_index
            if len(self.main_page.global_vars["joints_buffer"]) > 0:
                while self.main_page.global_vars["joints_buffer"][joints_index].header.stamp.to_sec() - self.main_page.global_vars["joints_buffer"][0].header.stamp.to_sec() < self.main_page.global_vars["current_time"] and joints_index < len(self.main_page.global_vars["joints_buffer"]) - 1:
                    joints_index += 1
                while self.main_page.global_vars["joints_buffer"][joints_index].header.stamp.to_sec() - self.main_page.global_vars["joints_buffer"][0].header.stamp.to_sec() > self.main_page.global_vars["current_time"] and joints_index > 0:
                    joints_index -= 1
                self.main_page.global_vars["joints_index"] = joints_index
                self.republishJoints(self.main_page.global_vars["joints_buffer"][joints_index])

        else:
            self.main_page.global_vars["sensor_index"] = len(self.main_page.global_vars["sensor_buffer"]) - 1
            self.main_page.global_vars["joints_index"] = len(self.main_page.global_vars["joints_buffer"]) - 1

    def getRollingWindowBound(self, seconds):
        if len(self.main_page.global_vars["sensor_buffer"]) > 0:
            if not self.main_page.global_vars["live"]:
                time_var = self.main_page.global_vars["current_time"]
                if self.main_page.global_vars["current_time"] < seconds:
                    return 0
            else:
                time_var = rospy.Time.now().to_sec() - self.main_page.global_vars["sensor_buffer"][0].current_time.to_sec()

            sensor_index = 0
            while self.main_page.global_vars["sensor_buffer"][sensor_index].current_time.to_sec() - self.main_page.global_vars["sensor_buffer"][0].current_time.to_sec() < time_var - seconds and sensor_index < len(self.main_page.global_vars["sensor_buffer"]) - 1:
                sensor_index += 1
            while self.main_page.global_vars["sensor_buffer"][sensor_index].current_time.to_sec() - self.main_page.global_vars["sensor_buffer"][0].current_time.to_sec() > time_var - seconds and sensor_index > 0:
                sensor_index -= 1
            return sensor_index




if __name__ == '__main__':
    app = QApplication(sys.argv)
    main = Window(1,1)
    main.resize(860, 640)
    main.show()
    sys.exit(app.exec_())
