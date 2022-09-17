# Remastered Interface v0.4
Updated Interface Based on PyQt5 library

# Usage

```
git init
git clone https://github.com/OSUrobotics/RemasteredInterface.git master
```

Interface uses ROS messages and ROS bags
so do not forget to

```
source devel/setup.bash
```

To start interface

```
roslaunch RemasteredInterface main_page.launch
```

# Dependencies

- Python packages
  - numpy-stl
  - matplotlib
  - PyQt5
  - multiprocessing

- ROS packages
  - rosbag
  - rospy  
  - rviz
  - xacro
  - kinova_description
  - robot_state_publisher
  - std_msgs
  - sensor_msgs
  - infrastructure_msgs https://github.com/OSUrobotics/infrastructure_msgs/tree/main

# Purpose and Design

The main purpose of the interface is to hide all scary ROS and Linux commands under a more user-friendly environment

### Initial Design concepts

### First Page
![alt text](https://github.com/OSUrobotics/RemasteredInterface/blob/development/design/FirstPage.png)


### Main Page
![alt text](https://github.com/OSUrobotics/RemasteredInterface/blob/development/design/MainPage.png)

Design By Sergiy Greblov
          Josh Campbell
          Ryan Roberts

Based on Earlier Iteration of the Interface:
https://github.com/OSUrobotics/infrastructure-packages/tree/development/new_gui



# Custom Elements

## GraphDistance
graphs distance vs time

Arguments
- p - parent
- index - location slot
- num - number of slots [1, 2, 4]


## Items
item selection choice for testbed

Arguments
- p - parent
- statusArray - current Item selection
- index - location slot
- num - number of slots [1, 2, 4]


## GraphFSR
Graphs FSR sensors data vs time

Arguments
- p - parent
- statusArray - active FSR sensors
- index - location slot
- num - number of slots [1, 2, 4]


## GraphImage
Graphs a 3d Model using pyplot and displays active sensors

Arguments
- p - parent
- statusArray - list of currently active FSR sensors
- index - location slot
- num - number of slots [1, 2, 4]


## Menu
Selection between various apparatus, arms, modes and sensors

Arguments
- ap - active apparatus
  1. Drawer
  2. Door
  3. Test Bed

- arm - active arm
  1. Kinova Jaco2
  2. Arm: Thor Arm

- mode - operational mode
  1. Live
  2. Recorded


## Add
Selection between dynamic interface elements using + button

Arguments
- p - parent
- index - location slot
- num - number of slots [1, 2, 4]


## Window
Wrapper class for all other elements in this page
maintains dynamic layout
maintains consistency between various inner elements

Arguments
- ap - active apparatus
  1. Drawer
  2. Door
  3. Test Bed

- arm - active arm
  1. Kinova Jaco2
  2. Arm: Thor Arm

- mode - operational mode
  1. Live
  2. Recorded
- num - number of slots [1, 2, 4]
