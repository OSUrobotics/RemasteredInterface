# Remastered Interface v0.4 
Updated Interface Based on PyQt5 library

# Usage

```
git init 
git clone https://github.com/OSUrobotics/RemasteredInterface.git master

```

# Custom Elements 

## GraphDistance
graphs distance vs time 

Args 
- p - parent 
- index - location slot 
- num - number of slots [1, 2, 4]


## Items
item selection choice for testbed 

- p - parent 
- statusArray - current Item selection 
- index - location slot 
- num - number of slots [1, 2, 4]


## GraphFSR
Graphs FSR sensors data vs time 

- p - parent 
- statusArray - active FSR sensors 
- index - location slot 
- num - number of slots [1, 2, 4]


## graphImage
Graphs a 3d Model using pyplot and displays active sensors 

- p - parent 
- statusArray - list of currently active FSR sensors 
- index - location slot 
- num - number of slots [1, 2, 4]


## Menu
Selection between various apparatus, arms, modes and sensors 

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

- p - parent 
- index - location slot 
- num - number of slots [1, 2, 4]


## Window
Wrapper class for all other elements in this page 
maintains dynamic layout
maintains consistency between various inner elements 

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
