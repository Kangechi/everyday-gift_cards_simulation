"""
Walking through the syntax of the whole process
As always:
1. Imports - importing pybullet, pybullet_data
-numpy & time
2. The setup process:
- p.connect()- pick the value you wnat dependant on the training process that your walking through
-p.setAdditionalSearchPath()- to enable the system to load the urdfs paths
- p.setGravity()- physics simulator thus needs to be able to get gravity set on the x-dimensional plane
-p.loadurdf- to load the plane, the declaration of a variable that enables you to now store the robot


so what have I learnt today:
 - familiarisation yes with the syntax and working with it
 - Inteeresting things are the IK na FK
 FK that helps in the rewarding loop by checking the actual pos
 IK that is convinient
 -Writing the rewarding code - eucludiean distance
 

"""
import time
import numpy as np   
import pybullet as p 
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.loadURDF('plane.urdf')





p.stepSimulation()
