import pybullet_data 
import pybullet as p
import time

physics_client = p.connect(p.GUI)

p.setAdditionSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)

p.loadURDF("plane.urdf")

for _ in range(3000):
    p.stepSimulation()
    time.sleep(1./240)

p.disconnect()