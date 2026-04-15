import pybullet as p
import pybullet_data 
import time, numpy as np    

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0, -9.8)
p.resetDebugVisualizerCamera(1.5, 50, -35, 
                             [0.4, 0, 0.55])

p.loadURDF('plane.urdf')

robot_id = p.loadURDF('franka_panda/panda.urdf', 
                      basePosition=[0,0,0],
                      useFixedBase= True
                      
                      )

def reset_home():
    home_pos = [-0.8, -0.3, 0, -1.5, 0, 1.4, 0.5]
    for i, angle in enumerate(home_pos):
        p.resetJointState(robot_id, i, angle)

def box_body(pos, mass, col,hw):
    c= p.createCollisionShape(p.GEOM_BOX,
                                   halfExtents= hw)
    v =p.createVisualShape(p.GEOM_BOX, halfExtents= hw,
                           rgbaColor =col)   
    return p.createMultiBody(mass, c,v, pos)

bench_id = box_body([0.40, 0, 0.58], 0, [0.76, 0.6, 0.42, 1], [0.40, 0.30, 0.02])
paper_id = box_body([0.40, 0.05, 0.602], 0.005, [1, 0.97, 0.88, 1], [0.074, 0.105, 0.001])
target = np.array([0.50,-0.10, 0.601 ])

END_EFF = 11
GRIP_DOWN = p.getQuaternionFromEuler([np.pi, 0,0])

def move_to(pos, vel=1.0, steps= 400):
    a =p.calculateInverseKinematics(
        robot_id, END_EFF, pos, GRIP_DOWN
    )
    arms_joints = list(range(7))

    p.setJointMotorControlArray(
        robot_id, arms_joints,
        p.POSITION_CONTROL, targetPositions=a[:len(arms_joints)],
          forces=[500]* len(arms_joints),
          targetVelocities=[vel]*len(arms_joints))
    
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)


def set_gripper(width):
    for fi in [9,10]:
        p.setJointMotorControl2(
            robot_id, fi, p.POSITION_CONTROL,
            width, 50
        )
    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./240.)


paper_start = [0.40, 0.05, 0.60]

print("Setting the arm poistion")
reset_home()

print("1. Open Gripper")
set_gripper(0.04)

print("2. Move above paper")
move_to([0.40, 0.05, 0.65])

print("3. Descend slowly to paper")
move_to([0.40, 0.05, 0.65], vel=0.3, steps=600)

print("4. Open Gripper")
set_gripper(0.0)

print("5. Lift Paper")
move_to([0.40, 0.05, 0.70])
pos = p.getBasePositionAndOrientation(paper_id)[0]
print(f"Paper z={pos[2]: .3f} (was 0.602 on bench)")

print("6 Move to target zone")
move_to([0.50, -0.10, 0.70])

print("7. Descend and place")
move_to([0.50, -0.10, 0.63], vel = 0.3, steps=600)
set_gripper(0.04)

print("8. compute reward")
final_pos = np.array([p.getBasePositionAndOrientation(paper_id)[0]])
dist = np.linalg.norm(final_pos - target)
reward = 1.0 - np.clip(dist, 0,1)
print(f' Distance to target: {dist*100: .1f}cm Reward: {reward: .3f}')

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    print("User interrupted")
finally:
    p.disconnect()