import pybullet as p
import pybullet_data
import time, numpy as np     

#Setup process with the imports as well as key commands/ instructions
#Connect - GUI/Direct - GUI to enable us to see simulation
p.connect(p.GUI)
#Enable to load urdfs path
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#setting the gravity for the physice engine
p.setGravity(0,0,-9.8)
#Camera view - distance, yaw, pitch target position
p.resetDebugVisualizerCamera(
    1.5, 50, -35, [0.4, 0, 0.55]
)

#Loading the ground + robot
p.loadURDF('plane.urdf')
robot_id = p.loadURDF('franka_panda/panda.urdf', 
                      basePosition= [0,0,0,],
                      useFixedBase=True)

#Defining our own objects using collison shape/ visual shape then multibox
b_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.40,0.30, 0.02])
b_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.40, 0.30,0.02], rgbaColor=[0.76, 0.60,0.42,1])

bench_id = p.createMultiBody(0, b_col, b_vis, [0.40, 0.0, 0.58])

p_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.074, 0.105, 0.01])
p_vis =p.createVisualShape(p.GEOM_BOX, halfExtents=[0.074, 0.105,0.01], rgbaColor=[1.0, 0.97,0.88,1])

paper_id = p.createMultiBody(0.005, p_col, p_vis, [0.40,0.05,0.602]) 
print(f"Paper created {paper_id}, workbench{bench_id}. ")


#Labeling the objects in our simulation - using addUserDebugText

p.addUserDebugText("paper", [0.40,0.05, 0.65], [1,1,1], 1.0)
p.addUserDebugText("bench", [0.70,0.0, 0.63], [0.8,0.6,0.3], 0.8)
p.addUserDebugLine([0.35, 0.10, 0.603], [0.45,0.10, 0.603], [0.2, 0.6,1], 2.0)

print(f"Panda Id: {robot_id}, Panda joints: {p.getNumJoints(robot_id)}")

END_EFF = 11
GRIP_DOWN = p.getQuaternionFromEuler([np.pi, 0,0])
print(f"End effector {END_EFF} world_pos {p.getLinkState(robot_id, END_EFF)[0]}")
print(f"{GRIP_DOWN} - gripper orientation")

def move_to(pos, steps=400):
    angles = p.calculateInverseKinematics(
        robot_id, END_EFF, pos,
        GRIP_DOWN
    )

    arm_joints = list(range(7))

    p.setJointMotorControlArray(
        robot_id,
        arm_joints,
        p.POSITION_CONTROL,
        targetPositions = angles[:len(arm_joints)],
        forces=[500]*len(arm_joints), targetVelocities=[0.25]*len(arm_joints)
        )
    

    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)


move_to([0.40,0.05, 0.55])
pos = p.getLinkState(robot_id, END_EFF)[4]
print(f"Gripper at z= {pos[2]: .3f} (target was 0.55)")
print(f"Moved to {pos}, error {np.linalg.norm(np.array(pos))}")


#Get Contact points - to check if the gripper caught the paper
contacts =p.getContactPoints(robot_id, paper_id)
print(f"Contacts with paper {len(contacts)}")

#Get Camera Image 
view = p.computeViewMatrix([0.40, 0.0, 1.1], [0.40,0.0, 0.6], [0,1,0])#Camera position, target position, up vector
proj = p.computeProjectionMatrix(60, 1.0, 0.1, 2.0, 0.5, 1.0) #FOV, aspect ratio, near plane, far plane
_, _, rgb, _, _ = p.getCameraImage(256, 256, view, proj, p.ER_TINY_RENDERER)
print(f'Camera image: {rgb}')

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    print("User Interrupted")
finally:
    p.disconnect()