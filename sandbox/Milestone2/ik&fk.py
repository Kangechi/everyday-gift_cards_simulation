"""
Learnt about inverse kinematics and Forwrd kinematics
FK - its input is angles with that its able to tell the position(getLinkState)
& orientation of the robot arm/gripper
-this is significance in that it supports the RL agent toprovide reward as well as test for accuracy
IK - is significant and convinient in the process of moving the joints to a certain target position
Its input is the position itself and it is able to calculate the angles required transposing the joints to that position
- so the variable of the IK saves the angles
They work together to ensure that everything works
It also build on the math of of vectors and tranformation - such that it requitres matrix multiplication similar to Euler angles


8/4/26
The logs I'm getting is a big difference where the arm is not moving & the joints
are fixed at the same position 
Hence adding a new part of the logic:
resetHome() - To orient the arm position to the home position before any movement
resetPoses() inside the ik_move() - to reset the position of the arm to the home position before moving to the target position
"""
import pybullet as p 
import pybullet_data
import numpy as np  
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.loadURDF('plane.urdf')
p.resetDebugVisualizerCamera(
    1.5, 50, -35, [0.4, 0, 0.55]
)

robot_id = p.loadURDF('franka_panda/panda.urdf', useFixedBase=True)
END_EFF = 11
DOWN_ORN = p.getQuaternionFromEuler([np.pi, 0, 0])

#Build scene - bench/paper/decoration target
"""
Shorthand for the creation of custom objects
- Here combination of collision shape/after that/ visualshape(defining the appearance ) then 
multibody - that actautes the actual object
"""
def box(pos,hw,mass,col):
    col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=hw)
    vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=hw, rgbaColor=col)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=col_id, baseVisualShapeIndex=vis_id, basePosition=pos)  


bench_id = box([0.40, 0,0.58], [0.40, 0.30,0.02], 0, [0.76, 0.6,0.42,1])
paper_id = box([0.40,0.05, 0.602], [0.074, 0.105,0.001], 0.005, [0.97,0.88,1])
p.addUserDebugText("Paper", [0.40, 0.05, 0.65], [1,1,1], 1.0)
p.addUserDebugText("Bench", [0.70, 0.0, 0.63], [0.80, 0.6, 0.3], 0.8)
#Assign the features of the card using change dynamics - features such as mass/friction
p.changeDynamics(paper_id, -1, lateralFriction=0.6, linearDamping=0.8)
grasp_con = None

def reset_to_home():
    home = [0, -0.5,0, -2.0, 0, 1.5, 0.8] #home co-ordinates for the 7 joints that are to move to the target position
    for i, angle in enumerate(home):
        p.resetJointState(robot_id, i, angle)
    for _ in range(200):
        p.stepSimulation()
        time.sleep(1./240.)



def ik_move(pos, orn=None, steps=400, vel=1.0):
    """
    We use the inverse kinematics function to get the angles in which the object should move 
    The jointmotorcontrolarray ensures that multiple joints can be moved in the simulation
    """
    arm_joints = list(range(7))
    rest_pose = [0, -0.5, 0, -2.0, 0, 1.5, 0.8]

    a = p.calculateInverseKinematics(robot_id, END_EFF,
                                     pos, orn or DOWN_ORN
                                    )
    print("IK output:", a[:7])
    
    p.setJointMotorControlArray(
        robot_id, arm_joints,
        p.POSITION_CONTROL, targetPositions=a[:len(arm_joints)], forces=[500]*len(arm_joints),
        targetVelocities=[vel]*len(arm_joints))
    
    print(f"the seven joints moving to {pos} with velocity {vel} while target position is {a[:7]}")
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)
    
    actual = p.getLinkState(robot_id, END_EFF)[0]#Get the world position of the endeffector (specific joint)
    diff = np.array(actual) - np.array(pos)
    print(f"Actual END efector pos {actual}")
    print(f"changeinx={diff[0]*100: .1f}cm, changeiny={diff[1]*100: .1f}cm changeinz={diff[2]*100: .1f}cm")
    return np.linalg.norm(diff)

def grasp():
    global grasp_con, paper_id

    g_pos = np.array(p.getLinkState(robot_id, END_EFF)[4])
    p_pos = np.array(p.getBasePositionAndOrientation(paper_id)[0])
    print(f"Grasping -grip_pos {g_pos}, paper_pos {p_pos}")

    if np.linalg.norm(g_pos - p_pos) > 0.05: 
        print(f"Too far to grasp -distance{np.linalg.norm(g_pos-p_pos)}")
        return False
    #Constraintweld paper to gripper
    """
    Including/Creating the constarint enables the gripper to hold onto the paper 
    and not to let go until the release function is called hence enabling the gripper and paper move together.
    """
    grasp_con = p.createConstraint(
        robot_id, END_EFF, paper_id, -1,
        p.JOINT_FIXED, [0,0,0], [0,0,-0.02], [0,0,0]
    )
    return True

def release():
    global grasp_con
    if grasp_con: 
        p.removeConstraint(grasp_con)
        grasp_con= None
    

def compute_reward():
    paper = np.array(p.getBasePositionAndOrientation(paper_id)[0])
    target = np.array([0.60, -0.20, 0.601])
    dist = np.linalg.norm(paper - target)

    contacts = p.getContactPoints(robot_id, paper_id)
    force_bonus = min(0.20, sum(c[9] for c in contacts) * 0.05) if contacts else 0
    return (1.0 - np.clip(dist, 0,1)) + force_bonus

reset_to_home()

err = ik_move([0.40, 0.05, 0.68])
print(f"Moved above paper - IK error {err}m")

ik_move([0.40, 0.05, 0.608], vel=0.15, steps=800)
if grasp(): print("Grasped")

ik_move([0.40, 0.05, 0.75])
ik_move([0.60, -0.10, 0.75])
ik_move([0.60, -0.20, 0.608], vel=0.25, steps=600)

release()

ik_move([0.50, 0.10, 0.614], p.getQuaternionFromEuler([np.pi,0,0]), vel=0.2)
ik_move([0.50, -0.10, 0.614], p.getQuaternionFromEuler([np.pi,0,0]),
        vel=0.15, steps=900)   # very slow drag along fold line

# 7. Compute final reward (FK: paper position, Dynamics: contact quality)
reward = compute_reward()
print(f"Final reward: {reward:.3f}")

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    print("Stopped by User")
finally:
    p.disconnect()
    print("Disconnected")


