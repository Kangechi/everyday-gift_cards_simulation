import pybullet as p
import pybullet_data
import numpy as np   
import time


p.connect(p.GUI, options="background_color_rgb=0.1,0.1,0.1")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.resetDebugVisualizerCamera(
    1.5, 50, -35, [0.4,0,0.68]
)

GRIPPER_JOINTS = [9,10]
END_EFF = 11
DOWN_ORN = p.getQuaternionFromEuler([np.pi, 0, 0])

def open_gripper():
    target = 0.04

    for joint in GRIPPER_JOINTS:
        p.setJointMotorControl2(
            robot_id,
            joint,
            p.POSITION_CONTROL,
            targetPosition=target,
            force=100
        )
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def close_gripper():
    target = 0.0

    for joint in GRIPPER_JOINTS:
        p.setJointMotorControl2(
            robot_id,
            joint,
            p.POSITION_CONTROL,
            targetPosition=target,
            force=200
        )
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def set_home():
    home_pos = [-0.8, -0.3, 0, -1.5, 0, 1.4, 0.5]
    for i, angle in enumerate(home_pos):
        p.resetJointState(robot_id, i, angle)
    for _ in range(200):
        p.stepSimulation()
        time.sleep(1./240.)

def ik_move(pos, orn=None, steps=600, vel=1.0):
    arm_joints= list(range(7))
    rest_pose = [-0.8, -0.3, 0, -1.5, 0, 1.4, 0.5]

    angles = p.calculateInverseKinematics(
        robot_id,
        END_EFF,
        pos,
        orn or DOWN_ORN
    )

    print(f"IK output {angles[:7]}")

    p.setJointMotorControlArray(
        robot_id,
        arm_joints,
        p.POSITION_CONTROL, targetPositions=angles[:len(arm_joints)], forces=[500]*len(arm_joints),
        targetVelocities=[vel]*len(arm_joints))
    print(f"Moving to {pos} with velocity {vel} to target position {angles[:7]}")

    actual = p.getLinkState(robot_id, END_EFF)[0]
    diff = np.array(actual) - np.array(pos)
    print(f"Actual end effector position {actual}")
    print(f'change in x = {diff[0]*100: .1f}cm, change in y = {diff[1]*100: .1f}cm , change in z = {diff[2] *100: .1f}cm')

    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)

def contact():
    global grasp_con
    g_pos = np.array(p.getLinkState(robot_id, END_EFF)[4])
    p_pos = np.array(p.getBasePositionAndOrientation(paper)[0])

    if np.linalg.norm(g_pos - p_pos) > 0.05:
        print(f"Too far to grasp {np.linalg.norm(g_pos-p_pos)}")
        return False
    
    grasp_con = p.createConstraint(
        robot_id, END_EFF, paper, -1,
        p.JOINT_FIXED,
        [0,0,0], [0,0,-0.02], [0,0,0]
    )
    return True

def release():
    global grasp_con
    if grasp_con:
        p.removeConstraint(grasp_con)
        grasp_con= None


def box(pos, hw, mass, col):
    col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=hw)
    vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=hw, rgbaColor= col)
    return p.createMultiBody(baseMass=mass, baseVisualShapeIndex= vis_id, baseCollisionShapeIndex=col_id, basePosition=pos)

def compute_reward():
    paper_id = np.array(p.getBasePositionAndOrientation(paper)[0])
    target = np.array([0.40, 0.005, 0.58])

    dist = np.linalg.norm(paper - target)

    contacts = p.getContactPoints(robot_id, paper)
    force_bonus = min(0.20, sum(c[9] for c in contacts) * 0.05) if contacts else 0
    return (1.0 - np.clip(dist, 0,1)) + force_bonus



bench = box([0.40,0,0.58], [0.40,0.30, 0.02], 0, [0.76, 0.6,0.42,1])
paper = box([0.40, 0.05, 0.602], [0.074, 0.105, 0.001], 0.005, [0.97,0.88,1])
p.changeDynamics(paper, -1, lateralFriction=0.6, linearDamping=0.8)
p.loadURDF('plane.urdf')
robot_id =p.loadURDF("franka_panda/panda.urdf", basePosition=[0,0,0], useFixedBase= True, baseOrientation=p.getQuaternionFromEuler([0,0,np.pi/4]))
set_home()
open_gripper()
ik_move([0.40, 0.005, 0.58])
contact()
close_gripper()
ik_move([0.55,0.15, 0.650])
ik_move([0.40, -0.20, 0.650])
ik_move([0.40, 0.10, 0.65])
ik_move([0.40, 0.005, 0.650])
compute_reward()
open_gripper()
release()
"""
Get the gripper joints inorder to manipulate the fingers

for i in range(p.getNumJoints(robot_id)):
    print(i, p.getJointInfo(robot_id,i)[1])
"""

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)
