import pybullet_data 
import pybullet as p
import time

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)


p.loadURDF("plane.urdf")
robot_id = p.loadURDF('franka_panda/panda.urdf', 
           basePosition= [0,0,0],
           useFixedBase=True)

print(f"\nPanda loaded. Robot ID = {robot_id}")
print(f"Total Joints {p.getNumJoints(robot_id)}")

JOINT_TYPES = {
    p.JOINT_REVOLUTE: 'REVOLUTE (rotates)',
    p.JOINT_PRISMATIC: 'PRISMATIC (slides)',
    p.JOINT_FIXED: 'FIXED (rigid, no movement)',
    p.JOINT_SPHERICAL: 'SPHERICAL (ball joint)',
    p.JOINT_PLANAR: 'PLANAR',
}
print(f"{'IDX':<5} {'NAME':<30} {'TYPE'}")
print("-"*60)
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    idx = info[0]
    name =info[1].decode()
    jtype = info[2]
    low_lim = info[8]
    up_lim = info[9]
    type_str = JOINT_TYPES.get(jtype, str(jtype))
    print(f"{idx:<5} {name:<30} {type_str}")



