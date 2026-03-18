import numpy as np
import pybullet as p
p.connect(p.DIRECT)

print("=" * 50)
print("CRAFT ROBOT _ MATH DEMO (ROBOTICS MATH)")
print("=" * 50)

"""
On Wednesday - Going over everything plus the exercise
1. Distance
2. Unit-vectors (direction)
3. Rotation- euler angles/rotation matrix/ Quartenation


"""

#Defining the positions of the objects in relation to the working table in regard to (x,y,z)

gripper_pos = np.array([])
paper_pos = np.array([])
target_pos = np.array([])#The position we want the paper to be placed by gripper

#Getting the distance the gripper needs to move in order to get the paper
to_paper = paper_pos - gripper_pos
#Getting the Euclidean distance
dist= np.linarg.norm(to_paper)

print(f"[1] Distance to paper: {dist*100: .1f} cm")

#Obtaining the unit vector - direction = given by vector/ euclidean distance

direction = to_paper / dist

print(f"[2] Move in direction {direction.round(3)}")

# The euler gripper orientation ensuring its at 180 - in order for it to face the bottom

euler_grasp = [np.pi, 0,0]

quat_grasp = p.getQuaternionFromEuler(euler_grasp)
#Note that pybullet uses quaternion yet euler is the human readerable version

print(f"[3] Grasp quaternion: {[round(q,2) for q in quat_grasp] }") 
