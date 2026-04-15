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

gripper_pos = np.array([0.40, 0.00, 0.55])
paper_pos = np.array([0.38, 0.05, 0.61])
target_pos = np.array([0.50, -0.1, 0.61])#The position we want the paper to be placed by gripper

#Getting the distance the gripper needs to move in order to get the paper
to_paper = paper_pos - gripper_pos
#Getting the Euclidean distance
dist= np.linalg.norm(to_paper)

print(f"[1] Distance to paper: {dist*100: .1f} cm")

#Obtaining the unit vector - direction = given by vector/ euclidean distance

direction = to_paper / dist

print(f"[2] Move in direction {direction.round(3)}")

# The euler gripper orientation ensuring its at 180 - in order for it to face the bottom

euler_grasp = [np.pi, 0,0]

quat_grasp = p.getQuaternionFromEuler(euler_grasp)
#Note that pybullet uses quaternion yet euler is the human readerable version

print(f"[3] Grasp quaternion: {[round(q,2) for q in quat_grasp] }") 



#After picking the card - rotate card 45 degrees before placing
#Matrix multiplication- tranformation combining both rotation and translation
def rot_z(deg):
    t = np.radians(deg) 
    return np.array([[np.cos(t), -np.sin(t), 0],
                    [np.sin(t), np.cos(t), 0],
                    [0,      0 ,     1]])

card_up = np.array([0,0,1])
card_up_rotated = rot_z(45) @ card_up
print(f"[4] Card top after 45 degress rotate {card_up_rotated.round(3)}")

#Build full transform for the placement pose
T_place = np.eye(4)
T_place[:3, :3] = rot_z(45)
T_place[:3, 3] = target_pos
print(f"[5] Placement transform: \n pos= {T_place[:3, 3]}" f" yaw=45")

#Reward Signal Preview-distance to target
current_card_pos = paper_pos
dist_to_target = np.linalg.norm(target_pos-current_card_pos)
raw_reward = 1.0 - np.clip(dist_to_target, 0,1)
print(f"\n[6] Distance to placement target: {dist_to_target*100:.1f}cm")
print(f" Reward signal: {raw_reward:.3f} (1.0 = perfect placement)")
