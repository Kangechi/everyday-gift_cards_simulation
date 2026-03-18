from tkinter import VERTICAL
import numpy as np

"""
Understanding the math that goes into robotic simulations.
- Vectors/ Vector distances and eucludian distance that is important in identifying the
distance between two objects in a space - an example(the hand of the robot & the paper card)
- Apart from that understanding that the vectors matrices - display the position of the objects in a space.
"""

gripper_pos = np.array([0.4,0.0, 0.5]) # gripper above workbench
paper_pos = np.array([0.4, 0.1,0.61]) #Paper sheet on the workbench

#Vector from paper to gripper
diff = gripper_pos - paper_pos
print(f"Displacement: {diff}" )

#Distance - euclidean distance
distance = np.linalg.norm(diff)
print(f"Distance: {distance: .4f} m")

#Unit vector - direction only
direction = diff/distance
print(f"Direction: {direction}")



#Is the gripper close enough to grasp?
GRASP_DISTANCE = 0.05
if distance < GRASP_DISTANCE:
    print("Close Enough, attempt grasp")
else:
    print(f"Move {distance - GRASP_DISTANCE: .3f}m closer")



"""
Dot and Cross product
1. Dot product aids in alignment of the objects in the space.
- It enables you to understand the angle between the two objects/vectors in the space.
- If it reteurn +1 = same direction(parallel) 0 = perpendicular -1 = Opposite direction
2. The Crossproduct - is used to identify the perpendicular axis 
-This is in order to identify which way the arm should rotate to reach the target

"""

#Dot product - Alignment
gripper_facing = np.array([0,0, -1]) #gripper pointing down
paper_normal = np.array([0,0, 1]) #Paper facing Up

alignment = np.dot(gripper_facing, paper_normal)
print(f"Alignment: {alignment}")

#Angle btwn the two vectors
cos_angle = np.dot(gripper_facing, paper_normal) / (np.linalg.norm(gripper_facing) * np.linalg.norm(paper_normal))

angle_deg = np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))
print(f"Angle between gripper and paper: {angle_deg: .1f}")


#- Cross product - perpendicular axis
#When you need to rotate Vector A to align with Vector B

a = np.array([1,0,0])# Current gripper direction
b = np.array([0,1,0])#Target direction

rotation_axis = np.cross(a,b)
print(f"Rotate around axis: {rotation_axis}")



"""
For the dot and cross product we'll use them in the use case of the card making
They help with the folding orientation and alignment
"""

fold_start = np.array([0.35, 0.10, 0.61])
fold_end = np.array([0.45, 0.10, 0.61])
fold_vec = fold_end - fold_start

horizontal = np.array([1,0,0])
vertical = np.array([0,1,0])

h_align = abs(np.dot(fold_vec/ np.linalg.norm(fold_vec), horizontal))
v_align = abs(np.dot(fold_vec / np.linalg.norm(fold_vec), vertical))

fold_orientation = "horizontal" if h_align > v_align else: "vertical"
print(f"Fold is {fold_orientation}")