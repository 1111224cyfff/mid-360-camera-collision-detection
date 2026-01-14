# save as inspect_rot.py
import numpy as np
from math import atan2, asin, degrees

R = np.array([
    [-0.63826, -0.0223947,  0.769495],
    [ 0.0165354, -0.999745, -0.0153804],
    [ 0.769643,  0.00290722, 0.638467]
])

# ZYX (yaw-pitch-roll) extraction
sy = -R[2,0]
if sy <= -1: sy = -1
if sy >= 1:  sy = 1
pitch = asin(sy)
yaw = atan2(R[1,0], R[0,0])
roll = atan2(R[2,1], R[2,2])

print("roll (deg):", degrees(roll))
print("pitch (deg):", degrees(pitch))
print("yaw (deg):", degrees(yaw))

# Also print axis-angle (optional)
angle = np.arccos((np.trace(R)-1)/2)
axis = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
axis = axis / np.linalg.norm(axis)
print("angle (deg):", degrees(angle), "axis:", axis)