#!/usr/bin/env python3
import numpy as np
from math import atan2, asin, degrees

def rot_to_zyx_euler(R):
    # Extract ZYX (yaw, pitch, roll) in radians
    # pitch = asin(-R[2,0]) in our earlier script; keep same sign convention
    sy = -R[2,0]
    if sy <= -1: sy = -1
    if sy >= 1:  sy = 1
    pitch = asin(sy)
    yaw = atan2(R[1,0], R[0,0])
    roll = atan2(R[2,1], R[2,2])
    return degrees(roll), degrees(pitch), degrees(yaw)

def axis_angle_from_R(R):
    # angle
    trace = np.trace(R)
    # numerical safety
    val = (trace - 1.0) / 2.0
    val = max(-1.0, min(1.0, val))
    angle = np.arccos(val)
    # axis
    axis = np.array([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]])
    norm = np.linalg.norm(axis)
    if norm < 1e-8:
        return degrees(angle), np.array([0.0, 0.0, 0.0])
    return degrees(angle), axis / norm

def pretty_print(R, name):
    print('---', name, '---')
    print('R =')
    print(R)
    r, p, y = rot_to_zyx_euler(R)
    print(f'ZYX Euler (deg)  roll={r:.6f}, pitch={p:.6f}, yaw={y:.6f}')
    ang, axis = axis_angle_from_R(R)
    print(f'Axis-angle (deg) angle={ang:.6f}, axis={axis}')
    # check axes mapping for quick intuition
    ex = R.dot(np.array([1.0,0.0,0.0]))
    ey = R.dot(np.array([0.0,1.0,0.0]))
    ez = R.dot(np.array([0.0,0.0,1.0]))
    print(f'R*[1,0,0] = {ex}')
    print(f'R*[0,1,0] = {ey}')
    print(f'R*[0,0,1] = {ez}')
    # check closeness to 180deg about Y: diag(-1,1,-1)
    Ry180 = np.diag([-1.0, 1.0, -1.0])
    diff = np.linalg.norm(R - Ry180)
    print(f'Norm(R - R_y(180)) = {diff:.6f}')
    print()

def main():
    R = np.array([
        [-0.63826, -0.0223947,  0.769495],
        [ 0.0165354, -0.999745, -0.0153804],
        [ 0.769643,  0.00290722, 0.638467]
    ])

    pretty_print(R, 'Original R')
    pretty_print(R.T, 'R Transpose')
    pretty_print(np.linalg.inv(R), 'R Inverse')

if __name__ == '__main__':
    main()
