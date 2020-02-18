import numpy as np 
from scipy.spatial.transform import Rotation as R
from math import cos, sin


def getRx(theta):
    T = np.eye(4)
    T[1,:] = [0,cos(theta),-sin(theta),0]
    T[2,:] = [0,sin(theta),cos(theta),0]
    return T

def getRy(theta):
    T = np.eye(4)
    T[0,:] = [cos(theta),0,sin(theta),0]
    T[2,:] = [-sin(theta),0,cos(theta),0]
    return T

def getRz(theta):
    T = np.eye(4)
    T[0,:] = [cos(theta),-sin(theta),0,0]
    T[1,:] = [sin(theta),cos(theta),0,0]
    return T

basePose = np.eye(3)

# r = R.from_euler('z', 90, degrees=True)
# new_base = r.apply(basePose)
# print(new_base)

# print('Custom rotation stuff')
# eye = np.eye(4)
# AEE_POSE = eye
# rot = R.from_matrix(AEE_POSE[:3,:3]).as_euler('xyz',degrees=True)
# print(AEE_POSE)
# print(rot)

# print('After rotation')
# AEE_POSE = np.matmul(AEE_POSE,getRx(-np.pi/2))
# rot = R.from_matrix(AEE_POSE[:3,:3]).as_euler('xyz',degrees=True)
# print(f'AEE_POSE rotation {AEE_POSE[:3,:3]}')
# print(f'rot {rot}')


# print('\nMatrix Reshaping')
# a = np.ones((3,1))
# print(f'a: {a}')
# print(f'shape of a: {a.shape}')
# print(f'type of a: {type(a)}')
# a = np.ones((1,3))
# print(f'a: {a}')
# print(f'shape of a: {a.shape}')
# a = a.reshape(-1)
# print(f'a: {a}')
# print(f'shape of a: {a.shape}')

a = np.array([[1,2,3],[1,2,4],[1,2,9],[6,2,-1]])
print(a)
print(a[a[:,2].argsort()])