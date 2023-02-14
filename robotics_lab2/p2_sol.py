# import the required modules
import math
import numpy as np

# use lecture slides for the mathematical equations 
def trans_x(units):
    trans = np.array([[1, 0, 0, units], [0, 1, 0, 0],
                     [0, 0, 1, 0], [0, 0, 0, 1]])
    return trans

def trans_y(units):
    trans = np.array([[1, 0, 0, 0], [0, 1, 0, units],
                     [0, 0, 1, 0], [0, 0, 0, 1]])
    return trans

def trans_z(units):
    trans = np.array([[1, 0, 0, 0], [0, 1, 0, 0],
                     [0, 0, 1, units], [0, 0, 0, 1]])
    return trans

def rot_x(theta):
    rot = np.array([[1, 0, 0, 0], [0, math.cos(theta), -math.sin(theta), 0],
                     [0, math.sin(theta), math.cos(theta), 0], [0, 0, 0, 1]])
    return rot

def rot_y(theta):
    rot = np.array([[math.cos(theta), 0, math.sin(theta), 0], [0, 1, 0, 0],
                     [-math.sin(theta), 0, math.cos(theta), 0], [0, 0, 0, 1]])
    return rot

def rot_z(theta):
    rot = np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                    [math.sin(theta), math.cos(theta), 0, 0],
                     [0, 0, 1, 0], [0, 0, 0, 1]])
    return rot

def vec(x,y,z):
#	Define a vector as an numpy and transpose it to a column vector.
	vec = np.array([[x, y, z, 1]]).T 
	return vec
