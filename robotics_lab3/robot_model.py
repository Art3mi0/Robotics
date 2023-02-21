import math
import numpy as np

def dh_transformation(angle, length, offset, twist):
    # Creates a combined homogenous transformation based off of parameters
    # angle = Theta
    # length = a
    # offset = d
    # twist = alpha

    trans = np.array([[math.cos(angle), -math.sin(angle) * math.cos(twist), math.sin(angle) * math.sin(twist), length * math.cos(angle)],
                     [math.sin(angle), math.cos(angle) * math.cos(twist), -math.cos(angle) * math.sin(twist), length * math.sin(angle)],
                     [0, math.sin(twist), math.cos(twist), offset],
                     [0, 0, 0, 1]])
    return trans

def kinematic_chain(lst):
    # returns a homogenous transformation of a kinematic chain
    total = np.identity(4)
    for i in lst:
        total = np.matmul(total, i)
    return total

def get_pos(transformation):
    # retunrs the x, y, and z components of the position
    return [transformation[0][3], transformation[1][3], transformation[2][3]]

def get_rot(rot):
    # returns the roll, pitch, and yaw angles of the position
    
    # I thought this part was required since it was in the lab, and the arctan math used posiotions of a 3X3 matrix, but it returns incorrect values
##    rot = np.array([[math.cos(z) * math.cos(y), -math.sin(z) * math.cos(x) + math.cos(z) * math.sin(y) * math.sin(x),
##                     math.sin(z) * math.sin(x) + math.cos(z) * math.sin(y) * math.cos(x)],
##                    [math.sin(z) * math.cos(y), math.cos(z) * math.cos(x) + math.sin(z) * math.sin(y) * math.sin(x),
##                     -math.cos(z) * math.sin(x) + math.sin(z) * math.sin(y) * math.sin(x)],
##                    [-math.sin(y), math.cos(y) * math.sin(x), math.cos(y) * math.cos(x)]])

    roll = math.atan2(rot[2][1], rot[2][2])
    pitch = math.atan2(-rot[2][0], ((rot[2][1] ** 2) * (rot[2][2] ** 2)) ** (1/2))
    yaw = math.atan2(rot[1][0], rot[0][0])
    return [roll, pitch, yaw]
