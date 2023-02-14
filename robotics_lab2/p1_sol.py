# import the required modules
import math
import numpy as np
import rbm

if __name__ == '__main__':
    # define a test value
    psi = math.pi/2 
    theta = math.pi/2 
    phi = math.pi/2 
    # update the output format
    np.set_printoptions(precision = 2, suppress = True)
    # define a 3D vector
    v0 = rbm.vec(1,0,0)
    # define a 3D rotation about x axis
    Rx = rbm.rot_x(psi)
    # define a 3D rotation about y axis
    Ry = rbm.rot_y(theta)
    # define a 3D rotation about z axis
    Rz = rbm.rot_z(phi)
    # calculate a total rotation via FIXED FRAME
    R = np.matmul(Rz, Ry)
    R = np.matmul(R, Rx)
    # calculate the results of rotation
    v01 = R.dot(v0)
    print('The transformed vector (rotations about FIXED FRAME) is:\n',v01)
