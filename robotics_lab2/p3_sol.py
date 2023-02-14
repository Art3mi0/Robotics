# import the required modules
import math
import numpy as np
import p2_sol

if __name__ == '__main__':
    np.set_printoptions(precision = 2, suppress = True)
    # define 3D vectors
    h_1 = p2_sol.vec(1,0,0)
    h_2 = p2_sol.vec(1,0,0)
    h_3 = p2_sol.vec(1,0,0)
    h_4 = p2_sol.vec(1,0,0)
    h_5 = p2_sol.vec(1,0,0)

    # define 3D Rotations about axis
    Rx = p2_sol.rot_x(math.pi/2)
    Rz = p2_sol.rot_z(-math.pi/2)
    
    # define 3D translations about axis
    Tx1 = p2_sol.trans_x(2.5)
    Tz1 = p2_sol.trans_z(0.5)
    Ty1 = p2_sol.trans_y(-1.5)

    Tx2 = p2_sol.trans_x(3)
    Tz2 = p2_sol.trans_z(-3)

    # calculate a total transformation via CURRENT FRAME
    T1 = np.matmul(Tx1, Tz1)
    T1 = np.matmul(T1, Ty1)
    
    T2 = np.matmul(Tz1, Tx1)
    T2 = np.matmul(T2, Ty1)
    
    T5 = np.matmul(Rx, Tx2)
    T5 = np.matmul(T5, Tz2)
    T5 = np.matmul(T5, Rz)

    # calculate a total transformation via FIXED FRAME
    T3 = np.matmul(Ty1, Tz1)
    T3 = np.matmul(T3, Tx1)
    
    T4 = np.matmul(Ty1, Tx1)
    T4 = np.matmul(T4, Tz1)

    # calculate the results of rotation
    h_1 = T1.dot(h_1)
    h_2 = T2.dot(h_2)
    h_3 = T3.dot(h_3)
    h_4 = T4.dot(h_4)
    h_5 = T5.dot(h_5)
    print('The transformed vector (Tx, Tz, Ty about CURRENT FRAME) is:\n',h_1)
    print('The transformed vector (Tz, Tx, Ty about CURRENT FRAME) is:\n',h_2)
    print('The transformed vector (Tx, Tz, Ty about FIXED FRAME) is:\n',h_3)
    print('The transformed vector (Tz, Tx, Ty about FIXED FRAME) is:\n',h_4)
    print('The transformed vector (Rx, Tx, Tz, Rz about CURRENT FRAME) is:\n',h_5)
