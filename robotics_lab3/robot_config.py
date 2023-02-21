import numpy as np
import math
import robot_model

if __name__ == '__main__':
# PART A
    np.set_printoptions(precision = 2, suppress = True)

    # Define links
    link1 = robot_model.dh_transformation(math.pi/2, 1, 0, 0)
    link2 = robot_model.dh_transformation(math.pi/2, 1, 0, 0)

    # Calculate total transformation
    total = robot_model.kinematic_chain([link1, link2])

    # Retrieving positions
    pos = robot_model.get_pos(total)
    x = pos[0]
    y = pos[1]
    z = pos[2]

    # Retrieving angles
    rot = robot_model.get_rot(total)
    roll = rot[0]
    pitch = rot[1]
    yaw = rot[2]

    # Displaying results
    print("Positions and orientations for part a:") 
    print("x =",x, "y =", y, "z =", z, "roll =", roll, "pitch =", pitch, "yaw =", yaw)
    print()

#PART B CASE 1
    # Define links
    link1 = robot_model.dh_transformation(0, 0, 0.1625, math.pi/2)
    link2 = robot_model.dh_transformation(0, -0.425, 0, 0)
    link3 = robot_model.dh_transformation(0, -0.3922, 0, 0)
    link4 = robot_model.dh_transformation(0, 0, 0.1333, math.pi/2)
    link5 = robot_model.dh_transformation(0, 0, 0.0997, -math.pi/2)
    link6 = robot_model.dh_transformation(0, 0, 0.0996, 0)

    # Calculate total transformation
    total = robot_model.kinematic_chain([link1, link2, link3, link4, link5, link6])

    # Retrieving positions
    pos = robot_model.get_pos(total)
    x = pos[0]
    y = pos[1]
    z = pos[2]

    # Retrieving angles
    rot = robot_model.get_rot(total)
    roll = rot[0]
    pitch = rot[1]
    yaw = rot[2]

    # Displaying results
    print("Positions and orientations for part b case 1:") 
    print("x =",x, "y =", y, "z =", z, "roll =", roll, "pitch =", pitch, "yaw =", yaw)
    print()

#PART B CASE 2
    # Define links
    link1 = robot_model.dh_transformation(0, 0, 0.1625, math.pi/2)
    link2 = robot_model.dh_transformation(-math.pi/2, -0.425, 0, 0)
    link3 = robot_model.dh_transformation(0, -0.3922, 0, 0)
    link4 = robot_model.dh_transformation(0, 0, 0.1333, math.pi/2)
    link5 = robot_model.dh_transformation(0, 0, 0.0997, -math.pi/2)
    link6 = robot_model.dh_transformation(0, 0, 0.0996, 0)

    # Calculate total transformation
    total = robot_model.kinematic_chain([link1, link2, link3, link4, link5, link6])

    # Retrieving positions
    pos = robot_model.get_pos(total)
    x = pos[0]
    y = pos[1]
    z = pos[2]

    # Retrieving angles
    rot = robot_model.get_rot(total)
    roll = rot[0]
    pitch = rot[1]
    yaw = rot[2]

    # Displaying results
    print("Positions and orientations for part b case 2:") 
    print("x =",x, "y =", y, "z =", z, "roll =", roll, "pitch =", pitch, "yaw =", yaw)
    print()
