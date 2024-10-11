import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # For 3D plotting

def dh(alpha, a, d, theta):
    # Take the DH-Parameters as inputs (one row of the DH-Table)
    # Output the homogenous transformation matrix T for any input joint position theta.

    # Row 1 of the matrix
    r_1_1 = np.cos(theta)
    r_1_2 = -1 * np.sin(theta) * np.cos(alpha)
    r_1_3 = np.sin(theta) * np.sin(alpha)
    r_1_4 = np.cos(theta) * a

    # Row 2 of the matrix
    r_2_1 = np.sin(theta)
    r_2_2 = np.cos(theta) * np.cos(alpha)
    r_2_3 = -1 * np.cos(theta) * np.sin(alpha)
    r_2_4 = np.sin(theta) * a

    # Row 3 of the matrix
    r_3_1 = 0
    r_3_2 = np.sin(alpha)
    r_3_3 = np.cos(alpha)
    r_3_4 = d

    # Row 4 of the matrix
    r_4_1 = 0
    r_4_2 = 0
    r_4_3 = 0
    r_4_4 = 1

    # Build the complete transformation matrix in an np.array
    T = np.array([[r_1_1, r_1_2, r_1_3, r_1_4],
                  [r_2_1, r_2_2, r_2_3, r_2_4],
                  [r_3_1, r_3_2, r_3_3, r_3_4],
                  [r_4_1, r_4_2, r_4_3, r_4_4]])
    return T


def fk_calc(q: np.ndarray, dh_para: np.ndarray, numjoints=3):
    T_all = []  # Initialize a list of the transforms we will need

    T_0_0 = np.array([[1, 0, 0, 0],  # create T_0_0
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

    T_all.append(T_0_0)  # Put our first transform in our list

    # In this loop we want to calculate the transform from the base frame to each joint
    for i in range(numjoints):
        alpha_i = dh_para[i, 0]
        a_i = dh_para[i, 1]
        d_i = dh_para[i, 2]
        theta_i = q[i]  # Joint angle from q

        # Get each transformation matrix
        T = dh(alpha_i, a_i, d_i, theta_i)

        # Compute the transformation from the base to the current joint
        T_0_i = np.matmul(T_all[i], T)  # Matrix multiplication
        T_all.append(T_0_i)  # Append the new transformation matrix

    return T_all[-1]  # Return the final transformation matrix

def jacobian_fromVP(q: np.ndarray, dh_para: np.ndarray):
    # Initialize the Jacobian matrix (6 rows: 3 linear, 3 angular for 3 joints)
    Jacobian = np.zeros((6, 3))

    # Initial transformation matrix (base to base)
    T_0_0 = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    # Calculate the homogenous transforms
    T_0_1 = dh(dh_para[0, 0], dh_para[0, 1], dh_para[0, 2], q[0])
    T_1_2 = dh(dh_para[1, 0], dh_para[1, 1], dh_para[1, 2], q[1])
    T_2_3 = dh(dh_para[2, 0], dh_para[2, 1], dh_para[2, 2], q[2])

    # Base to joint 2 and base to joint 3 transforms
    T_0_2 = np.matmul(T_0_1, T_1_2)
    T_0_3 = np.matmul(T_0_2, T_2_3)

    # Extract t (position vector) and z (rotation axis) from each transform matrix
    z_0 = T_0_0[0:3, 2]  # z vector for base frame
    t_0 = T_0_0[0:3, 3]  # position vector for base frame

    z_1 = T_0_1[0:3, 2]  # z vector for joint 1
    t_1 = T_0_1[0:3, 3]  # position vector for joint 1

    z_2 = T_0_2[0:3, 2]  # z vector for joint 2
    t_2 = T_0_2[0:3, 3]  # position vector for joint 2

    t_3 = T_0_3[0:3, 3]  # position vector for the end-effector

    # Build the Jacobian matrix
    # Column 1
    Jacobian[0:3, 0] = np.cross(z_0, (t_3 - t_0))  # Linear velocity part
    Jacobian[3:6, 0] = z_0                        # Angular velocity part

    # Column 2
    Jacobian[0:3, 1] = np.cross(z_1, (t_3 - t_1))  # Linear velocity part
    Jacobian[3:6, 1] = z_1                        # Angular velocity part

    # Column 3
    Jacobian[0:3, 2] = np.cross(z_2, (t_3 - t_2))  # Linear velocity part
    Jacobian[3:6, 2] = z_2                        # Angular velocity part

    return Jacobian

def jacobianMoreJoints(q: np.ndarray, dh_para: np.ndarray, numjoints=3):
    # Initialize the Jacobian with the correct size (6 rows, numjoints columns)
    Jacobian = np.zeros((6, numjoints))

    # Initialize the base frame transformation matrix
    T_all = []

    T_0_0 = np.array([[1, 0, 0, 0],     # create T_0_0
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

    T_all.append(T_0_0)  # Append the base frame

    # Loop to calculate transformations from base to each joint
    for i in range(numjoints):
        T = dh(dh_para[i, 0], dh_para[i, 1], dh_para[i, 2], q[i])  # find the transform for joint i
        T_0_i = np.matmul(T_all[i], T)  # Transform from base to joint i
        T_all.append(T_0_i)  # Append each transform to the list

    # Final transform to the end-effector
    T_0_end = T_all[-1]
    t_end = T_0_end[0:3, 3]  # Get the end-effector position

    # Loop through each joint to compute z and t vectors
    for i in range(numjoints):
        T_0_i = T_all[i]  # Get the transformation matrix up to joint i
        z_i = T_0_i[0:3, 2]  # Extract the z vector for joint i
        t_i = T_0_i[0:3, 3]  # Extract the position vector for joint i

        # Calculate each column of the Jacobian
        Jacobian[0:3, i] = np.cross(z_i, (t_end - t_i))  # Linear velocity part
        Jacobian[3:6, i] = z_i  # Angular velocity part

    return Jacobian


if __name__ == "__main__": \


    h1 = 0.5
    l2 = 1.4
    l3 = 1.2
    alpha1 = np.pi / 2.0

    a = [0.0, l2, l3]
    d = [h1, 0.0, 0.0]
    alpha = [alpha1, 0.0, 0.0]

    dh_paras = np.array([alpha, a, d]).T
    q = np.array([0.0, 0.0, 0.0])
    J = jacobian_fromVP(q, dh_paras)

    print("Jacobian matrix by using jacobian_fromVP() at joint angles q = [0, 0, 0]:")
    print(J)

 # Part (3c): Fill in what you need to run part (3c) and compare the two Jacobian functions you have created.
 # I recommend testing them at the 0 position first and then trying others positions. Testing 3-5 positions is sufficient.
 # Remember to use your dh() and fk_calc() functions as needed.




