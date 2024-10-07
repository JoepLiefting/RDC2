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


    # h1 = 0.5
    # l2 = 1.4
    # l3 = 1.2
    # alpha1 = np.pi / 2.0
    #
    # a = [0.0, l2, l3]
    # d = [h1, 0.0, 0.0]
    # alpha = [alpha1, 0.0, 0.0]

 # PART (2b): Fill in what you need to run part (2b) and return the final transform from the base frame
 # to the end-point frame. I have added some code here to get you started.

    # theta = [0.0, 0.0, 0.0]  # All joint angles set to 0 for part (2b)
    #
    # dh_paras = np.array([alpha, a, d]).T
    # q0 = [0.0, 0.0, 0.0]
    # T0 = fk_calc(q0, dh_paras, 3)
    #
    # print("Final transformation matrix when all joint angles are 0:")
    # print(T0)

 # PART (2c): Fill in what you need to run part (2c) and plot the end-point position as all three joints
 # move from 0 to pi/4.

    # # Step size and range for theta (0 to pi/4)
    # steps = 100
    # theta_range = np.linspace(0, np.pi / 4, steps)
    #
    # # Initialize lists to store the end-effector positions
    # x_positions = []
    # y_positions = []
    # z_positions = []
    #
    # # Iterate through each step and calculate the end-effector position
    # for theta_value in theta_range:
    #     # Joint angles increasing simultaneously
    #     theta = [theta_value, theta_value, theta_value]
    #
    #     # DH parameters (using part 2b settings)
    #     dh_paras = np.array([alpha, a, d]).T
    #
    #     # Calculate forward kinematics
    #     T = fk_calc(theta, dh_paras, 3)
    #
    #     # Extract end-effector position from the transformation matrix (last column)
    #     x, y, z = T[0:3, 3]
    #
    #     # Store the positions
    #     x_positions.append(x)
    #     y_positions.append(y)
    #     z_positions.append(z)
    #
    # # 2D Plot: End-effector positions vs joint angles
    # plt.figure()
    # plt.plot(theta_range, x_positions, label="X Position")
    # plt.plot(theta_range, y_positions, label="Y Position")
    # plt.plot(theta_range, z_positions, label="Z Position")
    #
    # plt.title('End-Effector Position vs Joint Angles (0 to pi/4)')
    # plt.xlabel('Joint Angles (radians)')
    # plt.ylabel('End-Effector Position')
    # plt.legend()
    # plt.grid(True)
    # plt.show()
    #
    # # 3D Plot: End-effector path in 3D space
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    #
    # ax.plot(x_positions, y_positions, z_positions, label='End-Effector Path')
    #
    # ax.set_title('End-Effector Path in 3D Space')
    # ax.set_xlabel('X Position')
    # ax.set_ylabel('Y Position')
    # ax.set_zlabel('Z Position')
    # ax.legend()
    # ax.grid(True)
    #
    # plt.show()

 # Part (3c): Fill in what you need to run part (3c) and compare the two Jacobian functions you have created.
 # I recommend testing them at the 0 position first and then trying others positions. Testing 3-5 positions is sufficient.
 # Remember to use your dh() and fk_calc() functions as needed.

    # dh_paras = np.array([alpha, a, d]).T
    # q = np.array([0.0, 0.0, 0.0])
    # J = jacobian_fromVP(q, dh_paras)
    #
    # print("Jacobian matrix at joint angles q = [0, 0, 0]:")
    # print(J)

 # Part (3d): Fill in what you need to run part (3d) and produce the end-point position and Jacobian for the
 # UR robot at the two requested positions. Remember to use your dh() and fk_calc() functions as needed.

# Universal Robot DH Parameters
    l1 = 0.1519
    l2 = 0.24365
    l3 = 0.21325
    l4 = 0.11235
    l5 = 0.08535
    l6 = 0.0819

    # Correct DH Parameters for the Universal Robot with negative a2 and a3
    a = [0, -l2, -l3, 0, 0, 0]
    d = [l1, 0, 0, l4, l5, l6]
    alpha = [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0]

    dh_paras = np.array([alpha, a, d]).T

    # Position 1: All joint angles set to 0
    q1 = np.array([0, 0, 0, 0, 0, 0])

    # Position 2: q = [0, pi/3, pi/3, 0, pi/4, 0]
    q2 = np.array([0, np.pi / 3, np.pi / 3, 0, np.pi / 4, 0])

    # Compute the forward kinematics and Jacobian for both positions
    print("Position 1: Joint angles set to [0, 0, 0, 0, 0, 0]")
    T1 = fk_calc(q1, dh_paras, 6)
    J1 = jacobianMoreJoints(q1, dh_paras, 6)
    print("End-effector position (T1):")
    print(T1[0:3, 3])  # Extract end-effector position
    print("Jacobian at Position 1:")
    print(J1)

    print("\nPosition 2: Joint angles set to [0, pi/3, pi/3, 0, pi/4, 0]")
    T2 = fk_calc(q2, dh_paras, 6)
    J2 = jacobianMoreJoints(q2, dh_paras, 6)
    print("End-effector position (T2):")
    print(T2[0:3, 3])  # Extract end-effector position
    print("Jacobian at Position 2:")
    print(J2)
