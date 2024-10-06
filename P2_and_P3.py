
import numpy as np


def dh(alpha, a, d, theta):

    # Take the DH-Parameters as inputs (one row of the DH-Table)
    # Output the homogenous transformation matrix T for any input joint position theta.

    # Row 1 of the matrix
    r_1_1 = np.cos(theta)
    r_1_2 = -1*np.sin(theta)*np.cos(alpha)
    r_1_3 =
    r_1_4 =

    # Row 2 of the matrix
    r_2_1 = np.sin(theta)
    r_2_2 =
    r_2_3 =
    r_2_4 =

    # Row 3 of the matrix


    # Row 4 of the matrix


    # Build the complete transformation matrix in an np.array
    T =
    return T

def fk_calc(q: np.ndarray, dh_para: np.ndarray, numjoints=3):
    T_all = []  # Initialize a list of the transforms we will need

    T_0_0 = np.array([[1, 0, 0, 0],  # create T_0_0
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

    T_all.append(T_0_0)  # Put our first transform in our list

    # In this loop we want to calculate the transform from the base frame to each joint
    # for example, if we have 3 rows in our DH-table we will have 4 transforms
    # since we include the identity base frame as initialized above. Once we calculate each
    # transform we will append it to the list.
    for i in range(numjoints):
        T =                 # Get each transformation matrix
        T_0_i =             # Do some calculations here
        T_all.append(T_0_i) # put them all in a list

    return T_all

def jacobian_fromVP(q: np.ndarray, dh_para: np.ndarray):

    # this function should calculate the robot jacobian using the velocity propagation approach
    # you should use your dh_to_T function to calculate your homogenous transforms
    # follow the method from class to calculate you z and t vectors

    Jacobian = np.zeros((6, 3))

    T_0_0 = np.array([[1, 0, 0, 0],         # create T_0_0
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    # Calculate each of the homogenous transforms for the robot at the prescribed joint position
    T_0_1 = dh(dh_para[0, 0], dh_para[0, 1], dh_para[0, 2], q[0])
    T_1_2 =
    T_2_3 =

    T_0_2 =
    T_0_3 =

    # Use the transformation matrices to calculate the z and t vectors needed for the velocity
    # propagation method as demonstrated in class.
    z_0 = T_0_0[0:3, 2]
    t_0 = T_0_0[0:3, 3]

    z_1 =
    t_1 =

    z_2 =
    t_2 =

    z_3 =
    t_3 =

    # Build the Jacobian

    # Column 1
    Jacobian[0:3, 0] = np.round(np.cross(z_0, (t_3-t_0)), 3)
    Jacobian[3:6, 0] = np.round(z_0, 3)

    # Column 2
    Jacobian[0:3, 1] =
    Jacobian[3:6, 1] =

    # Column 3
    Jacobian[0:3, 2] =
    Jacobian[3:6, 2] =


    return Jacobian

def jacobianMoreJoints(q: np.ndarray, dh_para: np.ndarray, numjoints=3):
    # This function should calculate the jacobian for a robot with 1 or more joints
    # you should use a loop to calculate the homogenous transform matrices and then
    # extract your z and t vectors from those matrices before using them to build
    # the complete jacobian.


    Jacobian =                          # Initialize the jacobian size correctly

    T_all = []                          # Initialize an empty list of the transforms we will need

    T_0_0 = np.array([[1, 0, 0, 0],     # create T_0_0
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

    T_all.append(T_0_0)                 #Put our first transform in our list


    # In this loop we want to calculate the transfrom from the base frame to each joint
    # for example, if we have 3 rows in our DH-table we will have 4 transforms
    # since we include the identity base frame as initialized above. Once we calculate our
    # transform we will append it to the list.
    for i in range(numjoints):
        T =                 # find the homogenous transform to the next joint
        T_0_i =             # find the homogenous transform from the baseframe to each joint
        T_all.append(T_0_i) #append each transform to the list

    # To calculate our Jacobian we need the final transform to the endpoint so lets get that:
    T_0_end =               # get the last homogenous transform in our list
    t_end =                 # get the t vector from that transform


    # In this loop we will get our z and t vectors for each transform
    # once we have our z and t vectors we build the jacobian 1 column at a time.
    for i in range(numjoints):
        T_0_i =    # get each transform
        z_i =      # get the z vector for the current transform
        t_i =      # get the t vector for the current transform

        # within the same loop, calculate each column of the jacobian
        # using velocity propagation
        Jacobian[0:3, i] =   # linear components
        Jacobian[3:6, i] =   # angular components


    return Jacobian


if __name__ == "__main__":\

 # PART (2b): Fill in what you need to run part (2b) and return the final transform from the base frame
 # to the end-point frame. I have added some code here to get you started.
    h1 =
    l2 =
    l3 =
    alpha1 =

    a =
    d =
    alpha =
    theta =

    dh_paras = np.array([alpha, a, d, theta]).T
    q0 = [0.0, 0.0, 0.0]
    q1 =
    T0 = fk_calc(q0, dh_paras, 3)
    T1 =

 # PART (2c): Fill in what you need to run part (2c) and plot the end-point position as all three joints
 # move from 0 to pi/4.

 # Part (3c): Fill in what you need to run part (3c) and compare the two Jacobian functions you have created.
 # I recommend testing them at the 0 position first and then trying others positions. Testing 3-5 positions is sufficient.
 # Remember to use your dh() and fk_calc() functions as needed.

 # Part (3d): Fill in what you need to run part (3d) and produce the end-point position and Jacobian for the
 # UR robot at the two requested positions. Remember to use your dh() and fk_calc() functions as needed.


