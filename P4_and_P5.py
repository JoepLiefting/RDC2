import numpy as np
import matplotlib.pyplot as plt


def cubicTraj(T, pos_s, pos_f, vel_s, vel_f):
# This function should return the coefficients for a cubic polynomial that will be used
# to define a trajectory between two points/angles (single degree of freedom) for some time duration T.
# The user should provide start and ending positions and start and ending velociites
# as well as the desired time duration for the trajectory.

# First define a matrix A as we did in class
    A = np.array(...) #fill in the matrix here

#Next define a vector b
    b = np.array([]) #fill in the vector

#Calculate the coefficients for the trajectory here
    coeffs =

    return coeffs

def quinticTraj(T, pos_s, pos_f, vel_s, vel_f, acc_s, acc_f):
    # This function should return the coefficients for a quintic polynomial that will be used
    # to define a trajectory between two points/angles (single degree of freedom) for some time duration T.
    # The user should provide start/ending positions, start/ending velocities and start/ending accelerations
    # as well as the desired time duration for the trajectory.

    #Use a similar strategy as with part (a) and as we did in class.

    A =
    b =
    coeffs =

    return coeffs

def trajPoints(coeffs, T, num, type="cubic"):
    # This function should return vectors of position, velocity and accelerations for a trajectory at each timestep
    # The user should provide the coefficients of the polynomial that define the trajectory as well as the duration
    # T of the trajectory AND the desired number of timesteps.

    # create a time vector with the correct number of time steps over the full duration
    t = np.linspace(0, T, num)

    pos =
    vel =
    acc =

    return pos, vel, acc

def traj3D(start, end, numpoints, type="quintic"):

    #This function should return a 3 degree of freedom trajectory (angular or cartesian) for a robotic manipulator
    #The user will provide a start/end points or angles, start/end velocities and start/end accelerations, as well
    #as the number of points that should be in the trajectory and the type of trajectory desired.

    pos = np.zeros(shape=(numpoints, 3))
    vel = np.zeros(shape=(numpoints, 3))
    acc = np.zeros(shape=(numpoints, 3))

    if type is 'quintic':
        for i in range(3):
    #generate the trajectory for each degree of freedom (joint or cartesian) for a quintic

    elif type is 'cubic':
        for i in range(3):
    # generate the trajectory for each degree of freedom (joint or cartesian) for a cubic

    else:
        print("not a valid trajectory type")

    return pos, vel, acc

if __name__ == "__main__":

 # PART (4c): Fill in what you need to run part (4c) and produce a plot showing position, velocity and
 # acceleration for the specified cubic trajectory

 #input variables
    T = 5. #time
    stp =  #start point
    edp =  #end point
    #add others as needed...

    coeffs = cubicTraj()  #you will definitely call this, but it will need some inputs
    pos, vel, acc = trajPoints() #then you will need this

 #do some plotting!

 # PART (4f): Fill in what you need to run part (4f) and produce a plot showing position, velocity and
 # acceleration for the specified quintic trajectory (should be very similar to the above)

 # PART (5b): Fill in what you need to run part (5b) and produce a 3D plot showing the requested trajectory
