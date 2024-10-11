import numpy as np
import matplotlib.pyplot as plt


def cubicTraj(T, pos_s, pos_f, vel_s, vel_f):
    # First define a matrix A as we did in class
    A = np.array([[1, 0,    0,     0],
                  [0, 1,    0,     0],
                  [1, T,  T**2,  T**3],
                  [0, 1, 2*T,  3*T**2]])
    # Next define a vector b
    b = np.array([pos_s, vel_s, pos_f, vel_f])
    # Calculate the coefficients for the trajectory here
    coeffs = np.linalg.solve(A, b)
    return coeffs

def quinticTraj(T, pos_s, pos_f, vel_s, vel_f, acc_s, acc_f):
    # Define matrix A
    A = np.array([[1, 0,    0,      0,      0,       0],
                  [0, 1,    0,      0,      0,       0],
                  [0, 0,    2,      0,      0,       0],
                  [1, T,  T**2,   T**3,   T**4,    T**5],
                  [0, 1,  2*T,   3*T**2, 4*T**3,  5*T**4],
                  [0, 0,    2,   6*T,   12*T**2, 20*T**3]])
    # Define vector b
    b = np.array([pos_s, vel_s, acc_s, pos_f, vel_f, acc_f])
    # Calculate coefficients
    coeffs = np.linalg.solve(A, b)
    return coeffs

def trajPoints(coeffs, T, num, type="cubic"):
    # Create a time vector with the correct number of time steps over the full duration
    t = np.linspace(0, T, num)
    if type == "cubic":
        # Coefficients: [a0, a1, a2, a3]
        a0, a1, a2, a3 = coeffs
        pos = a0 + a1*t + a2*t**2 + a3*t**3
        vel = a1 + 2*a2*t + 3*a3*t**2
        acc = 2*a2 + 6*a3*t
    elif type == "quintic":
        # Coefficients: [a0, a1, a2, a3, a4, a5]
        a0, a1, a2, a3, a4, a5 = coeffs
        pos = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
        vel = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
        acc = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3
    else:
        raise ValueError("Invalid trajectory type")
    return pos, vel, acc

def traj3D(start, end, T, numpoints, type="quintic", space="joint"):
    # Start and end are dictionaries with keys 'pos', 'vel', 'acc'
    pos = np.zeros(shape=(numpoints, 3))
    vel = np.zeros(shape=(numpoints, 3))
    acc = np.zeros(shape=(numpoints, 3))

    if space == 'joint' or space == 'cartesian':
        if type == 'quintic':
            for i in range(3):
                coeffs = quinticTraj(T, start['pos'][i], end['pos'][i],
                                     start['vel'][i], end['vel'][i],
                                     start['acc'][i], end['acc'][i])
                p, v, a = trajPoints(coeffs, T, numpoints, type="quintic")
                pos[:, i] = p
                vel[:, i] = v
                acc[:, i] = a
        elif type == 'cubic':
            for i in range(3):
                coeffs = cubicTraj(T, start['pos'][i], end['pos'][i],
                                   start['vel'][i], end['vel'][i])
                p, v, a = trajPoints(coeffs, T, numpoints, type="cubic")
                pos[:, i] = p
                vel[:, i] = v
                acc[:, i] = a
        else:
            print("not a valid trajectory type")
    else:
        print("Invalid space type. Use 'joint' or 'cartesian'.")

    return pos, vel, acc

# Part 6: Fame.py library (simplified for this context)
class Frame:
    def __init__(self, rotation=None, translation=None):
        if rotation is None:
            self.rotation = np.eye(3)
        else:
            self.rotation = rotation
        if translation is None:
            self.translation = np.zeros(3)
        else:
            self.translation = translation

    def to_homogeneous(self):
        T = np.eye(4)
        T[:3, :3] = self.rotation
        T[:3, 3] = self.translation
        return T

    def set_rotation_from_axis_angle(self, axis, angle):
        axis = axis / np.linalg.norm(axis)
        c = np.cos(angle)
        s = np.sin(angle)
        t = 1 - c
        x, y, z = axis
        R = np.array([[t*x*x + c,    t*x*y - s*z, t*x*z + s*y],
                      [t*x*y + s*z,  t*y*y + c,   t*y*z - s*x],
                      [t*x*z - s*y,  t*y*z + s*x, t*z*z + c]])
        self.rotation = R

    def set_rotation_from_quaternion(self, q):
        q = q / np.linalg.norm(q)
        q0, q1, q2, q3 = q
        R = np.array([[1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2)],
                      [2*(q1*q2 + q0*q3),     1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
                      [2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1),     1 - 2*(q1**2 + q2**2)]])
        self.rotation = R

    def to_axis_angle(self):
        R = self.rotation
        angle = np.arccos((np.trace(R) - 1) / 2)
        if angle == 0:
            axis = np.array([1, 0, 0])
        else:
            rx = R[2,1] - R[1,2]
            ry = R[0,2] - R[2,0]
            rz = R[1,0] - R[0,1]
            axis = np.array([rx, ry, rz]) / (2*np.sin(angle))
        return axis, angle

    def to_quaternion(self):
        R = self.rotation
        q0 = 0.5 * np.sqrt(1 + np.trace(R))
        q1 = (R[2,1] - R[1,2]) / (4*q0)
        q2 = (R[0,2] - R[2,0]) / (4*q0)
        q3 = (R[1,0] - R[0,1]) / (4*q0)
        q = np.array([q0, q1, q2, q3])
        return q

def calc_rot_error(R_d, R_e):
    R_err = R_d.T @ R_e
    axis, angle = Frame(rotation=R_err).to_axis_angle()
    return axis, angle

if __name__ == "__main__":

    # PART (4c): Generate a plot for cubic trajectory
    # Input variables
    T = 5.0  # time
    stp = np.pi/3  # start point
    edp = np.pi    # end point
    vel_s = 0.0    # start velocity
    vel_f = 0.0    # end velocity

    coeffs = cubicTraj(T, stp, edp, vel_s, vel_f)
    numpoints = 100
    pos, vel, acc = trajPoints(coeffs, T, numpoints, type="cubic")

    # Do some plotting!
    t = np.linspace(0, T, numpoints)
    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(t, pos)
    plt.title('Cubic Trajectory - Position')
    plt.subplot(3,1,2)
    plt.plot(t, vel)
    plt.title('Cubic Trajectory - Velocity')
    plt.subplot(3,1,3)
    plt.plot(t, acc)
    plt.title('Cubic Trajectory - Acceleration')
    plt.tight_layout()
    plt.show()

    # PART (4f): Generate a plot for quintic trajectory
    # Input variables
    T = 5.0  # time
    stp = np.pi/3  # start point
    edp = np.pi    # end point
    vel_s = 0.0    # start velocity
    vel_f = 0.0    # end velocity
    acc_s = 0.0    # start acceleration
    acc_f = 0.0    # end acceleration

    coeffs = quinticTraj(T, stp, edp, vel_s, vel_f, acc_s, acc_f)
    numpoints = 100
    pos, vel, acc = trajPoints(coeffs, T, numpoints, type="quintic")

    # Do some plotting!
    t = np.linspace(0, T, numpoints)
    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(t, pos)
    plt.title('Quintic Trajectory - Position')
    plt.subplot(3,1,2)
    plt.plot(t, vel)
    plt.title('Quintic Trajectory - Velocity')
    plt.subplot(3,1,3)
    plt.plot(t, acc)
    plt.title('Quintic Trajectory - Acceleration')
    plt.tight_layout()
    plt.show()

    # PART (5b): Generate a 3D plot showing the requested trajectory
    # Define start and end positions, velocities, accelerations
    start = {
        'pos': np.array([0.0, 0.0, 0.0]),
        'vel': np.array([0.0, 0.0, 0.0]),
        'acc': np.array([0.0, 0.0, 0.0])
    }
    end = {
        'pos': np.array([5.5, 3.0, 7.5]),
        'vel': np.array([0.0, 0.0, 0.0]),
        'acc': np.array([0.0, 0.0, 0.0])
    }
    T = 5.0
    numpoints = 100
    pos, vel, acc = traj3D(start, end, T, numpoints, type="quintic", space="cartesian")

    # Plot the trajectory in 3D
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos[:,0], pos[:,1], pos[:,2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Positional Trajectory')
    plt.show()

    # PART (6b): Create and manipulate frames
    # Create a frame rotated 20 degrees about x and 25 degrees about z, translated to (1,3,5)
    angle_x = np.deg2rad(20)
    angle_z = np.deg2rad(25)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(angle_x), -np.sin(angle_x)],
                   [0, np.sin(angle_x),  np.cos(angle_x)]])
    Rz = np.array([[np.cos(angle_z), -np.sin(angle_z), 0],
                   [np.sin(angle_z),  np.cos(angle_z), 0],
                   [0, 0, 1]])
    R = Rz @ Rx  # Combined rotation
    translation = np.array([1, 3, 5])
    frame = Frame(rotation=R, translation=translation)
    T = frame.to_homogeneous()
    print("Homogeneous Transform:\n", T)

    # Convert rotation matrix to quaternion and axis-angle
    q = frame.to_quaternion()
    print("Quaternion:", q)
    axis, angle = frame.to_axis_angle()
    print("Axis:", axis)
    print("Angle (radians):", angle)

    # Confirm conversion back to original rotation matrix
    frame_from_quat = Frame()
    frame_from_quat.set_rotation_from_quaternion(q)
    print("Rotation from Quaternion:\n", frame_from_quat.rotation)

    frame_from_axis_angle = Frame()
    frame_from_axis_angle.set_rotation_from_axis_angle(axis, angle)
    print("Rotation from Axis-Angle:\n", frame_from_axis_angle.rotation)

    # PART (6c): Calculate rotational error between frames
    static_frame = Frame()
    angles = np.deg2rad(np.linspace(2, 100, num=50))
    errors = []
    for angle in angles:
        rotated_frame = Frame()
        rotated_frame.set_rotation_from_axis_angle(np.array([0, 0, 1]), angle)
        axis_err, angle_err = calc_rot_error(static_frame.rotation, rotated_frame.rotation)
        errors.append(np.rad2deg(angle_err))

    # Plot the rotational errors
    plt.figure()
    plt.plot(np.rad2deg(angles), errors)
    plt.xlabel('Rotation Applied (degrees)')
    plt.ylabel('Calculated Rotational Error (degrees)')
    plt.title('Rotational Error vs Applied Rotation')
    plt.show()
