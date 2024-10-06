import numpy as np
import matplotlib.pyplot as plt

"""
Author: J. Micah Prendergast
Date: 01-10-2024

This is a library for creating, plotting and manipulating reference frames. It contains the Frame class
along with various methods you can call to on these frames to translate them, rotate them, copy them, etc.
Feel free to play around with some of these frames and see how they work before. There are a few functions
you will need to modify to get working as I have indicated in your assignment. The functions themselves are
present but they will not currently run so you can build on that skeleton to get them working. Note: You may 
want to comment these functions out at first so you can play around with the most basic components even before
you get everything working.

Below the Frame class,I have also included a few functions for plotting both the frames and vectors between 
frames. In addition there are several function for working with quaternions. You won't need to use these quaternion 
functions for your assignment, but they may be fun to play around with, particularly the slerp function. We will
discuss SLERP in class but we won't have time for you to implement it yourself so I have given you my basic 
implementation in case you want to try it out in combination with other trajectory planners. 

At the bottom I have made a few examples of basic functionality so that you can see how you might use some 
of the methods within the Frame class.

"""

class Frame:
    def __init__(self, name, size=1.0):
        """
        Initialize a Frame object.

        Parameters:
        - name: str, the name of the frame.
        - size: float, the size of the frame's axes for plotting.
        """
        self.name = name
        self.size = size
        self.matrix = np.identity(4)

    def set_transform(self, matrix):
        """
        Set the transformation matrix of the frame.

        Parameters:
        - matrix: 4x4 numpy array, the transformation matrix.
        """
        self.matrix = matrix

    def translate(self, dx=0, dy=0, dz=0, relative_to='origin'):
        """
        Translate the frame.

        Parameters:
        - dx, dy, dz: floats, the translation along x, y, z axes.
        - relative_to: 'origin' or Frame object, the reference frame.
        """
        t = np.identity(4)
        t[0:3, 3] = [dx, dy, dz]
        if relative_to == 'origin':
            self.matrix = t @ self.matrix
        elif isinstance(relative_to, Frame):
            self.matrix = relative_to.matrix @ t @ np.linalg.inv(relative_to.matrix) @ self.matrix
        else:
            raise ValueError("relative_to must be 'origin' or another Frame object")

    def rotate(self, axis, angle_in_rad, relative_to='origin'):
        """
        Rotate the frame.

        Parameters:
        - axis: 'x', 'y', or 'z', the axis of rotation.
        - angle_in_rad: float, the rotation angle in radians.
        - relative_to: 'origin' or Frame object, the reference frame.
        """
        c = np.cos(angle_in_rad)
        s = np.sin(angle_in_rad)
        r = np.identity(4)
        if axis == 'x':
            r[1,1] = c
            r[1,2] = -s
            r[2,1] = s
            r[2,2] = c
        elif axis == 'y':
            r[0,0] = c
            r[0,2] = s
            r[2,0] = -s
            r[2,2] = c
        elif axis == 'z':
            r[0,0] = c
            r[0,1] = -s
            r[1,0] = s
            r[1,1] = c
        else:
            raise ValueError("Axis must be 'x', 'y', or 'z'")
        if relative_to == 'origin':
            self.matrix = r @ self.matrix
        elif isinstance(relative_to, Frame):
            t_rel = relative_to.matrix
            self.matrix = t_rel @ r @ np.linalg.inv(t_rel) @ self.matrix
        else:
            raise ValueError("relative_to must be 'origin' or another Frame object")

    def copy(self, new_name=None):
        """
        Create a copy of the frame at its current location and orientation.

        Parameters:
        - new_name: str, optional, the name of the new frame. If not provided, '_copy' is appended to the original name.

        Returns:
        - Frame object, the copied frame.
        """
        if new_name is None:
            new_name = self.name + '_copy'
        new_frame = Frame(new_name, self.size)
        new_frame.matrix = np.copy(self.matrix)
        return new_frame

    def print_matrix(self):
        """
        Print the full 4x4 transformation matrix of the frame.
        """
        print(f"Transformation matrix of frame '{self.name}':")
        print(self.matrix)

    def to_axis_angle(self):
        """
        Convert the rotation matrix of the frame to axis-angle representation.

        Returns:
        - axis: numpy array of shape (3,), the rotation axis (unit vector).
        - angle: float, the rotation angle in radians.
        """
        R = self.matrix[0:3, 0:3]
        #Include the algorithm for computing the axis and angle from a rotation matrix

        return #axis and angle

    def to_quaternion(self):
        """
        Convert the rotation matrix of the frame to quaternion representation.

        Returns:
        - q: numpy array of shape (4,), the quaternion [w, x, y, z]
        """
        R = self.matrix[0:3, 0:3]

        # Compute the components of the quaternion
        #
        #
        #

        q = np.array([qw, qx, qy, qz]) #put the pieces together
        # Normalize the quaternion
        q = q / np.linalg.norm(q)
        return q

    def set_rotation_from_quaternion(self, quaternion):
        """
        Set the rotation matrix of the frame from a quaternion.

        Parameters:
        - quaternion: array-like of shape (4,), the quaternion [w, x, y, z]
        """
        qw, qx, qy, qz = quaternion

        # Normalize the quaternion
        norm = np.linalg.norm(quaternion)

        qw, qx, qy, qz = quaternion / norm
        # Compute the rotation matrix
        R = np.array([ ])
        self.matrix[0:3, 0:3] = R

    def set_rotation_from_axis_angle(self, axis, angle):
        """
        Set the rotation matrix of the frame from an axis-angle representation.

        Parameters:
        - axis: array-like of shape (3,), the rotation axis (should be a unit vector).
        - angle: float, the rotation angle in radians.
        """
        axis = np.asarray(axis)
        if np.linalg.norm(axis) == 0:
            raise ValueError("Zero-length axis vector")
        axis = axis / np.linalg.norm(axis)  # Ensure the axis is a unit vector
        #
        #
        #
        #
        #

        # Compute the rotation matrix
        R = np.array([])
        self.matrix[0:3, 0:3] = R

    def calc_rot_error(self, frame):
        """
        Calculate the difference or error between the current frame
        and some other reference frame

        Parameters:
        -frame: reference frame (must be another frame object)

        Returns:
        -error in radians as numpy array of shape (3,).
        """

        return err





    def __repr__(self):
        """
        String representation of the Frame object.
        """
        return f"Frame(name='{self.name}', size={self.size})"

# Additional functions
def quaternion_mul(q1, q2):
    """
    Multiply two quaternions.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.array([w, x, y, z])

def quaternion_conj(q):
    """
    Calculate the conjugate of a quaternion.
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])

def relative_quat(q_1, q_2):
    q_1_conj = np.array([q_1[0], -q_1[1], -q_1[2], -q_1[3]])
    q_rel = quaternion_mul(q_1_conj, q_2)
    return (q_rel / np.linalg.norm(q_rel))

def slerp(q0, q1, t):
    """
    Perform Spherical Linear Interpolation between two quaternions.
    """
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # Quaternions are close; use linear interpolation
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta_t = theta_0 * t
    sin_theta_t = np.sin(theta_t)
    s0 = np.sin(theta_0 - theta_t) / sin_theta_0
    s1 = sin_theta_t / sin_theta_0
    result = s0 * q0 + s1 * q1
    return result

def plot_frames(frames, ax=None, show=True, equal_axes=True):
    """
    Plot multiple frames in 3D.

    Parameters:
    - frames: list of Frame objects to plot.
    - ax: matplotlib axes object. If None, a new figure and axes are created.
    - show: bool, whether to display the plot immediately.
    - equal_axes: bool, whether to set equal scaling for all axes.
    """
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    else:
        fig = plt.gcf()

    for frame in frames:
        origin = frame.matrix[0:3, 3]
        x_axis = frame.matrix @ np.array([frame.size, 0, 0, 1])
        y_axis = frame.matrix @ np.array([0, frame.size, 0, 1])
        z_axis = frame.matrix @ np.array([0, 0, frame.size, 1])

        ax.quiver(origin[0], origin[1], origin[2],
                  x_axis[0]-origin[0], x_axis[1]-origin[1], x_axis[2]-origin[2],
                  color='r', length=frame.size, normalize=False)
        ax.quiver(origin[0], origin[1], origin[2],
                  y_axis[0]-origin[0], y_axis[1]-origin[1], y_axis[2]-origin[2],
                  color='g', length=frame.size, normalize=False)
        ax.quiver(origin[0], origin[1], origin[2],
                  z_axis[0]-origin[0], z_axis[1]-origin[1], z_axis[2]-origin[2],
                  color='b', length=frame.size, normalize=False)

    if equal_axes:
        # Set equal aspect ratio for all axes
        limits = np.array([ax.get_xlim(), ax.get_ylim(), ax.get_zlim()])
        min_limit = limits[:,0].min()
        max_limit = limits[:,1].max()
        ax.set_xlim(min_limit, max_limit)
        ax.set_ylim(min_limit, max_limit)
        ax.set_zlim(min_limit, max_limit)
        ax.set_box_aspect([1,1,1])  # For matplotlib >= 3.3.0

    if show:
        plt.show()
    else:
        return fig, ax

def draw_vector(frame_from, frame_to, color='k', ax=None):
    """
    Draw a connecting vector (arrow) from one frame to another.

    Parameters:
    - frame_from: Frame object, the starting frame.
    - frame_to: Frame object, the ending frame.
    - color: str, color of the vector.
    - ax: matplotlib axes object.
    """
    if ax is None:
        ax = plt.gca()
    origin = frame_from.matrix[0:3, 3]
    target = frame_to.matrix[0:3, 3]
    delta = target - origin
    ax.quiver(origin[0], origin[1], origin[2],
              delta[0], delta[1], delta[2],
              color=color, arrow_length_ratio=0.1, linewidth=1)




# Just some examples for creating and plotting frames

# Create the origin frame and a few other frames
origin_frame = Frame('Origin', size=1.0)
frame_test = Frame('test frame', size=0.75)
frame_ref = frame_test.copy()

#Move some things around
frame_test.rotate('z', np.deg2rad(90), relative_to=origin_frame)
frame_test.translate(dx=1.5, relative_to='origin')
frame_ref.translate(dx=1, relative_to='origin')

#Print some things
frame_ref.print_matrix()
frame_test.print_matrix()

#Make a new frame and give it the homogenous transformation matrix
#of an existing frame
newFrame = Frame('new frame', size=0.75)
newFrame.set_transform(frame_test.matrix)
newFrame.print_matrix()

#Plot the frames
frames = [origin_frame, frame_test, frame_ref]
fig, ax = plot_frames(frames, show=False)
plt.show()