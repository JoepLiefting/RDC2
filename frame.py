import numpy as np
import matplotlib.pyplot as plt

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
        epsilon = 1e-6

        tr = np.trace(R)
        cos_theta = (tr - 1) / 2
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        theta = np.arccos(cos_theta)
        sin_theta = np.sqrt(1 - cos_theta**2)

        if sin_theta > epsilon:
            rx = (R[2,1] - R[1,2]) / (2*sin_theta)
            ry = (R[0,2] - R[2,0]) / (2*sin_theta)
            rz = (R[1,0] - R[0,1]) / (2*sin_theta)
            axis = np.array([rx, ry, rz])
            axis = axis / np.linalg.norm(axis)
        else:
            # Angle is close to 0, axis not well-defined
            axis = np.array([0, 0, 0])

        return axis, theta

    def to_quaternion(self):
        """
        Convert the rotation matrix of the frame to quaternion representation.

        Returns:
        - q: numpy array of shape (4,), the quaternion [w, x, y, z]
        """
        R = self.matrix[0:3, 0:3]
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2  # S=4*qw
            qw = 0.25 * S
            qx = (R[2,1] - R[1,2]) / S
            qy = (R[0,2] - R[2,0]) / S
            qz = (R[1,0] - R[0,1]) / S
        else:
            if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
                S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2  # S=4*qx
                qw = (R[2,1] - R[1,2]) / S
                qx = 0.25 * S
                qy = (R[0,1] + R[1,0]) / S
                qz = (R[0,2] + R[2,0]) / S
            elif R[1,1] > R[2,2]:
                S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2  # S=4*qy
                qw = (R[0,2] - R[2,0]) / S
                qx = (R[0,1] + R[1,0]) / S
                qy = 0.25 * S
                qz = (R[1,2] + R[2,1]) / S
            else:
                S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2  # S=4*qz
                qw = (R[1,0] - R[0,1]) / S
                qx = (R[0,2] + R[2,0]) / S
                qy = (R[1,2] + R[2,1]) / S
                qz = 0.25 * S

        q = np.array([qw, qx, qy, qz])
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
        if norm == 0:
            raise ValueError("Zero-length quaternion")
        qw, qx, qy, qz = quaternion / norm

        # Compute the rotation matrix
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]
        ])
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
        x, y, z = axis
        c = np.cos(angle)
        s = np.sin(angle)
        C = 1 - c
        # Compute the rotation matrix
        R = np.array([
            [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
            [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
            [z*x*C - y*s,   z*y*C + x*s, c + z*z*C]
        ])
        self.matrix[0:3, 0:3] = R

    def calc_rot_error(self, frame):
        """
        Calculate the rotational error between the current frame
        and a reference frame.

        Parameters:
        - frame: Frame object (reference frame).

        Returns:
        - error: tuple (axis, angle) where:
            - axis: numpy array of shape (3,), the axis of rotation.
            - angle: float, the rotation angle in radians.
        """
        # Step 1: Calculate the relative rotation matrix
        R1 = self.matrix[0:3, 0:3]  # Current frame rotation matrix
        R2 = frame.matrix[0:3, 0:3]  # Reference frame rotation matrix
        R_err = R2 @ R1.T  # Relative rotation matrix

        # Step 2: Compute the angle of rotation (theta)
        cos_theta = (np.trace(R_err) - 1) / 2
        cos_theta = np.clip(cos_theta, -1.0, 1.0)  # Clipping to avoid numerical errors

        # Step 3: Handle small angles with precision issues
        if abs(cos_theta - 1.0) < 1e-6:
            # Angle is too small, approximate as zero (no rotation)
            theta = 0.0
            axis = np.array([0, 0, 0])  # No meaningful rotation axis
        else:
            theta = np.arccos(cos_theta)  # Angle in radians

            # Step 4: Compute the axis of rotation (r) only if theta is large enough
            if np.sin(theta) > 1e-6:  # Avoid division by zero
                rx = (R_err[2, 1] - R_err[1, 2]) / (2 * np.sin(theta))
                ry = (R_err[0, 2] - R_err[2, 0]) / (2 * np.sin(theta))
                rz = (R_err[1, 0] - R_err[0, 1]) / (2 * np.sin(theta))
                axis = np.array([rx, ry, rz])
                axis = axis / np.linalg.norm(axis)  # Normalize the axis
            else:
                # Angle is very small, axis not well-defined
                axis = np.array([0, 0, 0])

        return axis, theta

# Task (b)
# Create a frame that is rotated 20 degrees about x and 25 degrees in z
# and is then translated from the origin to point (1,3,5).
frame_b = Frame('Frame B')
frame_b.rotate('x', np.deg2rad(20), relative_to='origin')
frame_b.rotate('z', np.deg2rad(25), relative_to='origin')
frame_b.translate(dx=1, dy=3, dz=5, relative_to='origin')

# Compute the homogeneous transform
print("Homogeneous Transformation Matrix of Frame B:")
frame_b.print_matrix()

# Convert its rotation matrix to quaternion
q = frame_b.to_quaternion()
print("\nQuaternion representation of Frame B rotation:")
print(q)

# Convert its rotation matrix to axis and angle
axis, angle = frame_b.to_axis_angle()
print("\nAxis-Angle representation of Frame B rotation:")
print(f"Axis: {axis}")
print(f"Angle (radians): {angle}")
print(f"Angle (degrees): {np.degrees(angle)}")

# Confirm that you can also convert back to your original homogeneous transform.
# Using quaternion
frame_q = Frame('Frame from Quaternion')
frame_q.set_rotation_from_quaternion(q)
frame_q.translate(dx=1, dy=3, dz=5, relative_to='origin')

print("\nFrame from Quaternion Transformation Matrix:")
frame_q.print_matrix()

# Using axis-angle
frame_aa = Frame('Frame from Axis-Angle')
frame_aa.set_rotation_from_axis_angle(axis, angle)
frame_aa.translate(dx=1, dy=3, dz=5, relative_to='origin')

print("\nFrame from Axis-Angle Transformation Matrix:")
frame_aa.print_matrix()

# Verify that the matrices are the same
print("\nDifference between original Frame B and Frame from Quaternion:")
print(frame_b.matrix - frame_q.matrix)

print("\nDifference between original Frame B and Frame from Axis-Angle:")
print(frame_b.matrix - frame_aa.matrix)

# Task (c)
# Test calc_rot_error() with a range of different rotations from 2 degrees to 100 degrees
angles_deg = np.arange(2, 101, 2)
errors = []

frame_static = Frame('Static Frame')

for angle_deg in angles_deg:
    frame_rotated = Frame('Rotated Frame')
    frame_rotated.rotate('z', np.deg2rad(angle_deg), relative_to='origin')
    axis, error_angle = frame_rotated.calc_rot_error(frame_static)
    errors.append(np.degrees(error_angle))  # Store the error in degrees

# Plot the rotational error
plt.figure()
plt.plot(angles_deg, errors, 'b-o', label='Computed Rotational Error')
plt.xlabel('Rotation Angle (degrees)')
plt.ylabel('Rotational Error (degrees)')
plt.title('Rotational Error vs Rotation Angle')
plt.legend()
plt.grid(True)
plt.show()

# Plot the absolute difference between computed error and expected rotation angle
expected_errors_deg = angles_deg % 360
difference = np.abs(errors - expected_errors_deg)

plt.figure()
plt.plot(angles_deg, difference, 'm-o')
plt.xlabel('Rotation Angle (degrees)')
plt.ylabel('Absolute Error (degrees)')
plt.title('Absolute Difference Between Computed Error and Actual Rotation Angle')
plt.grid(True)
plt.show()

# Analysis of accuracy
print("Maximum absolute error in degrees:", np.max(difference))
print("Mean absolute error in degrees:", np.mean(difference))
