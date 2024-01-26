import numpy as np
from transforms3d import quaternions

# Your 4x4 transformation matrix
matrix = np.array([[-0.0106301, -0.99817292, 0.04285442, 0.63936765],
                   [-0.99986436, -0.04318853, -0.01302531, 0.34624031],
                   [0.01258062, -0.04225599, -0.99899642, 0.12477428],
                   [0., 0., 0., 1.]])

# Extract the upper-left 3x3 matrix (rotation component)
rotation_matrix = matrix[:3, :3]

# Convert the rotation matrix to a quaternion
quaternion = quaternions.mat2quat(rotation_matrix)

print("Quaternion:", quaternion)
