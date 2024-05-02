
from transforms3d.quaternions import mat2quat, quat2mat
import numpy as np

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped, Pose
from scipy.optimize import fmin, minimize



def pose_to_T(pose):
    # pose to HTM 
    q1 = pose.orientation.x
    q2 = pose.orientation.y
    q3 = pose.orientation.z
    q4 = pose.orientation.w
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    R = quat2mat([q4, q1, q2, q3])
    
    T = np.zeros([4, 4])
    T[:3, :3] = R
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    T[3, 3] = 1
    return T
def f_optimizePointOrientation(T_init, callibration_data_poses):

    
    R1 = T_init[:3, :3]  # Initial rotation matrix
    P1 = T_init[:3, 3]   # Initial position vector

    # Optimize position using fmin
    P1_optimized = fmin(func=objectiveFunPosition, x0=np.array([0,0,0.06]), args=(R1, callibration_data_poses), xtol=1e-20, ftol=1e-20,  disp=True)

    print("optimization finished")
    print(P1_optimized)
    T = np.eye(4)
    T[:3, 3] = P1_optimized

    return T
def CalculateCalipenTransformation(callibration_data_poses):
        T_init = np.eye(4, 4)
        T_init[2,3] = 0.06
        print(T_init)
        T = f_optimizePointOrientation(T_init, callibration_data_poses)
        return T
    
def objectiveFunPosition(p, R, T_data):
    """
    Optimization for position
    """
    T0A = T_data.copy()

    #T0A = select_points(T0A)
    # Initial supposed transform
    Tx = np.eye(4)
    Tx[:3, :3] = R
    Tx[:3, 3] = p

    num_of_mesurs = len(T0A)  # holds the number of measurements
    f = 0  # optimization function value initialized to 0
    for i in range(0, num_of_mesurs):
        T_OI = np.matmul(T0A[i], Tx) 
        for j in range(0, num_of_mesurs):
            T_OJ = np.matmul(T0A[j], Tx) 
            p_ = T_OI[:3, 3] - T_OJ[:3, 3]
            f += np.linalg.norm(p_)  # Euclidean distance
    print(f)
    return f

# Read the CSV file into a DataFrame
df = pd.read_csv('robot_EF_poses.csv')
# Extract x, y, and z coordinates from the DataFrame
x = df['x']
y = df['y']
z = df['z']
qx = df['qx']
qy = df['qy']
qz = df['qz']
qw = df['qw']
print(x[0])
Poses = []
HTMS = []
for i in range(len(x)):
    poza = Pose()
    poza.position.x = x[i]
    poza.position.y = y[i]
    poza.position.z = z[i]
    poza.orientation.x = qx[i]
    poza.orientation.y = qy[i]
    poza.orientation.z = qz[i]
    poza.orientation.w = qw[i]
    Poses.append(poza)
    HTMS.append(pose_to_T(poza))

print(Poses) 
HTM_markers_tip = CalculateCalipenTransformation(HTMS)
