import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Read the CSV file into a DataFrame
df = pd.read_csv('Kalipen_positions_and_robot_peak_position.csv')

# Extract x, y, and z coordinates from the DataFrame
x_rob = df['x_robot']
y_rob = df['y_robot']
z_rob = df['z_robot']
x_kalip = df['x_kaliepn']
y_kalip = df['y_kalipen']
z_kalip = df['z_kalipen']

x_rob_mean = np.mean(x_rob)
y_rob_mean = np.mean(y_rob)
z_rob_mean = np.mean(z_rob)

dx = []
dy = []
dz = []
dist = []
print("N: " + str(len(x_kalip)))
for i in range(len(x_kalip)):
    dx.append(x_kalip[i] - x_rob_mean )
    dy.append(y_kalip[i] - y_rob_mean )
    dz.append(z_kalip[i] - z_rob_mean )
    dist.append(np.sqrt(dx[i]**2 + dy[i]**2 + dz[i]**2))
    
print(dx)
print(dy)
print(dz)
print(dist)
print("dx_mean:" + str(np.mean(dx)))
print("dy_mean:" + str(np.mean(dy)))
print("dz_mean:" + str(np.mean(dz)))
print("dist_mean:" + str(np.mean(dist)))
# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_kalip, y_kalip, z_kalip, c='b', marker='o')
ax.scatter(x_rob_mean, y_rob_mean, z_rob_mean, c='r', marker='o')




# Set labels and title
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
ax.set_title('3D Scatter Plot')

#print("dx: {}, dy: {}, dz: {}".format((max(x)-min(x)), (max(y)-min(y)), (max(z)-min(z))))

plt.show()