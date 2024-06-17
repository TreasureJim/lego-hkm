from mpl_toolkits.mplot3d import axes3d
import numpy as np
import matplotlib.pyplot as plt

# Load data from file (adjust the path as per your file location)
data = np.loadtxt('matrix.txt')

# Create a new figure
fig = plt.figure()

# Add 3D subplot
ax = fig.add_subplot(111, projection='3d')

X = data[:, 0]
Y = data[:, 1]
Z = data[:, 2]

# Plot the data
ax.plot(X, Y, Z, '.-')

# Set labels for each axis
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

# Show the plot
plt.show()
