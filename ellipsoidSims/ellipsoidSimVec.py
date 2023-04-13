from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    if (s == 0):
        return np.eye(3)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

# Generate square figure
fig = plt.figure(figsize=plt.figaspect(1))  # Square figure
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim3d(-10,10)
ax.set_ylim3d(-10,10)
ax.set_zlim3d(-10,10)

# Generate desired vector direction
vec = np.array([5, 10, 0])
line_x = [0, vec[0]]
line_y = [0, vec[1]]
line_z = [0, vec[2]]
# Plot desired vector
ax.plot3D(line_x, line_y, line_z, color='r')

# Generate Original Vector (Can be anything, but currently set to x axis)
vec2 = np.array([10, 0, 0])
line2_x = np.array([0, vec2[0]])
line2_y = np.array([0, vec2[1]])
line2_z = np.array([0, vec2[2]])
# Plot Original Vector
ax.plot3D(line2_x, line2_y, line2_z, color='b')


# Generate Rotation Matrix
rotation = rotation_matrix_from_vectors(vec2, vec)

# Generate weighted covariance matrix (highest weight dependent on original vector direction)
mag = 10
k_mat = np.zeros((3,3))
k_mat[0][0] = 10
k_mat[1][1] = 2
k_mat[2][2] = 2

# Rotate covariance matrix
k_mat = rotation.T @ k_mat @ rotation

# Get Eigenvectors which are the principal directions of the ellipsoid
w, v = np.linalg.eig(k_mat)
print(w)
print(v)
print(k_mat)
# Plot principal directions.
for i in range(3):
    ax.quiver(0, 0, 0, w[i]*v[0][i], w[i]*v[1][i], w[i]*v[2][i], length = 5)

plt.show()