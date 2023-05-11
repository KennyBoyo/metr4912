from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

def rot_mat(angle, dir):
    if dir == 'x':
        return np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]])
    if dir == 'y':
        return np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
    if dir == 'z':
        return np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    return

def rot(rot_x, rot_y, rot_z):
    return rot_mat(rot_z, 'z') @ rot_mat(rot_y, 'y') @ rot_mat(rot_x, 'x')

def getAnglesFromAxes(vec):
    return np.arctan2(vec[1], vec[0]), np.arctan2(vec[2], np.linalg.norm([vec[0], vec[1]])), np.arctan2(vec[2], vec[1]) 

def rotYZ(rot_y, rot_z):
    return rot_mat(rot_z, 'z') @ rot_mat(rot_y, 'y')

def getRotAnglesYZ(vec):
    return -np.arctan2(vec[2], np.linalg.norm([vec[0], vec[1]])), np.arctan2(vec[2], vec[1]) 

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
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

#### ELLIPSOID
ellipseSteps= 100
u = np.linspace(0, 2 * np.pi, ellipseSteps)
v = np.linspace(0, np.pi, ellipseSteps)

# Transformation formulae for a spherical coordinate system.
x = 10 * np.outer(np.cos(u), np.sin(v))
y = 5 * np.outer(np.sin(u), np.sin(v))
z = 5 * np.outer(np.ones_like(u), np.cos(v))

fig = plt.figure(figsize=plt.figaspect(1))  # Square figure
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(x, y, z, color='b')
ax.set_xlabel('x')
ax.set_ylabel('y') 
ax.set_zlabel('z')
ax.set_xlim3d(-10,10)
ax.set_ylim3d(-10,10)
ax.set_zlim3d(-10,10)


#### ROTATED
print(x.shape)
print(y.shape)
print(z.shape)

agg = np.array([x, y, z])
agg = agg.reshape(3, -1)

rotated = rot(np.pi/4, np.pi/4, np.pi/4) @ agg
rotated = rot(0, -np.pi/4, 0) @ agg
rotated = rot(0, 0, np.pi/4) @ rotated
rotated = rotated.reshape(3, 100, 100)
rotated_x = rotated[0]
rotated_y = rotated[1]
rotated_z = rotated[2]

fig = plt.figure(figsize=plt.figaspect(1))  # Square figure
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(rotated_x, rotated_y, rotated_z, color='b')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim3d(-10,10)
ax.set_ylim3d(-10,10)
ax.set_zlim3d(-10,10)

vec = np.array([10, 10, 10])
line_x = [0, vec[0]]
line_y = [0, vec[1]]
line_z = [0, vec[2]]

ax.plot3D(line_x, line_y, line_z, color='r')


##### LINE

vec = np.array([-10, 10, 10])
line_x = [0, vec[0]]
line_y = [0, vec[1]]
line_z = [0, vec[2]]


fig = plt.figure(figsize=plt.figaspect(1))  # Square figure
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(line_x, line_y, line_z, color='r')

vec2 = np.array([10, 0, 0])
line2_x = np.array([0, vec2[0]])
line2_y = np.array([0, vec2[1]])
line2_z = np.array([0, vec2[2]])
ax.plot3D(line2_x, line2_y, line2_z, color='b')


agg = np.array([line2_x, line2_y, line2_z])
agg = agg.reshape(3, -1)
print(agg)
# phi, theta = getRotAnglesYZ(vec)
# print(phi, theta)
# rotated = rotYZ(phi, theta) @ agg

rotation = rotation_matrix_from_vectors(vec2, vec)
rotated = rotation @ agg
# rotated = rotated.reshape(3, 100, 100)
# rotated = rot(0, -b, 0) @ rotated
# rotated = rotated.reshape(3, 2, 2)
rotated_x = rotated[0]
rotated_y = rotated[1]
rotated_z = rotated[2]
ax.plot3D(rotated_x, rotated_y, rotated_z, color='b')

#### ROTATED
agg = np.array([x, y, z])
agg = agg.reshape(3, -1)
# phi, theta = getRotAnglesYZ(vec)
# print(phi, theta)
# rotated = rotYZ(phi, theta) @ agg

rotation = rotation_matrix_from_vectors(vec2, vec)
rotated = rotation @ agg

rotated = rotated.reshape(3, 100, 100)
rotated_x = rotated[0]
rotated_y = rotated[1]
rotated_z = rotated[2]

ax.plot_surface(rotated_x, rotated_y, rotated_z, color='b')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim3d(-10,10)
ax.set_ylim3d(-10,10)
ax.set_zlim3d(-10,10)

plt.show()
