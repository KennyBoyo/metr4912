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
        # print(s)
        return np.eye(3)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    # Rodrigues' formula
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix


def get_perpendicular_vectors(v0):
    v0 = v0/np.linalg.norm(v0)
    v1 = np.random.randn(3)
    v1 -= v1.dot(v0) * v0
    v1 /= np.linalg.norm(v1)
    v2 = np.cross(v0, v1)
    v2 /= np.linalg.norm(v2)
    # print(v1, v2)
    return v1, v2

def get_stiffness_matrix(v0, mag, ax):
    v1, v2 = get_perpendicular_vectors(v0)
    v0 = v0/np.linalg.norm(v0)
    v1 = v1/np.linalg.norm(v1)
    v2 = v2/np.linalg.norm(v2)
    # k_mat = np.zeros((3,3))
    # k_mat[0][0] = mag
    # k_mat[1][1] = mag/3
    # k_mat[2][2] = mag/3

    Q = np.zeros((3, 3))
    Q[:, 0] = v0
    Q[:, 1] = v1
    Q[:, 2] = v2
    E = np.zeros((3,3))
    E[0, 0] = mag
    E[1, 1] = mag/10
    E[2, 2] = mag/10

    # rot_v0 = rotation_matrix_from_vectors(k_mat[:, 0], v0)
    # rot_v1 = rotation_matrix_from_vectors(k_mat[:, 1], v1)
    # rot_v2 = rotation_matrix_from_vectors(k_mat[:, 2], v2)
    # print(rot_v0)


    # print(k_mat[:, 0])
    # print(k_mat[:, 1])
    # print(k_mat[:, 2])
    # r0 = rot_v0 @ v0 
    # r1 = rot_v0 @ v1
    # r2 = rot_v0 @ v2

    # k_mat[:, 0] = r0.T
    # k_mat[:, 1] = r1.T
    # k_mat[:, 2] = r2.T


    # print(v0)
    # print(v1)
    # print(v2)

    # print(Q)
    # print(E)
    print(E)
    print(Q)
    cov = Q @ E @ Q.T
    # print(cov)

    # fig = plt.figure(figsize=plt.figaspect(1))  # Square figure
    # ax = fig.add_subplot(111, projection='3d')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # ax.set_xlim3d(-10,10)
    # ax.set_ylim3d(-10,10)
    # ax.set_zlim3d(-10,10)
    # # ax.plot3D(line_x, line_y, line_z, color='r')

     
    # ax.quiver(0, 0, 0, v0[0], v0[1], v0[2], length = 5)
    # ax.quiver(0, 0, 0, v1[0], v1[1], v1[2], length = 5)
    # ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], length = 5)


    # for i in range(3):
    #     # ax.quiver(0, 0, 0, k_mat[i][0], k_mat[i][1], k_mat[i][2], length = 1)
    #     # ax.quiver(0, 0, 0, E[i][i]*Q[i][0], E[i][i]*Q[i][1], E[i][i]*Q[i][2], length = 1)
    #     ax.quiver(0, 0, 0, E[i][i]*Q[0][i], E[i][i]*Q[1][i], E[i][i]*Q[2][i], length = 1)
    # # # plt.show()
    
    # i = np.eye(3)
    # i[0, 1] = 3
    # print(i)
    # w, v = np.linalg.eig(i)
    # print(w)
    # print(v)

    w, v = np.linalg.eig(cov)
    print(w)
    print(v)

    return cov

    

#### ELLIPSOID
ellipseSteps= 100
u = np.linspace(0, 2 * np.pi, ellipseSteps)
v = np.linspace(0, np.pi, ellipseSteps)

mag = 10

# Transformation formulae for a spherical coordinate system.
x = mag * np.outer(np.cos(u), np.sin(v))
y = mag/10 * np.outer(np.sin(u), np.sin(v))
z = mag/10 * np.outer(np.ones_like(u), np.cos(v))

vec = np.array([10, 10, 100])
line_x = [0, vec[0]]
line_y = [0, vec[1]]
line_z = [0, vec[2]]


fig = plt.figure(figsize=plt.figaspect(1))  # Square figure
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(line_x, line_y, line_z, color='r')

vec2 = np.array([1, 0, 0])
line2_x = np.array([0, vec2[0]])
line2_y = np.array([0, vec2[1]])
line2_z = np.array([0, vec2[2]])
ax.plot3D(line2_x, line2_y, line2_z, color='b')


agg = np.array([line2_x, line2_y, line2_z])
agg = agg.reshape(3, -1)
# print(agg)
# phi, theta = getRotAnglesYZ(vec)
# print(phi, theta)
# rotated = rotYZ(phi, theta) @ agg

rotation = rotation_matrix_from_vectors(vec2, vec)
rotated = rotation.T @ agg
rotated_x = rotated[0]
rotated_y = rotated[1]
rotated_z = rotated[2]
ax.plot3D(rotated_x, rotated_y, rotated_z, color='b')

#### ROTATED
agg = np.array([x, y, z])
agg = agg.reshape(3, -1)

rotation = rotation_matrix_from_vectors(vec2, vec)
rotated = rotation @ agg

rotated = rotated.reshape(3, 100, 100)
rotated_x = rotated[0]
rotated_y = rotated[1]
rotated_z = rotated[2]

mag = 10
k_mat = np.zeros((3,3))
k_mat[0][0] = 10
k_mat[1][1] = 1
k_mat[2][2] = 1
# print(rotation)
# print(k_mat)

# k_mat = np.linalg.inv(k_mat.T) @ rotation @ np.linalg.inv(k_mat)

k_mat = rotation @ k_mat @ rotation.T

# print(k_mat.T @ rotation @ k_mat)
# print(rotation.T @ k_mat @ rotation)
# k_mat = get_stiffness_matrix(vec, mag, ax)
# print(get_stiffness_matrix(vec, mag))
# print(k_mat)
# w, v = np.linalg.eig(k_mat)
# print(w, v)
# # print(k_mat)
# for i in range(3):
#     # ax.quiver(0, 0, 0, k_mat[i][0], k_mat[i][1], k_mat[i][2], length = 1)
#     ax.quiver(0, 0, 0, k_mat[0][i], k_mat[1][i], k_mat[2][i], length = 1)


w, v = np.linalg.eig(k_mat)
print(w)
print(v)
print(k_mat)
for i in range(3):
    # ax.quiver(0, 0, 0, k_mat[i][0], k_mat[i][1], k_mat[i][2], length = 1)
    ax.quiver(0, 0, 0, w[i]*v[0][i], w[i]*v[1][i], w[i]*v[2][i], length = 10)
    # ax.quiver(0, 0, 0, w[i]*v[i][0], w[i]*v[i][1], w[i]*v[i][2], length = 10)

ax.plot_surface(rotated_x, rotated_y, rotated_z, color='b')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim3d(-10,10)
ax.set_ylim3d(-10,10)
ax.set_zlim3d(-10,10)

plt.show()
