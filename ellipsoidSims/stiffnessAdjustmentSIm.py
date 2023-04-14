from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from queue import Queue
from copy import deepcopy

# Start at with maximum stiffness
# Decrease if vel < threshold and force above threshold
# Increase if vel > threshold

# Increase and Decrease based on direction of force applied multiplied by a constant normalised amount.
# 

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

class adjuster:
	def __init__(self) -> None:
		# self.max = np.array([[200, 200, 200], [200, 200, 200], [200, 200, 200]])
		self.max = np.array([200, 200, 200, 200, 200, 200, 200, 200, 200])
		self.current = deepcopy(self.max)
		self.v_thres_low = 0.0001
		self.v_thres_high = 0.4
		self.f_thres_low = 2
		self.adjustment_queue = []


	def adjust(self, force, dist):
		f_mag = np.linalg.norm(force)
		f_dir = force / f_mag

		vel = dist*30
		
		f_mat = self.get_rotated_ellipsoid(f_dir).reshape(-1)

		# Adjustment matrix is a unit step in the direction of force
		adjustment_matrix = f_mat/np.linalg.norm(f_mat)

		if (vel < self.v_thres_low):
			if (force > self.f_thres_low):
				self.adjustment_queue.append(adjustment_matrix)
				self.current -= adjustment_matrix

			pass
		elif (vel > self.v_thres_high):
			self.current += self.adjustment_queue.pop()
			pass
		else:

			pass



	def get_rotated_ellipsoid(self, dir_vec, mag = 1, normal_scale = 0.1):
		axis = np.array([1, 0, 0])
		rotation = rotation_matrix_from_vectors(axis, dir_vec)

		k_mat = np.zeros((3,3))
		k_mat[0][0] = mag
		k_mat[1][1] = mag * normal_scale
		k_mat[2][2] = mag * normal_scale

		k_mat = rotation @ k_mat @ rotation.T
		return k_mat
		
a = adjuster()

while(1):
	i = input()
