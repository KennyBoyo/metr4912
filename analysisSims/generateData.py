import numpy as np

np.random.seed(0)

def gen_uniform_angles(n_samples=500):
    return np.random.uniform(-np.pi, np.pi, n_samples)


gen_uniform_angles()