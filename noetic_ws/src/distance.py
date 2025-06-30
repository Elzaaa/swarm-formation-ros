import numpy as np
from scipy.spatial.distance import cdist

# Example data
A = np.array([[0, 0], [1, 1], [2,3], [3,4]])   # Shape (2, 2)
B = np.array([[1, 0], [2, 2]])   # Shape (2, 2)

# Define a distance function (Euclidean)
def dist(a, b):
    return np.linalg.norm(a - b)

# Prepare a grid of indices
A_idx, B_idx = np.meshgrid(np.arange(len(A)), np.arange(len(B)), indexing='ij')

# Vectorize the distance function
vec_dist = np.vectorize(lambda i, j: dist(A[i], B[j]))

# Compute the distance matrix
# distance_matrix = vec_dist(A_idx, B_idx)
distance_matrix = cdist(A, B)
print(distance_matrix)


