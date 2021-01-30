import numpy as np

a = np.array([1,2])
b = np.array([1,2])
# c = np.dot(a, b)
c = np.arccos(np.clip(0.5, -1, 1))
d = np.arccos(np.clip(-0.5, -1, 1))
print(c,d)
print(np.arccos(0.9))