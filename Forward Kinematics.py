import numpy as np


def dh2a(dh):
    a, d, theta, alpha = dh
    A = ([np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
         [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
         [0, np.sin(alpha), np.cos(alpha), d],
         [0, 0, 0, 1])
    return A


# input
q = []
for i in range(0, 3):
    ele = float(input())
    q.append(ele)

DH = ([1.0, 1.0, q[0], 0.0],
      [1.0, 0.0, q[1], 0.0],
      [0.0, q[2], 0.0, 0.0])

A = []
for x in DH:
    a = dh2a(x)
    A.append(a)

T = np.matmul(np.matmul(A[0], A[1]), A[2])
Pose = [T[0][3], T[1][3], T[2][3]]
print(Pose)
