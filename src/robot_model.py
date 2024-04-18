import numpy  as np
import modern_robotics as mr



w0a = np.array([-1, 0, 0])
w1a = np.array([0, -1, 0])
w2a = np.array([0, 0, -1])
w3a = np.array([1, 0, 0])
w3b = np.array([-1, 0, 0])
w2b = np.array([0, 0, -1])
w1b = np.array([0, 1, 0])
w0b = np.array([-1, 0, 0])

q0a = np.array([0.1495, 0, -1.400])
q1a = np.array([0, 0, -1.200])
q2a = np.array([0, 0, -0.9365])
q3a = np.array([0, 0, -0.8665])
q3b = np.array([0, 0, -0.5335])
q2b = np.array([0, 0, -0.4635])
q1b = np.array([0, 0, -0.200])
q0b = np.array([-0.1495, 0, 0])


w = np.array([w0a, w1a, w2a, w3a, w3b, w2b, w1b, w0b])
q = np.array([q0a, q1a, q2a, q3a, q3b, q2b, q1b, q0b])

v = np.zeros((8, 3))

for i in range(0, len(w)):
    v[i] = np.cross(-w[i], q[i])

Blist = np.hstack((w, v)).T

print(Blist)
