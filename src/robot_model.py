import numpy  as np
import modern_robotics as mr

R = np.array([[1, 0, 0],
              [0, -1, 0],
              [0, 0, -1]])

p = np.array([[0, 0, -1.4]])

M = np.vstack((np.hstack((R,p.T)), [0, 0, 0, 1]))

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

wab = np.array([w0a, w1a, w2a, w3a, w3b, w2b, w1b, w0b])
qab = np.array([q0a, q1a, q2a, q3a, q3b, q2b, q1b, q0b])

vab = np.zeros((8, 3))
vba = np.zeros((8, 3))

wba = R @ wab.T
wba = wba.T

one_collum = np.ones((8, 1))
qab_tilda = np.hstack([qab, one_collum])
qba = M @ qab_tilda.T
qba = qba[0:3].T

for i in range(0, len(wab)):
    vab[i] = np.cross(-wab[i], qab[i])

for i in range(0, len(wba)):
    vba[i] = np.cross(-wba[i], qba[i])


Blist_ab = np.hstack((wab, vab)).T

Blist_ba = np.hstack((wba, vba)).T

print(Blist_ba)
