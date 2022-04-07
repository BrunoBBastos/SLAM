import numpy as np
from numpy import random
import numpy.matlib
import math
import matplotlib.pyplot as plt
import scipy.linalg as LA


#−−−−−−−−−−−− MODEL −−−−−−−−−−−−−−#

def ModeloDiferencial(X, u, dT):
    y = np.ndarray((3, 1))
    y[0][0] = X[0] + dT * u[0] * np.cos(X[2])
    y[1][0] = X[1] + dT * u[0] * np.sin(X[2])
    y[2][0] = X[2] + dT * u[1]
    return y

def GetRobotControl(X, Ref):
    thetaRef = np.arctan2(Ref[1] - X[1], Ref[0] - X[0])
    erroAngular = thetaRef - X[2]

    erroLinear = LA.norm(Ref[0:2] - X[0:2]) * np.cos(erroAngular)
    v = k1 * erroLinear
    w = k2 * erroAngular
    u = np.array([v, w]).reshape(2, 1)
    return u

#−−−−−−−− Drawing Vehicle −−−−−#
def DrawRobot(Xr):
	p = 0.02
	xmin, xmax, ymin, ymax = plt.axis()
	l1 = (xmax - xmin) * p
	l2 = (ymax - ymin) * p
	P = np.array([[-1, 1, 0, -1], [-1, -1, 3, -1]])
	theta = Xr[2] - np.pi/2
	c = math.cos(theta)
	s = math.sin(theta)
	rotation = np.array([[c, -s], [s, c]], dtype = float)
	P = rotation.dot(P)
	P[0, :] = P[0, :] * l1 + Xr[0]
	P[1, :] = P[1, :] * l2 + Xr[1]
	plt.plot(P[0, :], P[1, :], color = 'blue')
	plt.plot(Xr[0], Xr[1], 'b+')


dT = 0.1 # time steps size
nSteps = 2000 # length of run
l = 0.120 # length of vehicle
k1 = 0.05
k2 = 0.2
X = np.array([0, 0, 0]).reshape((3, 1))

#−−−−−−−− Set up graphics −−−−−−−−−−−#
plt.grid(b = True)
plt.axis([-2, 2, -2, 2])
plt.title('Teste co controle por referência')
plt.xlabel('x')
plt.ylabel('y')

#−−−−−−−− Main loop −−−−−−−−−−−#
for k in range(nSteps):
	# plt.clf() # Descomentar para animar

	# u = np.array([0.3, 0.05]).reshape(2, 1)
    u = GetRobotControl(X, [-1, 1])
    X = ModeloDiferencial(X, u, dT)

# draw occasionally

    if (k % 30) == 0: # Comentar para animar
        DrawRobot(X)
        plt.pause(0.1)

plt.show()