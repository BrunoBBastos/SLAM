import numpy as np
from numpy import random
import numpy.matlib
import math
import matplotlib.pyplot as plt
import scipy.linalg as LA


#−−−−−−−−−−−− MODEL −−−−−−−−−−−−−−#
def AckermannModel(x, u, dT, L):
	y = np.ndarray((3,1))
	y[0][0] = x[0] + dT * u[0] * np.cos(x[2])
	y[1][0] = x[1] + dT * u[0] * np.sin(x[2])
	y[2][0] = x[2] + dT * u[0] / L * np.tan(u[1])
	return y

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

#−−−−−−−− Drawing Covariance −−−−−#
def PlotEllipse(x, P, nSigma):
	P = P[0:2, 0:2]
	x = x[0:2]
	if not 0 in np.diag(P):
		D, V = np.linalg.eig(P)
		D = np.diag(D)
		s = np.linspace(0, 2 * np.pi, 50)
		pX = []
		pY = []
		for i in s:
			y = nSigma * np.array([[np.cos(i)], [np.sin(i)]])
			el = V.dot(LA.sqrtm(D)).dot(y)
			el =  np.matlib.repmat(x, 1, len(el[0]) + 1) + np.concatenate((el, el), axis = 1)
			pX += [el[0,0]]
			pY += [el[1,0]]
		plt.plot(pX, pY, color = 'red')


dT = 0.1 # time steps size
nSteps = 600 # length of run
L = 2 # length of vehicle
SigmaV = 0.1# 3cm/s std on speed
SigmaPhi = 4 * np.pi / 180 # steer inaccuracy

# initial knowledge pdf (prior @ k = 0)
P = np.diag([0.2, 0.2, 0])
x = np.array([0, 0, 0]).reshape((3, 1))
xtrue = np.copy(x)
Q = np.diag([SigmaV**2, SigmaPhi**2])

#−−−−−−−− Set up graphics −−−−−−−−−−−#
plt.grid(b = True)
plt.axis([-20, 20, -5, 30])
plt.title('Uncertainty bounds for Ackermann model')
plt.xlabel('x')
plt.ylabel('y')

#−−−−−−−− Main loop −−−−−−−−−−−#
for k in range(nSteps):
	# plt.clf() # Descomentar para animar
# control is a wiggle at constant velocity
	u = np.array([1, np.pi/5 * np.sin(4 * np.pi * k/nSteps)]).reshape(2, 1)
# calculate jacobians
	JacFx = np.array([[1, 0, -dT * u[0] * np.sin(x[2])],
					  [0, 1, dT * u[0] * np.cos(x[2])],
					  [0, 0, 1]], dtype = float)
	JacFu = np.array([[dT * np.cos(x[2]), 0],
					  [dT * np.sin(x[2]), 0],
					  [dT * np.tan(u[1]) / L, dT * u[0] * (1 / np.cos(u[1])**2)]], dtype = float)
# prediction steps
	P = JacFx.dot(P).dot(JacFx.T) + JacFu.dot(Q).dot(JacFu.T)
	xtrue = AckermannModel(xtrue, u + np.array([[SigmaV], [SigmaPhi]]) * random.normal(0, 1, size = (2,1)), dT, L)
	x = AckermannModel(x, u , dT, L)
# draw occasionally
	if (k % 30) == 0: # Comentar para animar
		PlotEllipse (x, P, 0.5)
		plt.plot(xtrue[0], xtrue[1], 'dk')
		DrawRobot(x)
		plt.pause(0.01)

plt.show()