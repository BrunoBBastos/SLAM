import numpy as np
from numpy import random
from numpy.linalg import inv
import numpy.matlib
import math
import matplotlib.pyplot as plt
import scipy.linalg as LA


##################################### UTILIDADES
def angleWrap(ang):
	if ang > np.pi:
		return ang - 2 * np.pi
	elif ang < -np.pi:
		return ang + 2 * np.pi
	else:
		return ang

def tcomp(tab, tbc):
	# print("oi", tab, tbc)
	result = tab[2] + tbc[2]
	if result > np.pi or result <= -np.pi:
		result = angleWrap(result)
	s = np.sin(tab[2])
	c = np.cos(tab[2])
	matriz = np.array([c, -s, s, c]).reshape(2, 2)
	tac = tab[0:2] + matriz.dot(tbc[0:2])
	tac = np.append(tac, [result], axis = 0)
	tac = tac.astype('float')
	# print(tac)
	return tac

def tinv(tab):
	tba = np.zeros(tab.size)
	for t in range(0, tab[0].size, 3):
		tba = tinv1(tab[t:t+3])
	return tba

def tinv1(tab):
	s = np.sin(tab[2])
	c = np.cos(tab[2])
	tba = np.array([-tab[0] * c - tab[1] * s, tab[0] * s - tab[1] * c, -tab[2]]).reshape(3, 1)
	return tba

##################################### FUNÇÕES
def GetRobotControl(k):
	return np.array([0, 0.025, 0.1 * np.pi / 180 * np.sin(3 * np.pi * k / nSteps)]).reshape(3, 1)
	# return np.array([0, 0.025, 0]).reshape(3, 1)

def GetOdometry(k):
	# Aparentemente python não tem uma forma persistente como 'static' do C++ ou 'persistent' do MATLAB, mas isso deve funcionar:
	# GetOdometry.LastOdom # internal to Robot low-level controller
	global UTrue
	if k == 0:
		GetOdometry.LastOdom = np.copy(XTrue)
	u = GetRobotControl(k)
	xnow = tcomp(GetOdometry.LastOdom, u)
	uNoise = np.sqrt(UTrue).dot(random.normal(0, 1, (3, 1)))
	xnow = tcomp(xnow, uNoise)
	GetOdometry.LastOdom = np.copy(xnow)
	return xnow

def GetObsJac(xPred, iFeature, Map):
	jH = np.zeros((2, 3))
	Delta = Map[:, iFeature].reshape(2, 1) - xPred[0:2]
	r = LA.norm(Delta)
	jH[0, 0] = -Delta[0] / r
	jH[0, 1] = -Delta[1] / r
	jH[1, 0] = Delta[1] / r**2
	jH[1, 1] = -Delta[0] / r**2
	jH[1, 2] = -1
	return jH

def simulateWorld(k):
	global XTrue
	u = GetRobotControl(k)
	XTrue = tcomp(XTrue, u)
	XTrue[2] = angleWrap(XTrue[2])

def doObservationModel(xVeh, iFeature, Map):
	Delta = Map[:, iFeature].reshape(2, 1) - xVeh[0:2]
	at = np.arctan2(Delta[1], Delta[0]) - xVeh[2] # Evitar incluir array dentro de array
	z = np.array([LA.norm(Delta), at[0]]).reshape(2, 1)
	z[1] = angleWrap(z[1])
	return z

def J1(x1, x2):
	s1 = np.sin(x1[2])
	c1 = np.cos(x1[2])

	Jac = np.array([[1, 0, -x2[0] * s1 - x2[1] * c1],
			   [0, 1, x2[0] * c1 - x2[1] * s1],
			   [0, 0, 1]], dtype = object)
	return Jac

def J2(x1, x2):
	s1 = np.sin(x1[2])
	c1 = np.cos(x1[2])

	Jac = np.array([[c1, -s1, 0],
			   [s1, c1, 0],
			   [0, 0, 1]], dtype = object)
	return Jac

def GetObservation(k):
	global Map, XTrue, RTrue, nSteps
	# Simular falha nos sensores
	if abs(k - nSteps/2) < 0.1 * nSteps:
		z = np.empty(0)
		iFeature = -1
	else:
		iFeature = int(np.floor(Map[0].size * np.random.uniform()))
		z = doObservationModel(XTrue, iFeature, Map) + np.sqrt(np.diag(RTrue)).reshape(2, 1) * random.normal(0, 1, size = (2, 1))
		z[1] = angleWrap(z[1])
	return z, iFeature

def DoGraphs():
	global XTrue, XEst, PEst
	plt.clf()
	plt.axis([-80, 80, -80, 80])
	plt.title('Localisation')
	plt.scatter(Map[0, :], Map[1, :], color = 'green')
	PlotEllipse(XEst, PEst, 3)
	DrawRobot(XEst, 'red')
	DrawRobot(XTrue, 'blue')
	DrawRobot(XPredOnly, 'green')
	plt.pause(0.001)

def DrawRobot(Xr, col):
	p = 0.02
	xmin, xmax, ymin, ymax = plt.axis()
	l1 = (xmax - xmin) * p
	l2 = (ymax - ymin) * p
	P = np.array([[-1, 1, 0, -1], [-1, -1, 3, -1]])
	theta = Xr[2]
	c = math.cos(theta)
	s = math.sin(theta)
	rotation = np.array([[c, -s], [s, c]], dtype = float)
	P = rotation.dot(P)
	P[0, :] = P[0, :] * l1 + Xr[0]
	P[1, :] = P[1, :] * l2 + Xr[1]
	plt.plot(P[0, :], P[1, :], color = col)
	plt.plot(Xr[0], Xr[1], color = col)

def PlotEllipse(x, Po, nSigma):
	x = np.copy(x[0:2])
	P = np.empty((2, 2)) # Copiando dessa forma por causa dos problemas de formatação da np
	for i in range(2):
		for j in range(2):
			P[i, j] = Po[i, j][0]
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

###################################################################### EXECUÇÃO
nSteps = 6000

Map = 140 * np.random.uniform(size = (2, 10)) - 70

UTrue = np.diag([0.01, 0.01, 1 * np.pi / 180])**2
RTrue = np.diag([2, 3 * np.pi / 180])**2

UEst = 1 * UTrue
REst = 1 * RTrue

XTrue = np.array([1, -40, -np.pi / 2]).reshape(3, 1)

xOdomLast = GetOdometry(0)

XEst = np.copy(XTrue)
XPredOnly = np.copy(XTrue)
PEst = np.diag([1, 1, (1 * np.pi / 180)**2])

gif = 0
for k in range(1, nSteps+1):

	simulateWorld(k)
	xOdomNow = GetOdometry(k)
	u = tcomp(tinv(xOdomLast), xOdomNow)
	xOdomLast = np.copy(xOdomNow)

	XPred = tcomp(XEst, u) 
	XPred[2] = angleWrap(XPred[2])
	# XPredOnly = np.copy(XPred)
	XPredOnly = tcomp(XPredOnly, u)
	PPred = J1(XEst, u).dot(PEst).dot(J1(XEst, u).T) + J2(XEst, u).dot(UEst).dot(J2(XEst, u).T)
	z, iFeature = GetObservation(k)

	if z.size != 0:
		zPred = doObservationModel(XPred, iFeature, Map)
		jH = GetObsJac(XPred, iFeature, Map)

		innov = z - zPred
		innov[1] = angleWrap(innov[1])

		S = jH.dot(PPred).dot(jH.T) + REst
		S = S.astype('float')
		W = PPred.dot(jH.T).dot(inv(S))

		XEst = XPred + W.dot(innov)
		XEst[2] = angleWrap(XEst[2])
		XEst = XEst.astype('float')

		# Forma de 'Joseph', dita ser numericamente estável
		I = np.eye(3)
		PEst = (I - W.dot(jH)).dot(PPred).dot((I - W.dot(jH)).T) + W.dot(REst).dot(W.T)
		PEst = 0.5 * (PEst + PEst.T) # WTF?

	else:
		XEst = np.copy(XPred)
		PEst = np.copy(PPred)
		innov = np.empty((2, 1))
		# S = np.eye(2) * None

	if k%15 == 1:
		DoGraphs()
		# plt.savefig('gif' + str(gif) + '.jpg')
		# gif+=1

plt.show()