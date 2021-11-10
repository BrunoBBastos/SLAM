import numpy as np
import numpy.matlib
import scipy as scp
import scipy.linalg
import matplotlib.pyplot as plt



############################################################################# FUNÇÕES

def AngleWrap(ang):
	if ang > np.pi:
		return ang - 2 * np.pi
	elif ang < -np.pi:
		return ang + 2 * np.pi
	else:
		return ang

def tcomp(tab, tbc):
	result = tab[2] + tbc[2]
	if result > np.pi or result <= -np.pi:
		result = angleWrap(result)
	s = np.sin(tab[2])
	c = np.cos(tab[2])
	matriz = np.array([c, -s, s, c]).reshape(2, 2)
	tac = tab[0:2] + matriz.dot(tbc[0:2])
	tac = np.append(tac, [result], axis = 0)
	tac = tac.astype('float')
	return tac

def GetObservation(k):
	# Escolher feature aleatória do mapa verdadeiro
	iFeature = int(np.floor(Map[0].size * np.random.uniform()))
	z = DoObservationModel(xVehicleTrue, Map[:, iFeature:iFeature+1]) + np.sqrt(RTrue).dot(np.random.normal(0, 1, size = (2, 1)))
	z[1][0] = AngleWrap(z[1][0])
	return z, iFeature

def DoObservationModel(xVeh, xFeature):
	Delta = xFeature - xVeh[0:2]
	z = ([[scp.linalg.norm(Delta)], [np.arctan2(Delta[1], Delta[0])[0] - xVeh[2][0]]])
	z[1][0] = AngleWrap(z[1][0])
	return z

def SimulateWorld(k):
	global xVehicleTrue
	u = GetRobotControl(k)
	xVehicleTrue = tcomp(xVehicleTrue, u)

def GetObsJacs(xPred, xFeature):
	jHxv = np.zeros((2, 3))
	jHxf = np.zeros((2, 2))
	Delta = xFeature - xPred[0:2]
	r = scp.linalg.norm(Delta)
	jHxv[0, 0] = -Delta[0] / r
	jHxv[0, 1] = -Delta[1] / r
	jHxv[1, 0] = Delta[1] / r**2
	jHxv[1, 1] = -Delta[0] / r**2
	jHxv[1, 2] = -1
	jHxf[0:2, 0:2] = -jHxv[0:2, 0:2]
	return jHxv, jHxf

def GetNewFeatureJacs(Xv, z):
	x = Xv[0, 0]
	y = Xv[1, 0]
	theta = Xv[2, 0]
	r = z[0][0]
	bearing = z[1][0]
	jGx = np.array([[1, 0, -r*np.sin(theta + bearing)], [0, 1, r*np.cos(theta + bearing)]])
	jGz = np.array([[np.cos(theta + bearing), -r*np.sin(theta + bearing)], [np.sin(theta + bearing), r*np.cos(theta+bearing)]])
	return jGx, jGz

def GetRobotControl(k):
	# return np.array([[0], [0.25], [0.3 * np.pi / 180 * np.sin(3*np.pi*k/nSteps)]])
	# Alternativa sugerida
	return np.array([[0], [0.25], [0.3 * np.pi/180]])
	
def DoMapGraphics(xMap, PMap, nSigma):
	global Map
	plt.clf()
	plt.axis([-150, 150, -150, 150])
	plt.title('Mapping')
	plt.scatter(Map[0], Map[1])
	for i in range(int(len(xMap)/2)):
		iL = 2*i 
		iH = 2*i+1
		x = xMap[iL:iH+1]
		P = PMap[iL:iH+1, iL:iH+1]
		PlotEllipse(x, P, nSigma)
		if x.size > 0:
			plt.plot(x[0], x[1], 'r+')
	plt.plot(xVehicleTrue[0], xVehicleTrue[1], 'b+')
	plt.pause(0.01)

def PlotEllipse(x, Po, nSigma):
	x = np.copy(x[0:2])
	P = np.empty((2, 2)) # Copiando dessa forma por causa dos problemas de formatação da np
	for i in range(2):
		for j in range(2):
			P[i, j] = Po[i, j]
	if not 0 in np.diag(P):
		D, V = np.linalg.eig(P)
		D = np.diag(D)
		s = np.linspace(0, 2 * np.pi, 50)
		pX = []
		pY = []
		for i in s:
			y = nSigma * np.array([[np.cos(i)], [np.sin(i)]])
			el = V.dot(scp.linalg.sqrtm(D)).dot(y)
			el =  np.matlib.repmat(x, 1, len(el[0]) + 1) + np.concatenate((el, el), axis = 1)
			pX += [el[0,0]]
			pY += [el[1,0]]
		plt.plot(pX, pY, color = 'red')

############################################################################# SETUP

nSteps = 600
nFeatures = 6
MapSize = 200
Map = MapSize * np.random.uniform(size = (2, nFeatures)) - MapSize/2
UTrue = np.diag([0.01, 0.01, 1 * np.pi/180])**2
RTrue = np.diag([8, 7 * np.pi/180])**2
UEst = 1 * UTrue
REst = 1 * RTrue
xVehicleTrue = np.array([[1],[-40], [-np.pi / 2]])

# Condições iniciais - sem mapa
xEst = np.empty(shape = (0))
# PEst = np.array([])
PEst = np.empty(shape = (0, 0)) # na blockdiag, arrays de tamanho zero serão ignorados, mas [] = (1, 0)
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.block_diag.html
MappedFeatures = np.ones([nFeatures, 2]) * -1

############################################################################# LOOP

for k in range(nSteps):
	
	SimulateWorld(k)
	# Modelo de predição simples
	xPred = xEst
	PPred = PEst

	# Observar feature aleatória
	z, iFeature = GetObservation(k)

	if z.size != 0:
		# Testar se a feature já foi vista antes # NaN None
		if MappedFeatures[iFeature, 0] != -1:

		# Predição da observação: descobrir onde a feature está no vetor de estados
			FeatureIndex = int(MappedFeatures[iFeature, 0])

			xFeature = xPred[FeatureIndex: FeatureIndex+2] 
			zPred = DoObservationModel(xVehicleTrue, xFeature)
			# Jacobiano da observação
			jHxv, jHxf = GetObsJacs(xVehicleTrue, xFeature)
			# Preencher Jacobiano de estados
			jH = np.zeros([2, len(xEst)])
			jH[:, FeatureIndex: FeatureIndex+2] = jHxf

			# Update de Kalman
			Innov = z - zPred
			Innov[1] = AngleWrap(Innov[1])
			S = jH.dot(PPred).dot(jH.T) + REst
			W = PPred.dot(jH.T).dot(np.linalg.inv(S))

			# Na referência da função 'dot' diz ser preferível usar * qd for array * escalar
			if Innov.size == 1:
				xEst = xPred + W*(Innov)
			else: 
				xEst = xPred + W.dot(Innov)

			PEst = PPred - W.dot(S).dot(W.T)
			I = np.eye(len(PEst))
			PEst = (I - W.dot(jH)).dot(PPred).dot((I - W.dot(jH)).T) + W.dot(REst).dot(W.T)
			# Garantir que P seja simétrica
			PEst = 0.5 * (PEst + PEst.T)

		else:
			# Esta é uma nova feature: adicione ao mapa
			nStates = len(xEst)
			xFeature = xVehicleTrue[0:2] + np.array([[z[0][0] * np.cos(z[1][0] + xVehicleTrue[2][0])], [z[0][0] * np.sin(z[1][0] + xVehicleTrue[2][0])]]) 
			
			if not nStates:
				xEst = np.copy(xFeature)
			else:
				xEst = np.append(xEst, xFeature, axis = 0)

			jGxv, jGz = GetNewFeatureJacs(xVehicleTrue, z)

			# Obra de arte:

			M = np.append(np.eye(nStates), np.zeros((nStates, 2)), axis = 1)
			M = np.append(M, np.append(np.zeros((2, nStates)), jGz, axis = 1), axis = 0)

			PEst = M.dot(scp.linalg.block_diag(PEst, REst)).dot(M.T)
			# lembrar desta feature como já mapeada, armazenaremos sua ID e posição no vetor de estados
			MappedFeatures[iFeature, :] = np.array([len(xEst) - 2, len(xEst) -1])

	# Desenhar

	if k % 3 == 0:
		DoMapGraphics(xEst, PEst, 5)

print()
print(xEst)
print()
print(Map)

plt.show()