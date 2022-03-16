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
		result = AngleWrap(result)
	s = np.sin(tab[2][0])
	c = np.cos(tab[2][0])
	matriz = np.array([c, -s, s, c]).reshape(2, 2)
	tac = tab[0:2] + matriz.dot(tbc[0:2])
	tac = np.append(tac, [result], axis = 0)
	tac = tac.astype('float')
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

def GetObservation(k):
	done = False
	Trys = 0
	z = np.array([])
	iFeature = -1
	while not done and Trys < 0.5 * Map[0].size:
		# Escolher feature aleatória do mapa verdadeiro
		iFeature = int(np.floor(nFeatures * np.random.uniform()))
		# print(iFeature, '\n', Map[:, iFeature])
		z = DoObservationModel(xVehicleTrue, Map[:, iFeature].reshape(2,1)) + np.sqrt(RTrue).dot(np.random.normal(0, 1, size = (2, 1)))
		z[1][0] = AngleWrap(z[1][0])
		# Olhar pra frente e checar se a feature está à vista
		if abs(np.pi/2 - z[1][0]) < sensorFieldOfView * np.pi/180 and z[0] < sensorRange:
			done = True
		else:
			Trys += 1
			z = np.array([])
			iFeature = -1
	return z, iFeature

def DoObservationModel(xVeh, xFeature):
	Delta = xFeature - xVeh[0:2]
	z = np.array([[np.linalg.norm(Delta)], [np.arctan2(Delta[1][0], Delta[0][0]) - xVeh[2][0]]])
	z[1][0] = AngleWrap(z[1][0])
	return z

def SimulateWorld(k):
	global xVehicleTrue
	u = GetRobotControl(k)
	xVehicleTrue = tcomp(xVehicleTrue, u)

def J1(x1, x2):
	s1 = np.sin(x1[2, 0])
	c1 = np.cos(x1[2, 0])
	# Apesar de ser uma linha com uma única coluna, endereçar os elementos de x2 assim evitam criar array dentro de array
	Jac = np.array([[1, 0, -x2[0, 0] * s1 - x2[1, 0] * c1],
			   		[0, 1, x2[0, 0] * c1 - x2[1, 0] * s1],
			   		[0, 0, 1]], dtype = object)
	return Jac

def J2(x1, x2):
	s1 = np.sin(x1[2, 0])
	c1 = np.cos(x1[2, 0])

	Jac = np.array([[c1, -s1, 0],
			   		[s1, c1, 0],
			   		[0, 0, 1]], dtype = object)
	return Jac

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
	jGx = np.array([[1, 0, -r*np.sin(theta + bearing)],
					[0, 1, r*np.cos(theta + bearing)]])
	jGz = np.array([[np.cos(theta + bearing), -r*np.sin(theta + bearing)],
					[np.sin(theta + bearing), r*np.cos(theta+bearing)]])
	return jGx, jGz

def GetOdometry(k):
	# Aparentemente python não tem uma forma persistente como 'static' do C++ ou 'persistent' do MATLAB, mas isso deve funcionar:
	# GetOdometry.LastOdom # internal to Robot low-level controller
	global UTrue
	if k == 0:
		GetOdometry.LastOdom = np.copy(xVehicleTrue)
	u = GetRobotControl(k)
	xnow = tcomp(GetOdometry.LastOdom, u)
	uNoise = np.sqrt(UTrue).dot(np.random.normal(0, 1, (3, 1)))
	xnow = tcomp(xnow, uNoise)
	GetOdometry.LastOdom = np.copy(xnow)
	return xnow

def GetRobotControl(k):
	global nSteps
	u = np.array([[0], [0.15], [0.2*np.pi / 180]])
	# u = np.array([[0], [0.25], [0.3*(np.pi/180)*np.sin(3*np.pi*k/nSteps)]])
	return u

def DoGraphics():
	global xVehicleTrue, xEst, PEst, Map
	plt.clf()
	plt.axis([-MapSize/2, MapSize/2, -MapSize/2, MapSize/2])
	plt.title('SLAM')
	plt.plot(Map[0, :], Map[1, :], 'g+')
	DoVehicleGraphics(xVehicleTrue, 'black')
	DoVehicleGraphics(xEst[0:3, 0:3], 'blue')
	PlotEllipse(xEst[0:3], PEst[0:3][0:3], 5)
	if len(xEst) > 3:
		DoMapGraphics(xEst[3:], PEst[3:,3:], 5)

	plt.pause(0.0000002)


def DoVehicleGraphics(Xr, col):
	p = 0.02
	xmin, xmax, ymin, ymax = plt.axis()
	l1 = (xmax - xmin) * p
	l2 = (ymax - ymin) * p
	P = np.array([[-1, 1, 0, -1], [-1, -1, 3, -1]])
	theta = Xr[2][0]
	c = np.cos(theta)
	s = np.sin(theta)
	rotation = np.array([[c, -s], [s, c]], dtype = float)
	P = rotation.dot(P)
	P[0, :] = P[0, :] * l1 + Xr[0]
	P[1, :] = P[1, :] * l2 + Xr[1]
	plt.plot(P[0, :], P[1, :], color = col)
	plt.scatter(Xr[0], Xr[1], color = col, s = 2)

def DoMapGraphics(xMap, PMap, nSigma):
	for i in range(int(len(xMap)/2)):
		iL = 2*i 
		iH = 2*i+1
		x = xMap[iL:iH+1]
		P = PMap[iL:iH+1, iL:iH+1]
		PlotEllipse(x, P, nSigma)
		if x.size > 0:
			plt.scatter(x[0], x[1], s = 4, color = 'red')

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

debug = False

sensorFieldOfView = 45
sensorRange = 100

drawEveryNFrames = 50
nSteps = 8000

nFeatures = 40
MapSize = 200
Map = MapSize * np.random.uniform(size = (2, nFeatures)) - MapSize/2
# Map = np.array([[15, 35], [0, 50]])

UTrue = np.diag([0.01, 0.01, 1.5 * np.pi/180])**2
RTrue = np.diag([1.1, 5 * np.pi/180])**2

UEst = 2 * UTrue
REst = 2 * RTrue
xVehicleTrue = np.array([[0],[0], [-np.pi / 2]])

xEst = np.copy(xVehicleTrue)
PEst = np.diag([1, 1, 0.01])
# PEst = np.diag([10, 10, 0.1])

MappedFeatures = np.ones([nFeatures, 2]) * -1
xOdomLast = GetOdometry(0)


############################################################################# TESTES
# print('testes')
# print(DoObservationModel(xVehicleTrue, Map[:, 0]))
# print(DoObservationModel(xVehicleTrue, Map[:, 1]))
# print('fim dos testes')

############################################################################# EXECUÇÃO

for k in range(1, nSteps+1):

	# Simulação do cenário
	SimulateWorld(k)

	# Encontrar o sinal de controle
	xOdomNow = GetOdometry(k)
	u = tcomp(tinv(xOdomLast), xOdomNow)
	xOdomLast = xOdomNow

	if debug:
		print(k)
		print("xEst")
		print(xEst)
		print()

	xVehicle = xEst[0:3]
	xMap = xEst[3:]

	# Predição
	xVehiclePred = tcomp(xVehicle, u)
	PPredVV = J1(xVehicle, u).dot(PEst[0:3, 0:3]).dot(J1(xVehicle, u).T) + J2(xVehicle, u).dot(UEst).dot(J2(xVehicle, u).T)
	PPredVM = J1(xVehicle, u).dot(PEst[0:3, 3:])
	PPredMM = np.copy(PEst[3:, 3:])

	xPred = np.append(xVehiclePred, xMap, axis = 0)
	PPred = np.append(PPredVV, PPredVM, axis = 1)
	PPred = np.append(PPred, np.append(PPredVM.T, PPredMM, axis = 1), axis = 0)

	if debug:
		print("PPred")
		print(PPred)
		print()

	# Observar uma feature aleatória
	z, iFeature = GetObservation(k)

	if debug:
		print("z")
		print(z)
		print()

	if z.size != 0:
		# Testar se a feature já foi vista antes
		if MappedFeatures[iFeature, 0] != -1:
			# Predição da observação: descobrir onde a feature está no vetor de estados
			FeatureIndex = int(MappedFeatures[iFeature, 0])
			xFeature = xPred[FeatureIndex: FeatureIndex+2] 
			zPred = DoObservationModel(xVehicle, xFeature)

			if debug:
				print("zpred")
				print(zPred)
				print()

			# Jacobiano da observação
			jHxv, jHxf = GetObsJacs(xVehicle, xFeature)
			# Preencher Jacobiano de estados
			jH = np.zeros([2, len(xEst)])
			jH[:, FeatureIndex: FeatureIndex+2] = jHxf
			jH[:, 0:3] = jHxv

			if debug:
				print('jH')
				print(jH)
				print()

			# Update de Kalman
			Innov = z - zPred
			Innov[1] = AngleWrap(Innov[1])
			S = jH.dot(PPred).dot(jH.T) + REst
			S = S.astype('float')

			W = PPred.dot(jH.T).dot(np.linalg.inv(S))

			if debug:
				print('Innov')
				print(Innov)
				print()
				print('S')
				print(S)
				print()
				print('W')
				print(W)
				print()


			# Na referência da função 'dot' diz ser preferível usar * qd for array * escalar
			if Innov.size == 1:
				xEst = xPred + W*(Innov)
			else: 
				xEst = xPred + W.dot(Innov)

			PEst = PPred - W.dot(S).dot(W.T)
			# I = np.eye(len(PEst))
			# PEst = (I - W.dot(jH)).dot(PPred).dot((I - W.dot(jH)).T) + W.dot(REst).dot(W.T)
			# Garantir que P seja simétrica
			PEst = 0.5 * (PEst + PEst.T)

			if debug:
				print('xEst')
				print(xEst)
				print()
				print('PEst')
				print(PEst)
				print()

		else:
			# Esta é uma nova feature: adicione ao mapa
			nStates = len(xEst)
			
			xFeature = xVehicle[0:2] + np.array([[z[0][0] * np.cos(z[1][0] + xVehicle[2][0])],
												 [z[0][0] * np.sin(z[1][0] + xVehicle[2][0])]]) 

			xEst = np.append(xEst, xFeature, axis = 0) # Expansão do vetor de estados

			jGxv, jGz = GetNewFeatureJacs(xVehicle, z)

			if debug:
				print('jGxv')
				print(jGxv)
				print()
				print('jGz')
				print(jGz)
				print()

			M = np.append(np.eye(nStates), np.zeros((nStates, 2)), axis = 1)
			M = np.append(M, np.append(np.append(jGxv, np.zeros((2, nStates - 3)), axis = 1), jGz, axis = 1), axis = 0)
			PEst = M.dot(scp.linalg.block_diag(PEst, REst)).dot(M.T)
			# lembrar desta feature como já mapeada, armazenaremos sua ID e posição no vetor de estados
			MappedFeatures[iFeature, :] = np.array([len(xEst) - 2, len(xEst) -1])

			if debug:
				print('PEst')
				print(PEst)
				print()

	else:
		xEst = np.copy(xPred)
		PEst = np.copy(PPred)

	# Desenhar
	if k % drawEveryNFrames == 0:
		DoGraphics()

print(xEst)
print()
print(Map)
print()
print(PEst)
plt.show()