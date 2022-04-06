import numpy as np
from numpy import random
import matplotlib.pyplot as plt
np.set_printoptions(precision = 8, suppress = True)

################## Parâmetros

StopTime = 600 # duração em segundos
dT = 0.1 # período do loop
clight = 2.998e8
EntryDrag = 5 # constante arrasto
ChuteDrag = 2.5 * EntryDrag # arrasto com o paraquedas aberto
g = 9.8 / 3 # gravidade em Marte
m = 50 # massa do veículo
RocketBurnHeight = 1000 # altura de acionamento do freio
OpenChuteHeight = 4000 # altura de abertura do paraquedas
X0 = 10000 # altura de entrada
V0 = 0 # velocidade de entrada
InitialHeightError = 0 # erro da altura na entrada
InitialVelocityError = 0 # erro da velocidade na entrada
InitialHeightStd = 100 # incerteza nas condições iniciais
InitialVelocityStd = 20
BurnTime = None
ChuteTime = None
LandingTime = None

# Condições iniciais do veículo ao entrar na atmosfera:
ChuteOpen = False
RocketsOn = False
Drag = EntryDrag
Thrust = 0
Landed = False

# Modelo da planta do processo
F = np.array([[1, dT], [0, 1]])
G = np.array([dT**2 / 2, dT]).reshape(2, 1)

SigmaQ = 0.2 # ms**-2

Q = (1.1 * SigmaQ)**2 # ms**-2 std
H = np.array([2/clight, 0])
SigmaR = 1.3e-7
R = (1.1 * SigmaR)**2
XTrue = np.array([X0, V0]).reshape(2, 1)

XEst = np.array([X0 + InitialHeightError, V0 + InitialVelocityError]).reshape(2, 1)
PEst = np.diag([InitialHeightStd**2, InitialVelocityStd**2])
# DoStore(0, XEst, PEst, [0], [0], None)

k = 0

################## Funções
# Sistema de medição
def GetSensorData(k):
	return H.dot(XTrue) + SigmaR * random.normal(0, 1)

# Estimação Filtro de Kalman
def DoEstimation(XEst, PEst, z):
	Fp = F
	Gp = G
	Qp = Q
	Rp = R
	Hp = H

	# predição
	XPred = Fp.dot(XEst)
	PPred = Fp.dot(PEst).dot(Fp.T) + Gp.dot(Qp).dot(Gp.T)

	Innovation = z - H.dot(XPred)
	S = Hp.dot(PPred).dot(Hp.T) + Rp
	W = PPred.dot(Hp.reshape(2, 1)).dot(S**-1) # A operação de transpor (.T) falha aqui, tive que usar reshape
	# if k == 5:
	# 	print(W)
	# update
	XEst = XPred + W * Innovation
	PEst = PPred - W.dot(S).dot(W.T)

	return XEst, PEst, S, Innovation

# Simulação
def DoWorldSimulation(k):
	global XTrue
	oldpos = XTrue[0]
	oldvel = XTrue[1]

	cxtau = 500 # fator exponencial espacial para densidade da atmosfera
	AtmDensityScaleFactor = 1 - np.exp(-(X0 - oldpos) / cxtau)
	c = AtmDensityScaleFactor * Drag
	# Limitar entre 0 e c
	c = min(max(c, 0), Drag)
	# Integral de Euler
	acc = (-c * oldvel - m * g + Thrust) / m + SigmaQ * random.normal(0, 1)
	newvel = oldvel + acc * dT
	newpos = oldpos + oldvel * dT + 0.5 * acc * dT**2
	XTrue = np.array([newpos, newvel])

def DoControl(k, XEst):
	global ChuteOpen, RocketsOn, Landed
	global Thrust, Drag
	if XEst[0] < OpenChuteHeight and not ChuteOpen:
		# abrir paraquedas
		ChuteOpen = True
		Drag = ChuteDrag
		print("Abrindo paraquedas em", dT * k, "s")
		ChuteTime = k * dT

	if XEst[0] < RocketBurnHeight:
		if not RocketsOn:
			print("Largando paraquedas em", dT * k, "s")
			print("Acionando foguetes")
			BurnTime = k * dT

		RocketsOn = True
		Drag = 0
		Thrust = (m * XEst[1]**2 - 1) / (2 * XEst[0]) + 0.99 * m * g

	if XEst[0] < 1:
		print("Aterrisou em", k * dT, "s")
		Landed = True
		LandingTime = k * dT

def DoStore(k, XTrue, XEst, PEst, Innovation, S, z):
	if k == 0:
		DoStore.time = []
		DoStore.XTrue = []
		DoStore.XEst = []
		DoStore.PEst = []
		DoStore.Innovation = []
		DoStore.S = []
		DoStore.z = []

	DoStore.time += [k * dT]
	DoStore.XTrue += [XTrue[0]]
	DoStore.XEst += [XEst[0]]
	DoStore.PEst += [np.diag(PEst)]
	DoStore.Innovation += [Innovation]
	DoStore.S += [S]
	DoStore.z += [z]

def DoMarsGraph():
	plt.plot(DoStore.time, DoStore.XEst, 'r-.')
	plt.plot(DoStore.time, DoStore.XTrue, 'b-.')
	plt.show()

	plt.figure()
	plt.ylim(-10, 10)
	error = np.empty(len(DoStore.XEst))
	for i in range(len(error)):
		error[i] += [DoStore.XTrue[i] - DoStore.XEst[i]]
	print(error)
	plt.plot(DoStore.time, error)
	plt.show()

################## Execução
while not Landed and k < StopTime/dT:
# while not Landed and k < 3:
	DoWorldSimulation(k)
	z = GetSensorData(k)
	XEst, PEst, S, Innovation = DoEstimation(XEst, PEst, z)

	DoControl(k, XEst)

	DoStore(k, XTrue, XEst, PEst, Innovation, S, z)

	k += 1

DoMarsGraph()