import numpy as np
from RobotSLAM import *

def angleWrap(ang):
	if ang > np.pi:
		return ang - 2 * np.pi
	elif ang < -np.pi:
		return ang + 2 * np.pi
	else:
		return ang

# def tcomp(tab, tbc):
# 	result = tab[2] + tbc[2]
# 	if result > np.pi or result <= -np.pi:
# 		result = angleWrap(result)
# 	s = np.sin(tab[2])
# 	c = np.cos(tab[2])
# 	matriz = np.array([c, -s, s, c]).reshape(2, 2)
# 	tac = tab[0:2] + matriz.dot(tbc[0:2])
# 	tac = np.append(tac, [result], axis = 0)
# 	tac = tac.astype('float')
# 	return tac

# def tinv(tab):
# 	tba = np.zeros(tab.shape)
# 	for t in range(0, len(tab), 3):
# 		tba = tinv1(tab[t:t+3])
# 	return tba

def tcomp(tab, tbc):
    # Material do Newman
	# Eq 4.26, pag 53
    result = tab[2] + tbc[2]
    result = angleWrap(result)

    s = np.sin(tab[2])
    c = np.cos(tab[2])
    
    tac = np.zeros((3, 1))
    tac[0, 0] = tab[0] + c * tbc[0] - tbc[1] * s
    tac[1, 0] = tab[1] + s * tbc[0] + tbc[1] * c
    tac[2, 0] = result

    return tac


def tinv(tab):
    def tinv1(tab):
        tba = np.zeros(3)
        s = np.sin(tab[2])
        c = np.cos(tab[2])
        tba[0] = (-1) * tab[0] * c - tab[1] * s
        tba[1] = tab[0] * s - tab[1] * c
        tba[2] = (-1) * tab[2]
        return tba

    num_rows = len(tab)
    tba = np.zeros(num_rows)

    for i in range(0, num_rows, 3):
        aux = tab[i:i+3]
        aux1 = tinv1(aux)
        tba[i:i+3] = aux1

    return tba

# def tinv1(tab):
# 	s = np.sin(tab[2])
# 	c = np.cos(tab[2])
# 	tba = np.array([[-tab[0] * c - tab[1] * s], [tab[0] * s - tab[1] * c], [-tab[2]]])
# 	return tba

def GetObsJac(xPred, iFeature, Map):
    jH = np.zeros((2, 3))
    Delta = Map[:, iFeature].reshape(2, 1) - xPred[0:2]
    r = np.linalg.norm(Delta)
    jH[0, 0] = -Delta[0, 0] / r
    jH[0, 1] = -Delta[1, 0] / r
    jH[1, 0] = Delta[1, 0] / r**2
    jH[1, 1] = -Delta[0, 0] / r**2
    jH[1, 2] = -1
    return jH

def doObservationModel(xVeh, iFeature, Map):
    Delta = Map[:, iFeature].reshape(2, 1) - xVeh[0:2]
    at = np.arctan2(Delta[1], Delta[0]) - xVeh[2]
    z = np.array([[np.linalg.norm(Delta)], [at[0]]])
    z[1, 0] = angleWrap(z[1, 0])
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


    

# Cria o objeto robô e espera ele se conectar ao dispositivo
robot = Robot("10.0.0.105")
while robot.connection_status == False:
    pass

#Mapa do features distribuídas pelo LII
Map = np.array([[-3.425, 0.80, 1.34, -1.74, -0.80, -0.80],
                [0.0,	 0.785,	0.0,  -1.03, 0.85,	-0.98]])

UTrue = np.diag([0.01, 0.01, 1 * np.pi / 180])**2
RTrue = np.diag([2, 3 * np.pi / 180])**2

UEst = 1 * UTrue
REst = 1 * RTrue

XTrue = np.array([[0.0], [0.0], [0.0]])

xOdomNow = np.copy(XTrue)
xOdomLast = np.copy(XTrue) #GetOdometry(0)

XEst = np.copy(XTrue)
PEst = np.diag([1, 1, (np.pi / 180)**2])

k = 0
XTudo = np.copy(XTrue)
while True:
    # Encontrar o controle u
	# xOdomNow = robot.readOdometry() #orig
	xOdomNow += robot.readOdometry()
	u = tcomp(tinv(xOdomLast), xOdomNow)
	xOdomLast = np.copy(xOdomNow)
    
	# Predição
	XPred = tcomp(XEst, u)	
	XPred[2] = angleWrap(XPred[2])
	PPred = J1(XEst, u).dot(PEst).dot(J1(XEst, u).T) + J2(XEst, u).dot(UEst).dot(J2(XEst, u).T)
		
	z, iFeature = robot.distanceSensorMeasurement #GetObservation()

	# Update
	if z != -1:
		zPred = doObservationModel(XPred, iFeature, Map)
		jH = GetObsJac(XPred, iFeature, Map)

		innov = (z - zPred).reshape(2, 1)
		innov[1, 0] = angleWrap(innov[1, 0])

		S = jH.dot(PPred).dot(jH.T) + REst
		S = S.astype('float')
		W = PPred.dot(jH.T).dot(np.linalg.inv(S))

		XEst = XPred + W.dot(innov)
		XEst[2, 0] = angleWrap(XEst[2, 0])
		XEst = XEst.astype('float')

		# Forma de 'Joseph', dita ser numericamente estável
		I = np.eye(3)
		PEst = (I - W.dot(jH)).dot(PPred).dot((I - W.dot(jH)).T) + W.dot(REst).dot(W.T)
		PEst = 0.5 * (PEst + PEst.T)

	else:
		XEst = np.copy(XPred)
		PEst = np.copy(PPred)
		innov = np.empty((2, 1))

	k += 1
	if k%1000==0:
		print(XEst)
