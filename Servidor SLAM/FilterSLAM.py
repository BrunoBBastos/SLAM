import numpy as np


def angleWrap(ang):
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

def tinv(tab):
	tba = np.zeros(tab.size)
	for t in range(0, tab[0].size, 3):
		tba = tinv1(tab[t:t+3])
	return tba

def tinv1(tab):
	s = np.sin(tab[2])
	c = np.cos(tab[2])
	tba = np.array([[-tab[0] * c - tab[1] * s], [tab[0] * s - tab[1] * c], [-tab[2]]])
	return tba


class EKF:
    def __init__(self, Map = None):

        # Map = np.array([[-3.425, 0.785, 1.34, -1.74, -0.80, -0.80],
		# 		[0.0,	 0.80,	0.0,  -1.03, 0.85,	-0.98]])
        self.Map = Map

        self.UTrue = np.diag([0.01, 0.01, 1 * np.pi / 180])**2
        self.RTrue = np.diag([2, 3 * np.pi / 180])**2

        self.UEst = 1 * self.UTrue
        self.REst = 1 * self.RTrue
        
        self.XTrue = np.array([[0], [0], [0]])

        self.xOdomLast = self.XTrue #GetOdometry(0)

        self.XEst = np.copy(self.XTrue)
        self.PEst = np.diag([1, 1, (np.pi / 180)**2])

	
    def GetObsJac(xPred, iFeature, Map):
        jH = np.zeros((2, 3))
        Delta = Map[:, iFeature].reshape(2, 1) - xPred[0:2]
        r = np.linalg.norm(Delta)
        jH[0, 0] = -Delta[0] / r
        jH[0, 1] = -Delta[1] / r
        jH[1, 0] = Delta[1] / r**2
        jH[1, 1] = -Delta[0] / r**2
        jH[1, 2] = -1
        return jH

    def doObservationModel(self, xVeh, iFeature, Map):
        Delta = Map[:, iFeature].reshape(2, 1) - xVeh[0:2]
        at = np.arctan2(Delta[1], Delta[0]) - xVeh[2]
        z = np.array([np.linalg.norm(Delta), at[0]])
        z[1] = angleWrap(z[1])
        return z

    def J1(self, x1, x2):
        s1 = np.sin(x1[2])
        c1 = np.cos(x1[2])

        Jac = np.array([[1, 0, -x2[0] * s1 - x2[1] * c1],
                [0, 1, x2[0] * c1 - x2[1] * s1],
                [0, 0, 1]], dtype = object)
        return Jac

    def J2(self, x1, x2):
        s1 = np.sin(x1[2])
        c1 = np.cos(x1[2])

        Jac = np.array([[c1, -s1, 0],
                [s1, c1, 0],
                [0, 0, 1]], dtype = object)
        return Jac

    def localization(self):
        xOdomNow = self.GetOdometry()
        u = tcomp(tinv(xOdomLast), xOdomNow)
        xOdomLast = np.copy(xOdomNow)

        XPred = tcomp(XEst, u) 
        XPred[2] = angleWrap(XPred[2])
        XPredOnly = tcomp(XPredOnly, u)
        PPred = self.J1(XEst, u).dot(PEst).dot(self.J1(XEst, u).T) + J2(XEst, u).dot(self.UEst).dot(self.J2(XEst, u).T)
        z, iFeature = self.GetObservation()


        if z.size != 0:
            zPred = self.doObservationModel(XPred, iFeature, self.Map)
            jH = self.GetObsJac(XPred, iFeature, self.Map)

            innov = (z - zPred).reshape(2, 1)
            innov[1] = angleWrap(innov[1])

            S = jH.dot(PPred).dot(jH.T) + self.REst
            S = S.astype('float')
            W = PPred.dot(jH.T).dot(np.linalg.inv(S))

            XEst = XPred + W.dot(innov)
            XEst[2] = angleWrap(XEst[2])
            XEst = XEst.astype('float')

            # Forma de 'Joseph', dita ser numericamente estável
            I = np.eye(3)
            self.PEst = (I - W.dot(jH)).dot(PPred).dot((I - W.dot(jH)).T) + W.dot(self.REst).dot(W.T)
            self.PEst = 0.5 * (self.PEst + self.PEst.T)

        else:
            self.XEst = np.copy(XPred)
            self.PEst = np.copy(PPred)
            innov = np.empty((2, 1))

        
