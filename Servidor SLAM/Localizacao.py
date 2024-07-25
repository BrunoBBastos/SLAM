#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BIBLIOTECAS
import numpy as np
from numpy.linalg import inv
import math
import matplotlib.pyplot as plt
import matplotlib.patches as pat
import paho.mqtt.client as mqtt
import simplejson as json
# import keyboard
# import scipy.linalg as LA
# import numpy.matlib
# from numpy import random



#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRANSFORMAÇÕES E UTILIDADES
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
	tba = np.array([-tab[0] * c - tab[1] * s,
				 	tab[0] * s - tab[1] * c,
					-tab[2]]).reshape(3, 1)
	return tba
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRANSFORMÇÕES E UTILIDADES

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SINAIS E MODELOS

def GetObservation(k):
	global obsMQTT
	if obsMQTT [2]== -1:
		return (np.zeros((2,1)), -1)

	prov = np.copy(obsMQTT)
	# prov = [[prov[0], [prov[1]]], prov[2]]
	z = np.array([[prov[0, 0]], [prov[1, 0]]]).astype(np.float64)
	iFeature = prov[2, 0]
	return (z, iFeature)

def GetObsJac(xPred, iFeature, Map):
	jH = np.zeros((2, 3))
	Delta = Map[iFeature].reshape(2, 1) - xPred[0:2]
	r = np.linalg.norm(Delta)
	jH[0, 0] = -Delta[0] / r
	jH[0, 1] = -Delta[1] / r
	jH[1, 0] = Delta[1] / r**2
	jH[1, 1] = -Delta[0] / r**2
	jH[1, 2] = -1
	return jH

def doObservationModel(xVeh, iFeature, Map):
	# Delta = Map[:, iFeature].reshape(2, 1) - xVeh[0:2]
	Delta = Map[iFeature].reshape(2, 1) - xVeh[0:2]
	at = np.arctan2(Delta[1], Delta[0]) - xVeh[2] # Evitar incluir array dentro de array
	z = np.array([np.linalg.norm(Delta), at[0]]).reshape(2, 1)
	z[1] = angleWrap(z[1])
	return z

def GetOdometry(k):
	global odomMQTT
	prov = np.copy(odomMQTT)
	return prov

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

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRÁFICOS
def DoGraphs():
	global XTrue, XEst, PEst
	plt.clf()
	plt.plot([0, 1.60, 1.60, 0, 0], [0, 0, 1.60, 1.60, 0], 'b',label="Referência") #Referencia
	# plt.axis([-4, 4, -4, 4])
	plt.axis([-0.5, 2.1, -0.5, 2.1])
	plt.title('Odometria')
	x_vals = [x[0] for x in Map.values()]
	y_vals = [y[1] for y in Map.values()]
	# plt.plot(x_vals, y_vals, '*k',label = "Features")
	plt.plot(XEst[0],XEst[1],"r+")
	# PlotEllipse(XEst, PEst, 3)
	DrawRobot(XEst, 'red')
	#DrawRobot(XTrue, 'blue')
	DrawRobot(XPredOnly, 'green')
	plt.axis('equal')
	plt.pause(0.001)

def DrawRobot(Xr, col):
	p = 0.02
	xmin, xmax, ymin, ymax = plt.axis()
	l1 = (xmax - xmin) * p
	l2 = (ymax - ymin) * p
	P = np.array([[-.5, .5, 0, -0.5], [-1.3, -1.3, 0.6,-1.3]])
	theta = Xr[2][0]-np.pi/2
	c = math.cos(theta)
	s = math.sin(theta)
	rotation = np.array([[c, -s], [s, c]], dtype = float)
	P = rotation.dot(P)
	P[0, :] = P[0, :] * l1 + Xr[0]
	P[1, :] = P[1, :] * l2 + Xr[1]
	plt.plot(P[0, :], P[1, :], color = col)
	plt.plot(Xr[0], Xr[1], color = col)

def PlotEllipse(x, Po, nSigma):
	# x = np.copy(x[0:2])
	# P = np.empty((2, 2)) # Copiando dessa forma por causa dos problemas de formatação da np
	# for i in range(2):
	# 	for j in range(2):
	# 		P[i, j] = Po[i, j][0]
	# if not 0 in np.diag(P):
	# 	D, V = np.linalg.eig(P)
	# 	D = np.diag(D)
	# 	s = np.linspace(0, 2 * np.pi, 50)
	# 	pX = []
	# 	pY = []
	# 	for i in s:
	# 		y = nSigma * np.array([[np.cos(i)], [np.sin(i)]])
	# 		el = V.dot(LA.sqrtm(D)).dot(y)
	# 		el =  np.matlib.repmat(x, 1, len(el[0]) + 1) + np.concatenate((el, el), axis = 1)
	# 		pX += [el[0,0]]
	# 		pY += [el[1,0]]
	# 	plt.plot(pX, pY, color = 'red')
	eigenvalues, eigenvectors = np.linalg.eig(Po)
	ang = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
	largura = 2 * nSigma * np.sqrt(eigenvalues[0])
	altura = 2 * nSigma * np.sqrt(eigenvalues[1])

	ellipse = pat.Ellipse(xy=(x[0], x[1]),
					   	  width=largura, height=altura, angle=ang, alpha=0.5,
						  edgecolor='blue', facecolor = 'none')
	plt.gca().add_patch(ellipse)
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRÁFICOS

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MQTT

def on_connect(client, userdata, flags, reason_code, properties):
	if(reason_code==0):
		print("Connected to MQTT broker "+ str(reason_code))
		client.subscribe("robot/odometria")
		client.subscribe("robot/observation")
		client.subscribe("controle/stop")
	else:
		print("Failed to Connect to MQTT broker "+ str(reason_code))

def on_message(client, userdata, msg):
	global odomMQTT, obsMQTT, RunSim
	if (msg.topic == "robot/odometria"):
		odomstr = str(msg.payload.decode())
		odomstr = json.loads(odomstr)
		odomvalues = odomstr["odometria"].split(', ')
		odomMQTT = (np.array([[odomvalues[0]],
							  [odomvalues[1]],
							  [odomvalues[2]]],
							  dtype = 'float32'))

	elif (msg.topic == "robot/observation"):
		obsstr = str(msg.payload.decode())
		obsstr = json.loads(obsstr)
		obsMQTT = np.array([[obsstr["r"]],
					  		[obsstr["theta"]],
							[obsstr["label"]]])

	elif (msg.topic == "controle/stop"):
			RunSim = False
	else:
		print("foi para Outro tópico")

def on_subscribe(client, obj, mid, reason_code_list, properties):
    print("Subscribed: " + str(mid) + " " + str(reason_code_list))


def on_publish(client, obj, mid, reason_code, properties):
	pass
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MQTT

###################################################################### EXECUÇÃO

################# MQTT ####################
mqtt_broker = "10.7.220.187"
mqtt_port = 1883
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id= "localizacao")
client.on_connect = on_connect
client.on_message = on_message
client.on_subscribe = on_subscribe
client.on_publish = on_publish
client.connect(mqtt_broker, mqtt_port)
client.loop_start()
topic_odometria = "slam/odometria"
topic_xest = "slam/xest"
##########################################

Map = {
	"cubo1":		np.array([0., -0.20]),
	"cubo2":		np.array([-0.20, 1.60]),
	"cubo3":		np.array([1.60, 1.80]),
}

UTrue = np.diag([0.01, 0.01, 2* np.pi / 180])**2
RTrue = np.diag([0.5, 1.5* np.pi / 180])**2

UEst = 1 * UTrue
REst = 1 * RTrue

# XTrue = np.array([-0.80, -0.80, 0.]).reshape(3, 1)
XTrue = np.array([0., 0., 0.]).reshape(3, 1)

xOdomNow = np.copy(XTrue)
xOdomLast = np.copy(XTrue)

odomMQTT = np.array([[0.0], [0.0], [0.0]])
obsMQTT = np.array([[0.0], [0.0], [-1]])

XEst = np.copy(XTrue)
XPredOnly = np.copy(XTrue)
PEst = np.diag([.1, .1, (1 * np.pi / 180)**2])

RunSim = True

lxXest = list()
lyXest = list()

lxXpred = list()
lyXpred = list()


gif = 0
k = 0
while(RunSim):
	# PREDIÇÃO
	xOdomNow = GetOdometry(k)
	u = tcomp(tinv(xOdomLast), xOdomNow)
	xOdomLast = np.copy(xOdomNow)

	XPred = tcomp(XEst, u)
	XPred[2] = angleWrap(XPred[2])
	# XPredOnly = np.copy(XPred)
	XPredOnly = tcomp(XPredOnly, u)

	PPred = J1(XEst, u).dot(PEst).dot(J1(XEst, u).T) + J2(XEst, u).dot(UEst).dot(J2(XEst, u).T)
	(z, iFeature) = GetObservation(k)

	if iFeature != -1:
		# UPDATE
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
		#PEst = 0.5 * (PEst + PEst.T) # WTF?

	else:
		XEst = np.copy(XPred)
		PEst = np.copy(PPred)
		innov = np.empty((2, 1))
		# S = np.eye(2) * None

	lxXest.append(XEst[0,0])
	lyXest.append(XEst[1,0])

	lxXpred.append(XPredOnly[0,0])
	lyXpred.append(XPredOnly[1,0])

	##MQTT
	# msg = f'{{"x":{XEst[0,0]},"y":{XEst[1,0]},"theta":{XEst[2,0]}}}'
	# client.publish(topic_xest, msg)
	# msg = f'{{"x":{XPredOnly[0,0]},"y":{XPredOnly[1,0]},"theta":{XPredOnly[2,0]}}}'
	# client.publish(topic_odometria, msg)

	# print(XPredOnly)

	if k%15 == 1:
		DoGraphs()
		# plt.savefig('gif' + str(gif) + '.jpg')
		# gif+=1
	k += 1
print("Fim")
print("XEst:\n",XEst)
print("PEst:\n",PEst)
print("XPredonly:\n",XPredOnly)
plt.clf()
DoGraphs()

plt.plot(lxXpred,lyXpred,"--g",label = "Odometria")
# plt.plot(lxXest,lyXest,"--r",label = "Estimado")
plt.xlabel("$x$ (m)")
plt.ylabel("$y$ (m)")
plt.legend()
plt.plot
plt.show()