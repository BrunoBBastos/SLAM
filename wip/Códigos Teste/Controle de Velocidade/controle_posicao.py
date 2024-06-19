import numpy as np
import paho.mqtt.client as mqtt
import simplejson as json

# parâmetros do robô
RAIO_RODA = 31.75/1000 
BASE_RODAS = 120 / 1000 # distância entre rodas
RPM = 100 # rotações por minutos do motor, estão descalibradas

# velocidade linear máxima das rodas
MAX_VEL = RPM/60 * 2 * np.pi * RAIO_RODA
# correção por causa da diferença de velocidade dos motores
MAX_VEL = 0.3


goal = np.array([[1.6], [1.60], [np.pi/2]])
kp_linear = 0.3
kp_angular = 0.6

odomMQTT = np.array([[0.0], [0.0], [0.0]])
obsMQTT = np.array([[0.0], [0.0], [-1]])

newOdom = False
RunSim = False


def getOdometry():
	global odomMQTT
	prov = np.copy(odomMQTT)
	return prov

#---------- BEGIN MQTT ----------#
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT broker with "+str(reason_code))
        connection_status = True
        client.subscribe(topic_state)
    else:
        print("Failed to connect to MQTT broker with "+str(reason_code))

def on_message(client, userdata, msg):
    # if msg.topic == state_topic:
    #     update_from_state_msg(msg.payload)
    pass

def on_connect(client, userdata, flags, reason_code, properties):
	if(reason_code==0):
		print("Connected to MQTT broker "+ str(reason_code))
		client.subscribe("robot/odometria")
		client.subscribe("robot/observation")
		client.subscribe("controle/stop")
	else:
		print("Failed to Connect to MQTT broker "+ str(reason_code))

def on_message(client, userdata, msg):
	global odomMQTT, obsMQTT, RunSim, newOdom
	
	if (msg.topic == "robot/odometria"):
		odomstr = str(msg.payload.decode())
		odomstr = json.loads(odomstr)
		odomvalues = odomstr["odometria"].split(', ')
		odomMQTT = (np.array([[odomvalues[0]], [odomvalues[1]], [odomvalues[2]]], dtype = 'float32'))
		newOdom = True

	elif (msg.topic == "robot/observation"):
		obsstr = str(msg.payload.decode())
		obsstr = json.loads(obsstr)
		obsMQTT = np.array([[obsstr["r"]], [obsstr["theta"]], [obsstr["label"]]])

	elif (msg.topic == "controle/stop"):
			RunSim = False
	else:
		print("foi para Outro tópico")

def on_subscribe(client, obj, mid, reason_code_list, properties):
    print("Subscribed: " + str(mid) + " " + str(reason_code_list))

def on_publish(client, obj, mid, reason_code, properties):
	pass

#---------- END MQTT ----------

'''
Ajusta um ângulo para seu equivalente no intervalo [-pi, pi]
'''
def wrap_angle(ang):
	if ang > np.pi:
		return ang - 2 * np.pi
	elif ang < -np.pi:
		return ang + 2 * np.pi
	else:
		return ang

'''
Mantém o ângulo dentro do intervalo [-pi/2, pi/2] saturando valores
no limite do intervalo.
Usado para modelos de robôs unidirecionais.
'''
def saturate_angle(ang):
	if ang < -np.pi/2:
		return -np.pi/2
	elif ang > np.pi/2:
		return np.pi/2
	else:
		return ang

'''
Mantém o ângulo dentro do intervalo [-pi/2, pi/2] espelhando valores
maiores ou menores no eixo y.
Usado para modelos de robôs bidirecionais.
'''
def normalize_angle(ang):
	if ang < -np.pi/2:
		return -ang -np.pi
	elif ang > np.pi/2:
		return -ang + np.pi
	else:
		return ang

''' INCOMPLETO
Gera uma velocidade linear e uma velocidade angular para controle de posição
de um robô a partir de sua posição atual e de uma referência a ser seguida.
    args:
        pos: pose do robô no formato np.array([[x], [y], [theta]])
		ref: posição da referência a ser seguida no mesmo formato de `pos`
	return:
        v: velocidade linear
		w: velocidade angular
'''
def generate_control_signal(pos, ref):
	# encontrar um vetor que aponta do robô para a referência
    delta_position = ref[0:2] - pos[0:2]
	# encontrar a orientação do vetor anterior
    delta_theta = np.arctan2(delta_position[1, 0], delta_position[0, 0])
	
    # encontrar o erro angular
    error_theta = delta_theta - pos[2]
    error_theta = wrap_angle(error_theta)
	# ecnontrar o erro de distância linear
    error_distance = np.linalg.norm(delta_position) * np.cos(error_theta)
    
    # INCOMPLETO em versões futuras, usar um controle melhor
    v = kp_linear * error_distance
    w = kp_angular * error_theta
    return v, w

''' INCOMPLETO
Gera velocidades para as rodas esquerda e direita a partir de uma combinação
de velocidades linear e angular
    args:
        v: velocidade linear escalar
		w: velocidade angular escalar
	return:
        v_left: velocidade da roda esquerda
		v_right: velocidade da roda direita
'''
def generate_wheel_speed(v, w):
	# INCOMPLETO transformar em função da `classe Robo`
	v_right = (v + w * BASE_RODAS) / 2
	v_left = (v - w * BASE_RODAS) / 2
	return v_left, v_right

''' INCOMPLETO
Gera uma referência para o controle de posicionamento tal que ao seguí-la,
o robô termine na pose referência original com a orientação especificada.
    args:
        pos: pose do robô no formato np.array([[x], [y], [theta]])
		ref: posição da referência a ser seguida no mesmo formato de `pos`
	return:
        mobile_reference: referência para o controle de posicionamento
'''
def generate_false_reference(pos, ref):
	# encontrar um vetor que aponta para a referência
    delta_position = ref[0:2] - pos[0:2]
	# encontrar a orientação do vetor anterior
    beta = np.arctan2(delta_position[1, 0], delta_position[0, 0])
    beta = wrap_angle(beta)
	
    # gamma é o ângulo entre a orientação desejada
    # e a orientação do primeiro vetor `delta_position`
    gamma = beta - ref[2, 0]
    gamma = wrap_angle(gamma)
	
    # o trabalho que propôs esse cntrole explica que há duas formas de lidar
	# com gamma, que deve pertencer a um intervalo específico, comente um:
    gamma = saturate_angle(gamma)
    # gamma = normalize_angle(gamma)
	
    # INCOMPLETO o trabalho afirma ser (beta - gamma), mas parece estar errado 
    mobile_reference = np.array(
		[
			[pos[0,0] + delta_position[0, 0] * np.cos(beta + gamma)], # x_ref
		    [pos[1,0] + delta_position[1, 0] * np.sin(beta + gamma)], # y_ref
            [0] # será preenchido em seguida
		])
	# encontrar o vetor que aponta do robô à nova referência
    mobileDelta = mobile_reference[0:2] - pos[0:2]
	# encontrar a orientação da nova referência
    mobile_reference[2, 0] = np.arctan2(mobileDelta[1, 0], mobileDelta[0, 0])
    mobile_reference[2, 0] = wrap_angle(mobile_reference[2, 0])
	
    return mobile_reference

def main():
    global newOdom
    while True:
        if newOdom:
            falseRef = generate_false_reference(odomMQTT, goal)
            v, w = generate_control_signal(odomMQTT, falseRef)
            # v, w = generate_control_signal(odomMQTT, goal)
            l, r = generate_wheel_speed(v, w)
            # print(l, r)
            msg = f'{{"type":"vel", "l":"{l[0]}", "r":"{r[0]}"}}'
            client.publish(topic, msg)
            # print(msg)
            newOdom = False


if __name__ == '__main__':

    # configurar o mqtt para coletar e envio de dados
	# INCOMPLETO implementar as seguintes funcionalidades na nossa
	# própria biblioteca, e padronizar os canais de comunicação
    mqtt_broker = "10.7.220.187"
    mqtt_port = 1883
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="controle_posicao")
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe
    client.on_publish = on_publish
    client.connect(mqtt_broker, mqtt_port)
    client.loop_start()
    topic = 'slam/ref'
    topic_state = "robot/state"
    topic_odometria = "slam/odometria"

    main()