import numpy as np
import hid
import serial
import paho.mqtt.client as mqtt
import simplejson as json


# INCOMPLETO - Implementar lib do robô
# parâmetros do robô
RAIO_RODA = 31.75/1000 
BASE_RODAS = 120 / 1000 # distância entre rodas
RPM = 100 # rotações por minuto do motor, estão descalibrados
# velocidade linear máxima das rodas
MAX_VEL = RPM/60 * 2 * np.pi * RAIO_RODA
# correção por causa da diferença de velocidade dos motores
MAX_VEL = 0.3

# Lista de tópicos que serão monitorados aqui
# atualize a on_message() de acordo
topics_subscribe = [
	                "robot/odometria",
					"robot/observation",
					"controle/stop"
					]

#---------- BEGIN MQTT ----------#

def on_connect(client, userdata, flags, reason_code, properties):
	global topics_subscribe
	if(reason_code==0):
		print("Connection to MQTT broker: "+ str(reason_code))
		for topic in topics_subscribe:
			client.subscribe(topic)
	else:
		print("Failed to Connect to MQTT broker "+ str(reason_code))

def on_message(client, userdata, msg):
	global odomMQTT, obsMQTT, RunSim, newOdom
	
	if (msg.topic == "robot/odometria"):
		odomstr = str(msg.payload.decode())
		odomstr = json.loads(odomstr)
		odomvalues = odomstr["odometria"].split(', ')
		odomMQTT = (np.array([[odomvalues[0]],
						      [odomvalues[1]],
							  [odomvalues[2]]],
							  dtype = 'float32'))
		newOdom = True

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

#---------- END MQTT ----------

#---------- FUNÇÕES -----------

def mapFloat(var, in_min, in_max, out_min, out_max):
    return (var - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def main():
    print("-- Leitura de joystick --")
    #efetua a leitura do joystick sempre que possível
    while True:
        gamepadReport = gamepad.read(64)
        if gamepadReport:
			# Os seguintes comandos são respectivos ao modelo de joystick
            # disponível no laboratório:
            # Os botões de interessa são 'x', 'quadrado' e o analog esquerdo
            # analógico esquerdo: gera sinal de velocidade linear ^/v e angular </>
            # 'x' envia o sinal
            # 'sq' quadrado: encerra o programa
            xAxis = gamepadReport[0]
            yAxis = gamepadReport[1]
            xButton = gamepadReport[5] & 0b00000010
            sqButton = gamepadReport[5] & 0b00000001
            if xButton:
                sig = np.array([xAxis, yAxis], dtype = float)
                sig = np.array([mapFloat(sig[0], 127, 0, 0., 0.50),
								mapFloat(sig[1], 127, 0, 0., 1.)])
				
                sig = np.array([sig[1] - sig[0], sig[1] + sig[0]]) * MAX_VEL
                sig = np.clip(sig, -MAX_VEL, MAX_VEL)
				
                msg = f'{{"type":"vel", "l":"{sig[0]}", "r":"{sig[1]}"}}'

                client.publish(topic_pub_vel, msg)
                print(msg)
				
            if sqButton:
                break

if __name__ == '__main__':

    # ------- CONFIG GAMEPAD -------
    # Leitura dos dispositivos usb, rodar uma vez pra descobrir
    # o código que vai no gamepad.open()
    # for device in hid.enumerate():
    #     print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x}
    #               {device['product_string']}")
	# while True:
        # pass

    gamepad = hid.device()
    gamepad.open(0x12bd, 0xc003)
    gamepad.set_nonblocking(True)
    # ------- CONFIG GAMEPAD -------

    # --------- CONFIG MQTT --------
    # configurar o mqtt para coletar e envio de dados
	# INCOMPLETO implementar as seguintes funcionalidades na nossa
	# própria biblioteca, e padronizar os canais de comunicação
    mqtt_broker = "10.7.220.187" # Nosso broker no lab: "10.7.220.187"
    mqtt_port = 1883
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="gamepad")
	# definir as seguintes funções no código acima de acordo com a necessidade
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe
    client.on_publish = on_publish
	# conectar ao broker
    client.connect(mqtt_broker, mqtt_port)
    client.loop_start()
    topic_pub_vel = 'slam/ref'
    # --------- CONFIG MQTT --------

    main()