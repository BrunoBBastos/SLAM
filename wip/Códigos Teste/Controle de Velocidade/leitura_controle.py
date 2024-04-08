import numpy as np
import hid
import serial
import paho.mqtt.client as mqtt

raioRoda = 31.75/1000
rpm = 100
maxVel = rpm/60 * 2 * np.pi * raioRoda


#---------- BEGIN MQTT ----------#
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT broker with "+str(reason_code))
        connection_status = True
        client.subscribe(state_topic)
    else:
        print("Failed to connect to MQTT broker with "+str(reason_code))

def on_message(client, userdata, msg):
    # if msg.topic == state_topic:
    #     update_from_state_msg(msg.payload)
    pass

#---------- END MQTT ----------




#---------- FUNÇÕES -----------

def mapFloat(var, in_min, in_max, out_min, out_max):
    return (var - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def analogRead2Velocity(x, y):
    w = mapFloat(x, 127, 0, 0, maxVel)
    v = mapFloat(y, 127, 0, 0, maxVel)
    esq = v - w
    dir = v + w
    return esq, dir

def main():
    #efetua a leitura do joystick constantemente
    while True:
        gamepadReport = gamepad.read(64)
        if gamepadReport:
            #os botões de interessa são 'x', 'quadrado' e o analog esquerdo
            #analógico esquerdo: gera sinal de velocidade
            #'x' envia o sinal
            #'sq' quadrado: encerra o programa
            xAxis = gamepadReport[0]
            yAxis = gamepadReport[1]
            xButton = gamepadReport[5] & 0b00000010
            sqButton = gamepadReport[5] & 0b00000001
            if xButton:
                sig = np.array([xAxis, yAxis], dtype = float)
                sig = np.array([mapFloat(sig[0], 127, 0, 0., 0.3), mapFloat(sig[1], 127, 0, 0., 1.)])
             
                sig = np.array([sig[1] - sig[0], sig[1] + sig[0]]) * 255
                sig = np.clip(sig, -255, 255)

                sig = np.array(sig)
                sig = sig.astype('int')

                # esq, dir = analogRead2Velocity(xAxis, yAxis)
                # msg = f'{{"type":"vel", "l":"{esq}", "r":"{dir}"}}'

                msg = f'{{"type":"pwm", "l":"{sig[0]}", "r":"{sig[1]}"}}'

                client.publish(topic, msg)
                print(msg)
            if sqButton:
                break

if __name__ == '__main__':
    

    state_topic = "robot/state"
    mqtt_broker = "10.7.220.187"
    mqtt_port = 1883
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="gamepad")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(mqtt_broker, mqtt_port)
    client.loop_start()
    topic = 'slam/ref'

    # ------- CONFIG GAMEPAD -------
    # Leitura dos dispositivos usb, rodar uma vez pra descobrir o código que vai no gamepad.open()
    # for device in hid.enumerate():
    #     print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")

    gamepad = hid.device()
    gamepad.open(0x12bd, 0xc003)
    gamepad.set_nonblocking(True)
    # ------- CONFIG GAMEPAD -------

    main()