
import serial
import time


COM_ARDUINO = True
ARDUINO_PORT = 'COM3'
TEMPO_ENVIO = 0.5

if COM_ARDUINO == True:
    # Configurar a porta serial
    porta_serial = serial.Serial(ARDUINO_PORT, 9600)  # Replace 'COM4' with the correct serial port of your Arduino

time.sleep(1)  # Wait for serial communication to stabilize


# iniciar entrada de informaçoes

#             # BDA BDB BEA  BEB  PDA  PDB  PEA  PEB  CA
#     angles = [0,  0,   0,   0,   0,   0,   0,   0,  0]
angles = [0, 1, 2, 3, 4, 5, 6, 7, 8]

angles[0] = 0
angles[1] = 50
angles[2] = 0
angles[3] = 0
angles[4] = 0
angles[5] = 0
angles[6] = 0
angles[7] = 0
angles[8] = 0
# 0, 0, 0, 0, 0, 0, 0, 0,  #

tempo_anterior = time.time()

while (1):
    if (time.time() - tempo_anterior)>=TEMPO_ENVIO:
        enviar_dados = ""
        for i in range(len(angles)):
            enviar_dados = enviar_dados + str(angles[i]) + ","
        enviar_dados = enviar_dados + "#"
        print(enviar_dados)

        if COM_ARDUINO == True:
            porta_serial.write(enviar_dados.encode())

        tempo_anterior = time.time()


if COM_ARDUINO == True:
    # Fechar a porta serial
    porta_serial.close()
