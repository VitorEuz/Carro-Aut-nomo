# Bibliotecas
import cv2
import cv2 as cv
import numpy as np
import math
import serial
import time


DIREITA  = 0
ESQUERDA = 1
AMBOS = 2
RETO = 3

ANALISAR_FAIXA = 0
DISTANCIA_CENTRO_FAIXA = 170 # distancia centro do veiculo / camera até a faixa, para controle
KP = 0.3
KI = 0.005
KD = 0.05
MIN = -31
MAX = 32

FRAME_WIDTH = int(1920 / 4)
FRAME_HEIGHT = int(1080 / 4)
FRAME_CENTER = int(FRAME_WIDTH / 2)
print("Redimensionado: ", FRAME_WIDTH, "x", FRAME_HEIGHT)

# capura da area de interesse
AREA_INT_INICIO = 200
AREA_INT_FIM = 220

#Valor da Webcam
portaWeb = 0

# constantes de envio de informaçoes por serial
ENVIAR_DADOS = True    # enviar dados para o arduino
TEMPO_ENVIO  = 0.1      # tempo em segundos
PORTA_COM    = 'COM7'  # porta COM para comunicar com Arduino
porta_serial = None

# vetor para envio de dados para o Arduino -> 8 bytes
# posiçao 0 -> velocidade do carro: 0~255
# posiçao 1 -> direção do carro: 0~180 onde 90 corresponde ao centro
enviar_dados = [255, 150, 90, 90, 0, 0, 0, 0, 0]
tempo_anterior = time.time()




# Funcao: Class PID
class pid_direcao:
    # calibrado
    kp = 0.0
    ki = 0.0
    kd = 0.0

    # ajuste da integral inicial
    i = 0

    maxValor = 0
    minValor = 0
    setPoint = 0
    lastTime = 0

    def __init__(self, _setPoint, _kp, _ki, _kd, _minValor, _maxValor):
        self.setPoint = _setPoint
        self.kp = _kp
        self.ki = _ki
        self.kd = _kd
        self.minValor = _minValor
        self.maxValor = _maxValor

    def calcularSaida(self, _entrada):
        erro = self.setPoint - _entrada
        deltaTempo = 0.1

        # PID
        p = self.kp * erro
        self.i = (self.i + ((self.ki * erro) * deltaTempo))

        d = (erro * self.kd) / deltaTempo

        varPID = p + self.i + d
        if varPID > self.maxValor:
            varPID = self.maxValor
        elif varPID < self.minValor:
            varPID = self.minValor

        return varPID

# Função de callback para a trackbar, não faz nada por enquanto
def nothing(x):
    pass

# Cria uma janela onde os controles serão exibidos
cv.namedWindow('Controles')

# Cria as trackbars (sliders) para controle de Hue, Saturation e Value
cv.createTrackbar('canny_1', 'Controles', 200, 300, nothing)
cv.createTrackbar('canny_2', 'Controles', 220, 400, nothing)
cv.createTrackbar('velocidade', 'Controles', 0, 255, nothing)
cv.createTrackbar('Lado','Controles',2,4,nothing)
cv.createTrackbar('pare','Controles',0,1,nothing)



# Funcao: enviar_dados_serial()
def calcular_distancia_centro_faixa(img, num_linhas, intervalo):
    altura, largura = img.shape

    centro_y = altura // 2
    centro_x = largura // 2

    distancias_esquerda = []
    distancias_direita = []

    # Iterar por linhas, saltando por 'intervalo' linhas
    for i in range(centro_y - (num_linhas // 2) * intervalo, centro_y + (num_linhas // 2) * intervalo, intervalo):
        if i < 0 or i >= altura:
            continue  # Certifica de que não ultrapassar as bordas da imagem

        # Analisar pista direita (centro -> direita)
        for x_direita in range(centro_x, largura):
            if img[i, x_direita] >= 50:  # Detecta um pixel branco
                distancias_direita.append(x_direita - centro_x)
                break

        # Analisar pista esquerda (centro -> esquerda)
        for x_esquerda in range(centro_x, -1, -1):
            if img[i, x_esquerda] >= 50:  # Detecta um pixel branco
                distancias_esquerda.append(centro_x - x_esquerda)
                break

    # Se não encontrar branco em um dos lados, evitar divisão por zero
    media_distancia_esquerda = np.mean(distancias_esquerda) if distancias_esquerda else float('inf')
    media_distancia_direita = np.mean(distancias_direita) if distancias_direita else float('inf')

    faixaEsquerda = media_distancia_esquerda
    faixaDireita = media_distancia_direita

    return media_distancia_esquerda, media_distancia_direita, faixaEsquerda, faixaDireita

# Funcao: enviar_dados_serial()
def enviar_dados_serial():
    global tempo_anterior
    global enviar_dados
    global TEMPO_ENVIO
    global porta_serial

    if (time.time() - tempo_anterior)>=TEMPO_ENVIO:
        preparar_dados = ""
        for i in range(len(enviar_dados)):
            preparar_dados = preparar_dados + str(enviar_dados[i]) + ","
        preparar_dados = preparar_dados + "#"
        print(preparar_dados)

        if ENVIAR_DADOS == True:
            porta_serial.write(preparar_dados.encode())

        tempo_anterior = time.time()

# Funcao: fecharAplicativo()
def fecharAplicativo():
    if ENVIAR_DADOS == True:
        porta_serial.close() # Fechar a porta serial

    cap.release()
    cv.destroyAllWindows()
    quit()


# Inicializa o controlador PID com ganhos KP, KI, KD
pid_direcao = pid_direcao(DISTANCIA_CENTRO_FAIXA, KP, KI, KD, MIN, MAX)

if ENVIAR_DADOS == True:
    # Estabelecer conexao com o Arduino
    porta_serial = serial.Serial(PORTA_COM, 9600)
    time.sleep(1)

# config. de saida de texto -> Plota na tela o texto
font = cv.FONT_HERSHEY_SIMPLEX
fontScale = 0.5
fontCor = (0, 0, 255)  # BGR
fontThickness = 1

cap = cv.VideoCapture(portaWeb, cv2.CAP_DSHOW)

while(1):

    # pegar frame do video / camera
    ret, frame = cap.read()

    # se nao tiver imagem fecha a aplicacao
    if not ret:
        print("entrada cap nao detectado!")
        # fechar programa jumto com as conexoes
        fecharAplicativo()

    # ajustar o tamanho do video, quanto menor mais rapido o processamento
    frame = cv.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

    imgFiltro = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    imgFiltro = cv.GaussianBlur(imgFiltro, (5, 5), 0)

    # valores iniciais para Filtro Canny, cannyThreshold1 = 280 e cannyThreshold2 = 380
    # configurar o filtro Canny
    canny_threshold_1 = cv.getTrackbarPos('canny_1', 'Controles')
    canny_threshold_2 = cv.getTrackbarPos('canny_2', 'Controles')
    imgFiltro = cv.Canny(imgFiltro, canny_threshold_1, canny_threshold_2)

    # capura da area de interesse
    x1, y1 = 0,AREA_INT_INICIO
    x2, y2 = 480,AREA_INT_FIM
    area_interesse = imgFiltro[min(y1, y2):max(y1, y2), min(x1, x2):max(x1, x2)]

    # Configurações da função
    num_linhas = 10  # Número de linhas a serem analisadas
    intervalo = round((AREA_INT_FIM-AREA_INT_INICIO)/ num_linhas) # Intervalo entre as linhas analisadas

    # Calcular as distâncias médias
    media_esquerda, media_direita,faixaEsquerda,faixaDireita = calcular_distancia_centro_faixa(area_interesse, num_linhas, intervalo)

    # distancia pista lado direito
    if math.isinf(media_direita)==False:
        ponto_superior_esquerdo = (FRAME_CENTER, AREA_INT_INICIO)
        ponto_inferior_direito = (round(media_direita) + FRAME_CENTER, AREA_INT_FIM)
        cv.rectangle(frame, ponto_superior_esquerdo, ponto_inferior_direito, (255, 0, 0), -1)

        # Plota o texto da curva media_direito:
        text = str(media_direita)
        posicao = (int((FRAME_CENTER + 30)), AREA_INT_INICIO-10)
        cv.putText(frame, text, posicao, font, fontScale, fontCor, fontThickness, cv.LINE_AA, False)

    # distancia pista lado esquerdo
    if math.isinf(media_esquerda)==False:
        ponto_superior_esquerdo = (FRAME_CENTER, AREA_INT_INICIO)
        ponto_inferior_direito = (FRAME_CENTER - round(media_esquerda), AREA_INT_FIM)
        cv.rectangle(frame, ponto_superior_esquerdo, ponto_inferior_direito, (0, 0, 255), -1)

        # Plota o texto da curva media_esquerda:
        text = str(media_esquerda)
        posicao = (int((FRAME_CENTER - 60)), AREA_INT_INICIO-10)
        cv.putText(frame, text, posicao, font, fontScale, fontCor, fontThickness, cv.LINE_AA, False)

    # linha de centro
    cv.line(frame,
            (int(FRAME_WIDTH / 2), 0),
            (int(FRAME_WIDTH / 2), FRAME_HEIGHT),
            (0, 255, 255), 2)

    # saidas das imagens processadas
    cv.imshow("Saida de Videos", frame)
    cv.imshow("Saida Filtro", imgFiltro)
    cv.imshow("Area de Intersse", area_interesse)

    # Captura o valor atual da trackbar 'Lado'
    Lado = cv.getTrackbarPos('Lado', 'Controles')

    pare = cv.getTrackbarPos('pare', 'Controles')

    # Define qual faixa deve ser analisada com base no valor da trackbar


    #----------------------------------------------------------------------------------------------------------------------

    enviar_dados[0] = cv.getTrackbarPos('velocidade', 'Controles')   # VELOCIDADE
    enviar_dados[1] = 150                                                               # FAROL_FRONTAL

    if pare == 0:
        # O CARRO VAI CONTINUAR NORMAL
        enviar_dados[4] = 0
    else:
        # O CARRO VAI PARAR
        enviar_dados[4] = 1

    if Lado == 0:
        enviar_dados[0] = cv.getTrackbarPos('velocidade', 'Controles')  # VELOCIDADE
        # Controlar o veiculo pela lado direito
        if not math.isinf(media_direita):
            enviar_dados[2] = round(pid_direcao.calcularSaida(media_direita))
            enviar_dados[2] = (enviar_dados[2] * -1) + 85
    elif Lado == 1:
        enviar_dados[0] = cv.getTrackbarPos('velocidade', 'Controles')  # VELOCIDADE
        # Controlar o veiculo pela lado esquerdo
        if not math.isinf(media_esquerda):
            enviar_dados[3] = round(pid_direcao.calcularSaida(media_esquerda))
            enviar_dados[3] = enviar_dados[3] + 85
    elif Lado == 2:
        enviar_dados[0] = cv.getTrackbarPos('velocidade', 'Controles')  # VELOCIDADE
        if not math.isinf(media_direita):
            enviar_dados[2] = round(pid_direcao.calcularSaida(media_direita))
            enviar_dados[2] = (enviar_dados[2] * -1) + 85
        if not math.isinf(media_esquerda):
            enviar_dados[3] = round(pid_direcao.calcularSaida(media_esquerda))
            enviar_dados[3] = enviar_dados[3] + 85
    else:
        enviar_dados[0] = cv.getTrackbarPos('velocidade', 'Controles')  # VELOCIDADE
        if not math.isinf(media_direita):
            enviar_dados[2] = 90
        if not math.isinf(media_esquerda):
            enviar_dados[3] = 90


    #enviar_dados[6] = faixaDireita  #valor da faixa da direita
    #enviar_dados[7] = faixaEsquerda #valor da faixa da esquerda
    enviar_dados[8] = ANALISAR_FAIXA




# ----------------------------------------------------------------------------------------------------------------------

    # chama a funcao para enviar tudo por serial para o arduino
    enviar_dados_serial()
    # verificar tecla para fechar o programa
    if cv2.waitKey(1) == 27:  # 27 é o código ASCII para a tecla ESC
        break



# fechar programa jumto com as conexoes
fecharAplicativo()
