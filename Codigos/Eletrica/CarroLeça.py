# Bibliotecas
import cv2
import cv2 as cv
import numpy as np
import math
import serial
import time
from ultralytics import YOLO


DIREITA  = 0
ESQUERDA = 1
AMBOS = 2
RETO = 3

SetPoint_DIREITA = 170
SetPoint_ESQUERDA = 170

#Valor da Webcam
CameraFaixa = 0
CameraPlaca = 0

# constantes de envio de informaçoes por serial
ENVIAR_DADOS = False    # enviar dados para o arduino
TEMPO_ENVIO  = 0.1      # tempo em segundos
PORTA_COM    = 'COM8'  # porta COM para comunicar com Arduino
porta_serial = None

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
    cap2.release()
    cv2.destroyAllWindows()
    quit()

# Função para calcular a distância
def calculate_distance(known_width, focal_length, pixel_width):
    return (known_width * focal_length) / pixel_width

# Função para verificar a cor dominante
def get_dominant_color(image, region):
    # Converter a região para o espaço de cores HSV
    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

    # Definir faixas de cor para vermelho e verde no espaço HSV
    red_lower1 = np.array([0, 100, 100])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([160, 100, 100])
    red_upper2 = np.array([180, 255, 255])

    green_lower = np.array([40, 50, 50])
    green_upper = np.array([80, 255, 255])

    # Criar máscaras para as cores
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    # Contar os pixels vermelhos e verdes
    red_count = cv2.countNonZero(red_mask)
    green_count = cv2.countNonZero(green_mask)

    # Determinar a cor dominante
    if red_count > green_count:
        return "Red"
    elif green_count > red_count:
        return "Green"
    else:
        return "Unknown"

# Função para verificar se um ponto está abaixo da linha
def is_below_line(x1, y1, x2, y2, px, py):
    # Fórmula para verificar se o ponto (px, py) está abaixo da linha definida por (x1, y1) e (x2, y2)
    return (py - y1) * (x2 - x1) > (y2 - y1) * (px - x1)

if ENVIAR_DADOS == True:
    # Estabelecer conexao com o Arduino
    porta_serial = serial.Serial(PORTA_COM, 9600)
    time.sleep(1)

# Definir parâmetros conhecidos
KNOWN_WIDTHS = {"person": 0.5, "stop sign": 0.75, "traffic light": 0.4}  # Largura média de pessoas e objetos em metros
FOCAL_LENGTH = 600  # Valor hipotético, precisa ser ajustado dependendo da câmera

# Carregar o modelo YOLOv8 personalizado (.pt) com o logging desativado
model = YOLO("C:/Users/limao/Área de Trabalho/Arquivos dos aplicativos/PYTHON/yolov8n.pt")  # Substitua pelo caminho do seu arquivo .pt

# Desativar os logs do YOLOv8
model.overrides['verbose'] = False  # Desativa mensagens do modelo

# Cria uma janela onde os controles serão exibidos
cv.namedWindow('Controles')

# Cria as trackbars (sliders) para controle de Hue, Saturation e Value
cv.createTrackbar('canny_1', 'Controles', 200, 300, nothing)
cv.createTrackbar('canny_2', 'Controles', 220, 400, nothing)
cv.createTrackbar('velocidade', 'Controles', 78, 255, nothing)
cv.createTrackbar('Lado','Controles',2,3,nothing)

# Inicializa o controlador PID com ganhos KP, KI, KD
pid_direcao_DIREITA = pid_direcao(SetPoint_DIREITA, KP, KI, KD, MIN, MAX)
pid_direcao_ESQUERDA = pid_direcao(SetPoint_ESQUERDA, KP, KI, KD, MIN, MAX)

# config. de saida de texto -> Plota na tela o texto
font = cv.FONT_HERSHEY_SIMPLEX
fontScale = 0.5
fontCor = (0, 0, 255)  # BGR
fontThickness = 1

# Definir os pontos da linha vermelha (pode ajustar conforme necessário)
line_start = (100, 450)  # Ponto inicial da linha
line_end = (400, 450)    # Ponto final da linha

cap = cv.VideoCapture(CameraFaixa, cv2.CAP_DSHOW)
cap2 = cv2.VideoCapture(CameraPlaca, cv2.CAP_DSHOW)

while(1):
    # pegar frame do video / camera
    ret, frame = cap.read()
    ret2, frame2 = cap2.read()

    # se nao tiver imagem fecha a aplicacao
    if not ret and ret2:
        print("entrada cap nao detectado!")
        # fechar programa jumto com as conexoes
        fecharAplicativo()

# ------------------------------CODIGO 2---------------------------------------------------------------------
    # Desenhar a linha vermelha na imagem
    cv2.line(frame2, line_start, line_end, (0, 0, 255), 2)

    # Realizar a detecção usando o modelo YOLOv8
    results = model(frame2)

    ParaPlaca = 0
    ParaPessoa = 0
    ParaSemaforo = 0

    for result in results:
        boxes = result.boxes
        for box in boxes:
            class_id = int(box.cls)
            label = model.names[class_id]
            if label in KNOWN_WIDTHS:  # Verifica se a classe é uma das desejadas
                # Obter coordenadas da caixa delimitadora
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w = x2 - x1
                h = y2 - y1

                # Calcular a distância
                object_width = KNOWN_WIDTHS[label]
                distance = calculate_distance(object_width, FOCAL_LENGTH, w)

                # Desenhar caixa e a distância
                color = (0, 255, 0)
                cv2.rectangle(frame2, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame2, f"{label}: {distance:.2f}m", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # Verificar se é um semáforo e determinar a cor da luz
                if label == "traffic light":
                    region_upper = frame2[y1:int(y1 + h // 3), x1:x2]  # Parte superior (vermelho)
                    region_lower = frame2[int(y1 + 2 * h // 3):y2, x1:x2]  # Parte inferior (verde)
                    upper_color = get_dominant_color(frame2, region_upper)
                    lower_color = get_dominant_color(frame2, region_lower)
                    if upper_color == "Red":
                        light_status = "Red light"
                    elif lower_color == "Green":
                        light_status = "Green light"
                    else:
                        light_status = "Unknown light"
                    cv2.putText(frame2, light_status, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    if light_status == "Red light" and distance < 2:
                        ParaSemaforo = 1

                # Verificar se uma pessoa está abaixo da linha e a menos de 2 metros
                if label == "person" and distance < 2:
                    person_center_x = (x1 + x2) // 2
                    person_center_y = y2  # Considerando a parte inferior da pessoa
                    if is_below_line(*line_start, *line_end, person_center_x, person_center_y):
                        ParaPessoa = 1

                # Verificar se uma placa de pare está a menos de 2 metros
                if label == "stop sign" and distance < 3.8:
                    ParaPlaca = 1

# ------------------------------CODIGO 1--------------------------------------------------------------------

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
    cv.imshow("Saida Faixa", frame)
    cv2.imshow("Saida Pessoa", frame2)
    cv.imshow("Saida Filtro", imgFiltro)
    cv.imshow("Area de Intersse", area_interesse)

    # Captura o valor atual da trackbar 'Lado'
    Lado = cv.getTrackbarPos('Lado', 'Controles')

#----------------------------------------------------------------------------------------------------------------------

    enviar_dados[0] = cv.getTrackbarPos('velocidade', 'Controles')   # VELOCIDADE
    enviar_dados[1] = 150                                                               # FAROL_FRONTAL

    if Lado == 0:
        enviar_dados[0] = cv.getTrackbarPos('velocidade', 'Controles')  # VELOCIDADE
        # Controlar o veículo pelo lado direito
        if not math.isinf(media_direita):
            enviar_dados[2] = round(pid_direcao_DIREITA.calcularSaida(media_direita))
            enviar_dados[2] = (enviar_dados[2] * -1) + 85
    elif Lado == 1:
        enviar_dados[0] = cv.getTrackbarPos('velocidade', 'Controles')  # VELOCIDADE
        # Controlar o veiculo pela lado esquerdo
        if not math.isinf(media_esquerda):
            enviar_dados[3] = round(pid_direcao_ESQUERDA.calcularSaida(media_esquerda))
            enviar_dados[3] = enviar_dados[3] + 85
    elif Lado == 2:
        enviar_dados[0] = cv.getTrackbarPos('velocidade', 'Controles')  # VELOCIDADE
        if not math.isinf(media_direita):
            enviar_dados[2] = round(pid_direcao_DIREITA.calcularSaida(media_direita))
            enviar_dados[2] = (enviar_dados[2] * -1) + 85
        if not math.isinf(media_esquerda):
            enviar_dados[3] = round(pid_direcao_ESQUERDA.calcularSaida(media_esquerda))
            enviar_dados[3] = enviar_dados[3] + 85
    else:
        enviar_dados[0] = cv.getTrackbarPos('velocidade', 'Controles')  # VELOCIDADE
        if not math.isinf(media_direita):
            enviar_dados[2] = 90
        if not math.isinf(media_esquerda):
            enviar_dados[3] = 90

    enviar_dados[5] = ParaPlaca
    enviar_dados[6] = ParaSemaforo
    enviar_dados[7] = ParaPessoa


# ----------------------------------------------------------------------------------------------------------------------

    # chama a funcao para enviar tudo por serial para o arduino
    enviar_dados_serial()
    if cv2.waitKey(1) == 27:  # 27 é o código ASCII para a tecla ESC
        break

# fechar programa jumto com as conexoes
fecharAplicativo()
