import cv2
import numpy as np
import time
import serial

# Configurações do Arduino
COM_ARDUINO = True
ARDUINO_PORT = 'COM3'
TEMPO_ENVIO = 0.4  # Tempo de envio em segundos

def conectar_arduino(porta, baud_rate=9600):
    """Conecta ao Arduino na porta especificada e retorna o objeto Serial."""
    try:
        porta_serial = serial.Serial(porta, baud_rate)
        time.sleep(1)  # Aguarda a comunicação serial estabilizar
        return porta_serial
    except serial.SerialException as e:
        print(f"Erro ao conectar ao Arduino: {e}")
        return None

def enviar_dados_serial(porta_serial, dados):
    """Envia dados pela porta serial para o Arduino."""
    if porta_serial is not None:
        porta_serial.write(dados.encode())
    else:
        print("Porta serial não está conectada.")

def nothing(x):
    pass

def detectar_faixas_por_contornos(frame, canny_min, canny_max, thresh_low, thresh_high):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, thresh_low, thresh_high, cv2.THRESH_BINARY)
    blurred = cv2.GaussianBlur(thresh, (5, 5), 0)
    edges = cv2.Canny(blurred, canny_min, canny_max)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours, edges

def calcular_distancia_horizontal(roi, ponto_central, contours):
    altura, largura = roi.shape[:2]
    ponto_vermelho = (int(largura / 2), int(altura / 2))
    menor_distancia = float('inf')
    ponto_mais_proximo = None

    if contours is not None:
        for contour in contours:
            for i in range(len(contour) - 1):
                x1, y1 = contour[i][0]
                x2, y2 = contour[i + 1][0]

                if x1 > ponto_central[0] and x2 > ponto_central[0]:
                    distancia = min(abs(x1 - ponto_central[0]), abs(x2 - ponto_central[0]))
                    if distancia < menor_distancia:
                        menor_distancia = distancia
                        ponto_mais_proximo = (int((x1 + x2) / 2), int((y1 + y2) / 2))

    return menor_distancia if menor_distancia != float('inf') else 0, ponto_mais_proximo

def definir_roi(frame, roi_height):
    altura, largura = frame.shape[:2]
    roi = frame[int(altura * (1 - roi_height)):altura, :]
    return roi

# Configuração da câmera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Configuração da porta serial
porta_serial = conectar_arduino(ARDUINO_PORT) if COM_ARDUINO else None

cv2.namedWindow('Filtros')
cv2.createTrackbar('Canny Low', 'Filtros', 50, 255, nothing)
cv2.createTrackbar('Canny High', 'Filtros', 150, 255, nothing)
cv2.createTrackbar('Threshold Low', 'Filtros', 147, 255, nothing)
cv2.createTrackbar('Threshold High', 'Filtros', 255, 255, nothing)
cv2.createTrackbar('ROI Height', 'Filtros', 3, 10, nothing)

distancia_desejada = 60  # Distância de pixels entre o ponto vermelho e a faixa escolhida
setpointMin = 55
setpointMax = 65
dados = [0] * 9  # Inicializa o array dados com zeros



frame_count = 0
distancias = []
tempo_inicio = time.time()
tempo_anterior = time.time()

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("Erro ao capturar o frame")
        break

    canny_min = cv2.getTrackbarPos('Canny Low', 'Filtros')
    canny_max = cv2.getTrackbarPos('Canny High', 'Filtros')
    thresh_low = cv2.getTrackbarPos('Threshold Low', 'Filtros')
    thresh_high = cv2.getTrackbarPos('Threshold High', 'Filtros')
    roi_height = cv2.getTrackbarPos('ROI Height', 'Filtros') / 10.0

    roi = definir_roi(frame, roi_height)
    contours, edges = detectar_faixas_por_contornos(roi, canny_min, canny_max, thresh_low, thresh_high)

    if contours is not None:
        cv2.drawContours(roi, contours, -1, (0, 255, 0), 2)
        ponto_central = (roi.shape[1] // 2, roi.shape[0] // 2)
        distancia, ponto_mais_proximo = calcular_distancia_horizontal(roi, ponto_central, contours)
        distancias.append(distancia)
        frame_count += 1

        if frame_count == 5:
            frame_count = 0
            if time.time() - tempo_inicio >= 1.5:
                media_distancia = np.mean(distancias)
                diferenca_distancia = media_distancia - distancia_desejada

                distancia_img = np.zeros_like(roi)
                cv2.putText(distancia_img, f'Distancia Media: {int(media_distancia)} pixels', (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow("Distancia", distancia_img)

                diferenca_img = np.zeros_like(roi)
                cv2.putText(diferenca_img, f'Diferenca Media: {int(diferenca_distancia)} pixels', (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                if int(media_distancia) > setpointMax:
                    print("Vai para 0")
                    angle = int(50)
                elif int(media_distancia) < setpointMin:
                    print("Vai para 180")
                    angle = int(130)
                elif int(media_distancia) > setpointMin and int(media_distancia) < setpointMax:
                    print("PARA")
                    angle = int(0)


                #Eviar dados
                dados[0] = angle
                dados[1] = int(diferenca_distancia)
                dados[2] = int(media_distancia)
                dados[3] = 0
                dados[4] = 0
                dados[5] = 0
                dados[6] = 0
                dados[7] = 0
                dados[8] = 0

                cv2.imshow("Diferenca de Distancia", diferenca_img)

                tempo_inicio = time.time()
                distancias.clear()

        cv2.circle(roi, ponto_central, 5, (0, 0, 255), -1)
        if ponto_mais_proximo is not None:
            cv2.line(roi, ponto_central, (ponto_mais_proximo[0], ponto_central[1]), (255, 0, 0), 2)
            cv2.circle(roi, ponto_mais_proximo, 5, (255, 0, 0), -1)

    cv2.imshow("Frame", frame)
    cv2.imshow("ROI", roi)
    cv2.imshow("Edges", edges)

    if (time.time() - tempo_anterior) >= TEMPO_ENVIO:
        enviar_dados = ""
        for i in range(len(dados)):
            enviar_dados += str(dados[i]) + ","
        enviar_dados += "#"
        print(enviar_dados)
        enviar_dados_serial(porta_serial, enviar_dados)
        tempo_anterior = time.time()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if COM_ARDUINO and porta_serial:
    porta_serial.close()
