from simple_pid import PID
import time

# Configura o controlador PID com os ganhos Kp, Ki, Kd e o setpoint que será o valor "0" (erro zero)
pid = PID(0.0, 0.0, 0.0, setpoint=0)  # Setpoint é zero, pois queremos que o erro seja zero

# Limita o valor de saída do PID para corresponder ao ângulo do servo
pid.output_limits = (20, 120)  # Limita o valor do controle do servo


# Loop de controle contínuo
while True:
    # Obtém o erro da câmera (centro da rua - centro do carro)
    error = VALOR DO ERRO

    # Calcula o valor de controle com base no erro
    control = pid(error)

    # Envia o valor calculado ao servo para corrigir o ângulo do carro
    print("o erro é: ", error )
    print("Ajuste o servo: ", control)

    # Espera um tempo antes de fazer a próxima leitura (para evitar saturação)
    time.sleep(0.5)
