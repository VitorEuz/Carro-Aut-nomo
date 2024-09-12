/*
Array[0] = SERVO
Array[1] = Velocidade


*/



#include <Servo.h>

Servo servo_direcao;

// MOTOR 1: Pinos responsáveis pelo controle do motor 1
const byte MOTOR_1A = 18;
const byte MOTOR_1B = 19;
// MOTOR 2: Pinos responsáveis pelo controle do motor 2
const byte MOTOR_2A = 16;
const byte MOTOR_2B = 17;


const byte LED_PIN = 13;       // Pino do LED integrado
const byte SERVO_PIN = 12; // Pino para o controle do servo motor

void setup()
{
  // Configura o pino do LED como saída
  pinMode(LED_PIN, OUTPUT);

  // Configura o servo no pino especificado
  servo_direcao.attach(SERVO_PIN);

  // Configura os pinos dos motores como saídas
  pinMode(MOTOR_1A, OUTPUT);
  pinMode(MOTOR_1B, OUTPUT);
  pinMode(MOTOR_2A, OUTPUT);
  pinMode(MOTOR_2B, OUTPUT);

  // Inicializa os motores desligados (sem tensão)
  analogWrite(MOTOR_1A, LOW);
  analogWrite(MOTOR_1B, LOW);
  analogWrite(MOTOR_2A, LOW);
  analogWrite(MOTOR_2B, LOW);

  // Inicia a comunicação serial para receber comandos
  Serial.begin(9600);
  delay(500); // Pequeno atraso para estabilização
}

void loop() {
  // Verifica se há dados disponíveis na comunicação serial
  if (Serial.available() > 0) {
    // Lê a string até encontrar o caractere '#'
    String input = Serial.readStringUntil('#');
    input.trim(); // Remove espaços em branco no início e no fim da string

    // Verifica se a string termina com uma vírgula, e a remove se for o caso
    if (input.endsWith(",")) {
      input = input.substring(0, input.length() - 1);

      // Cria um array de strings para armazenar os valores dos ângulos
      String enviarArray[9]; // 8 posições para valores enviados
      int enviarIndex = 0;    // Índice do array
      int startIndex = 0;    // Ponto de início para a extração de substrings

      // Faz o parse da string de entrada e separa os valores dos dados enviados
      for (int i = 0; i < input.length(); i++) {
        // Quando encontra uma vírgula, extrai o valor
        if (input.charAt(i) == ',') {
          enviarArray[enviarIndex] = input.substring(startIndex, i);
          startIndex = i + 1; // Atualiza o índice de início
          enviarIndex++;       // Incrementa o índice do array
        }
      }
      // Armazena o último valor após a última vírgula
      enviarArray[enviarIndex] = input.substring(startIndex);

      // CONTROLE DO SERVO DA DIREÇÃO: utiliza o primeiro valor (posição 0 do array)
      servo_direcao.write(enviarArray[0].toInt());

      // CONTROLE DE VELOCIDADE PARA FRENTE: utiliza o segundo valor (posição 1 do array)
      int speed = enviarArray[1].toInt();

      // Aciona os motores para movimentação à frente com a velocidade especificada
      analogWrite(MOTOR_1A, LOW);
      analogWrite(MOTOR_1B, speed);
      analogWrite(MOTOR_2A, LOW);
      analogWrite(MOTOR_2B, speed);

      // Alterna o estado do LED, quando uma informação chegar!
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
}
