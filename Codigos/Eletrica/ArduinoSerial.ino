/*
PonteH = 5 PWM
Servo = 3 PWM
FarolFrontal = 6 PWM

LedIndicadorDireito = 7
LedIndicadorEsquerdo = 4

BotãoDireto = A5
BotãoEsquerdo = A4

Array[0] = MOTOR
Array[1] = Farol Frontal
Array[2] = SERVO DIREITA
Array[3] = SERVO ESQUERDO
*/

#include <Servo.h>

Servo servo_direcao;

const byte MOTOR_1R = 5;
const byte FaroisFrontal = 13;        
const byte SERVO_PIN = 3;       
const byte FarolFrontal = 6;

const int botaoDireito = A5;
const int botaoEsquerdo = A4; 
const byte LedIndicadorDireito = 7;
const byte LedIndicadorEsquerdo = 4;

bool botaoSelecionado = false;

void setup() {
  pinMode(FaroisFrontal, OUTPUT);
  pinMode(FarolFrontal, OUTPUT);
  pinMode(MOTOR_1R, OUTPUT);
  pinMode(LedIndicadorDireito, OUTPUT);
  pinMode(LedIndicadorEsquerdo, OUTPUT);

  pinMode(botaoDireito, INPUT_PULLUP);  
  pinMode(botaoEsquerdo, INPUT_PULLUP);

  servo_direcao.attach(SERVO_PIN);
  servo_direcao.write(90);

  digitalWrite(LedIndicadorDireito, HIGH); 
  digitalWrite(LedIndicadorEsquerdo, HIGH);

  Serial.begin(9600);
  delay(500);
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



      // CONTROLE DO SERVO DA DIREÇÃO: utiliza o primeiro valor (posição 0 do array)
      servo_direcao.write(enviarArray[2].toInt());

      // CONTROLE DE VELOCIDADE PARA FRENTE: utiliza o segundo valor (posição 1 do array)
      int speed = enviarArray[0].toInt();
      analogWrite(MOTOR_1R, speed);

      //Acende os leds frontais
      int ligadoF = enviarArray[1].toInt();
      analogWrite(FarolFrontal, ligadoF);

      // CONTROLE DO SERVO DA DIREÇÃO: utiliza o primeiro valor (posição 0 do array)
      servo_direcao.write(enviarArray[2].toInt());

      // CONTROLE DO SERVO DA DIREÇÃO: utiliza o primeiro valor (posição 0 do array)
      //servo_direcao.write(enviarArray[3].toInt());


    }
  }
}
