

/*
PonteH = 5 PWM
Servo = 3 PWM
FarolFrontal = 6 PWM

LedIndicadorDireito = 7
LedIndicadorEsquerdo = 4
LedTrasieors = 8

BotãoDireto = A5
BotãoEsquerdo = A4

ultra1_tri = 9
ultra1_ech = 10

ultra2_tri = 11
ultra2_ech = 12

Array[0] = MOTOR
Array[1] = Farol Frontal
Array[2] = SERVO DIREITA
Array[3] = SERVO ESQUERDO
*/

#include <Servo.h>
#include <Ultrasonic.h>


Servo servo_direcao;

const byte MOTOR_1R = 5;
const byte FaroisFrontal = 13;        
const byte SERVO_PIN = 3;       
const byte FarolFrontal = 6;

const int botaoDireito = A5;
const int botaoEsquerdo = A4; 
const byte LedIndicadorDireito = 7;
const byte LedIndicadorEsquerdo = 4;
const byte LedTrasieors = 8;

bool estadoDireito = false;
bool estadoEsquerdo = false;

const int ultra1_tri = 9;
const int ultra1_ech = 10;
Ultrasonic sensor1(ultra1_tri,ultra1_ech);
const int ultra2_tri = 11;
const int ultra2_ech = 12;
Ultrasonic sensor2(ultra2_tri,ultra2_ech);


int ladocerto = 5;

unsigned long previousMillis = 0;
unsigned long interval = 0;
bool motorState = LOW; 


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

  analogWrite(LedTrasieors, 0);
  delay(500);
  analogWrite(LedTrasieors, 150);
  delay(500);
  analogWrite(LedTrasieors, 0);

  Serial.begin(9600);
  delay(500);
}

void loop() {
  unsigned long currentMillis = millis();

  if (digitalRead(botaoDireito) == LOW) {
    if (!estadoDireito) {
      digitalWrite(LedIndicadorDireito, HIGH);
      digitalWrite(LedIndicadorEsquerdo, LOW);
      ladocerto = 0;
      estadoDireito = true;
      estadoEsquerdo = false;
    }
  } else if (digitalRead(botaoEsquerdo) == LOW) {
    if (!estadoEsquerdo) {
      digitalWrite(LedIndicadorDireito, LOW);
      digitalWrite(LedIndicadorEsquerdo, HIGH);
      ladocerto = 1;
      estadoEsquerdo = true;
      estadoDireito = false;
    }
  }
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
      int speed = enviarArray[0].toInt();
      int Placa = enviarArray[5].toInt();
      int Semaforo = enviarArray[6].toInt();
      int Pessoa = enviarArray[7].toInt();
      
      // Verifica se Pessoa ou Semaforo estão iguais a 1
      if (Pessoa == 1 || Semaforo == 1) {
        // Se qualquer um dos dois for igual a 1, o motor para imediatamente
        analogWrite(MOTOR_1R, 0);  // Para o motor
        analogWrite(LedTrasieors, 150);
        motorState = LOW;          // Define o estado como desligado
      }
      // Caso contrário, se Placa for 1, usa a lógica de millis para controle de tempo
      else if (Placa == 1) {
        if (motorState == LOW && (currentMillis - previousMillis >= interval)) {
          // Para o motor por 2 segundos
          analogWrite(MOTOR_1R, 0);      // Desliga o motor (0 = parado)
          analogWrite(LedTrasieors, 150);
          motorState = HIGH;             // Define o estado do motor como parado
          interval = 2000;               // Define o intervalo de 2 segundos
          previousMillis = currentMillis; // Atualiza o tempo de referência
        }
        else if (motorState == HIGH && (currentMillis - previousMillis >= interval)) {
          // Após 2 segundos, liga o motor por 5 segundos
          analogWrite(MOTOR_1R, speed); // Liga o motor na velocidade máxima (255 = 100%)
          analogWrite(LedTrasieors, 0);
          interval = 5000;                   // Define o intervalo de 5 segundos
          previousMillis = currentMillis;    // Atualiza o tempo de referência
          motorState = LOW;                  // Define o motor como ligado
        }
      }
      else if (Placa == 0 && Pessoa == 0 && Semaforo == 0) {
        // Se todos os valores forem 0, liga o motor normalmente
        analogWrite(MOTOR_1R, speed); // Liga o motor normalmente
        analogWrite(LedTrasieors, 0);
        motorState = LOW;  // Define o motor como ligado
        interval = 0;      // Reseta o intervalo para reiniciar o ciclo
      }
      
      //Acende os leds frontais
      int ligadoF = enviarArray[1].toInt();
      analogWrite(FarolFrontal, ligadoF);

      if (ladocerto == 0){
       servo_direcao.write(enviarArray[2].toInt()); 
      }else {
        servo_direcao.write(enviarArray[3].toInt());}


    } 
  }
}
