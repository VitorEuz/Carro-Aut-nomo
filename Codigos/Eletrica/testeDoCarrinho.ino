// ainda falta colocar mais alguns detalhes sobre a ponte H mas forma geral é isso

#include <Servo.h>

Servo valorServo;

int motorDire = 10; // direção do motor DC
int motorVelo = 11; // velocidade do motor DC

void setup() {
  pinMode(motorDire, OUTPUT);
  pinMode(motorVelo, OUTPUT); 

  valorServo.attach(9);
  valorServo.write(90);
}

void loop() {
  int comando = comandoSerial();

  switch (comando) {
    case 1:
      frente();
      delay(10000);
      parar();
      break;
    case 2:
      voltar();
      delay(10000);
      parar();
      break;
    case 3:
      frente();
      valorServo.write(0);
      delay(10000);
      parar();
      break;
    case 4:
      frente();
      valorServo.write(180);
      delay(10000);
      parar();
      break;
    default:
      parar();
      break;
  }
}

int comandoSerial() {
  if (Serial.available() > 0) {
    return Serial.parseInt();
  }
  return 0;
}


void frente() {
  digitalWrite(motorDire, HIGH);
  analogWrite(motorVelo, 255);
}

void voltar() {
  digitalWrite(motorDire, LOW);
  analogWrite(motorVelo, 255);
}

void parar() {
  digitalWrite(motorDire, LOW);
  analogWrite(motorVelo, 0);
}
