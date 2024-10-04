// Definindo os pinos dos botões e LEDs
const int botaoDireita = 7;  // Pino onde o botão da direita está conectado
const int botaoEsquerda = 8;  // Pino onde o botão da esquerda está conectado
const int ledDireita = 5;    // Pino onde o LED da direita está conectado
const int ledEsquerda = 6;    // Pino onde o LED da esquerda está conectado

int lado;

void setup() {
  // Configurando os pinos dos botões como entrada
  pinMode(botaoDireita, INPUT_PULLUP); // Usa resistor interno
  pinMode(botaoEsquerda, INPUT_PULLUP); // Usa resistor interno

  // Configurando os pinos dos LEDs como saída
  pinMode(ledDireita, OUTPUT);
  pinMode(ledEsquerda, OUTPUT);

  // Inicializa os LEDs apagados
  digitalWrite(ledDireita, LOW);
  digitalWrite(ledEsquerda, LOW);
  Serial.begin(9600);  // Inicia a comunicação serial
  delay(500); 
}

void loop() {
  int estadoBotaoDireita = digitalRead(botaoDireita);
  int estadoBotaoEsquerda = digitalRead(botaoEsquerda);
  // Se o botão da direita for pressionado
  if (estadoBotaoDireita == LOW) {
    digitalWrite(ledDireita, HIGH); // Acende o LED da direita
    digitalWrite(ledEsquerda, LOW); // Apaga o LED da esquerda
    lado = 0;
  }
  // Se o botão da esquerda for pressionado
  if (estadoBotaoEsquerda == LOW) {
    digitalWrite(ledEsquerda, HIGH); // Acende o LED da esquerda
    digitalWrite(ledDireita, LOW); // Apaga o LED da direita
    lado = 1;
  }

    // Aqui você pode definir valores adicionais
    int valor_1 = lado;  // Lê o valor do LDR
    int valor_2 = 0;  // Substitua por alguma lógica se necessário
    int valor_3 = 0;  // Substitua por alguma lógica se necessário
    int valor_4 = 0;  // Substitua por alguma lógica se necessário
    int valor_5 = 0;  // Substitua por alguma lógica se necessário


    // Monta a string para enviar ao Python
    String enviarDados = String(valor_1) + "," + String(valor_2) + "," + String(valor_3) + "," + String(valor_4) + "," + String(valor_5) +  "#";
    // Envia os dados
    Serial.println(enviarDados); // Envia a string para o Python
    delay(500); // Espera 200 milissegundos antes de enviar novamente
}
