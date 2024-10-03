const int LDR_PIN = A0;  // Pino analógico onde o LDR está conectado

void setup() {
    Serial.begin(9600);  // Inicia a comunicação serial
    delay(500); 
}

void loop() {
    int valor_1 = 874;  // Lê o valor do LDR

    // Aqui você pode definir valores adicionais
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
