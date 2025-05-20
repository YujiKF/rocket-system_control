// === Servo ===
#include <Servo.h>
Servo valveServo;     // Cria um objeto chamado "valveServo" da classe Servo.
                      // Esse objeto será usado para controlar a posição da válvula (via servo motor).
                      // Ele encapsula todos os detalhes do sinal PWM necessário para posicionar o servo.


// === Bluetooth ===
#include <BluetoothSerial.h>     // Biblioteca oficial para Bluetooth SPP no ESP32
BluetoothSerial SerialBT;        // Instancia a interface Serial Bluetooth

// === Pinos ===
#define PIN_PRESSURE 34   // Entrada analógica do sensor (ADC1_6)
#define PIN_SERVO 18      // Saída PWM para o servo


// === PID ===
float kp = 5.0;
float ki = 1.0;
float kd = 0.2;

float setpoint = 5.0; // Pressão alvo [bar]
float integrative = 0.0; // Integral, utilizada junto com o nosso Ki
float previous_error = 0.0; // Erro anterior, serve para calcular a derivativa
unsigned long lastTime = 0; // Utilizada para calcular dt também

// === Setup ===
void setup() {
  Serial.begin(115200);             // Estabele baudrate de 115200 bps para o console serial (para computador, via USB) onde será exibido nossas informações
  SerialBT.begin("ESP32_Rocket");   // Nome Bluetooth visível no celular
  valveServo.attach(PIN_SERVO);     // Conecta o objeto "valveServo" ao pino digital especificado (PIN_SERVO).
  pinMode(PIN_PRESSURE, INPUT);     // Declara o pino do transdutor como entrada analógica
}

float readPressureBar() {
  int adc_raw = analogRead(PIN_PRESSURE);           // Leitura direta do ADC (0-4095, já que são 12 bits)
  float voltage = adc_raw * (3.3 / 4095.0);         // Tensão lida no ADC (em V), convertendo ADC (0-4095) para tensão (0-3.3V)
  float voltage_original = voltage * (5.0 / 3.3);   // Revertendo o divisor de tensão (Estamos usando dividor porque a tensão máxima na ESP é 3.3V e sai 5V na saída do sensor.)

  float pressure_mpa = (voltage_original / 5.0) * 1.2;  // Sensor: 0–5V → 0–1.2 MPa (Supondo que ele lê de forma linear)
  float pressure_bar = pressure_mpa * 10.0;             // Convertendo pressão de Pascal para Bar, 1 MPa = 10 bar

  return pressure_bar;
}


float computePID(float pressure, float dt) {
  float error = setpoint - pressure;                // Calcula o erro entre a pressão desejada (setpoint) e a medida atual (feedback do sensor)
  integrative += error * dt;                        // Parte integral: soma o erro ao longo do tempo (acumula erro). Essa parte ajuda o sistema a eliminar erros constantes e alcançar o setpoint com mais precisão.
  float derivative = (error - previous_error) / dt; // Parte derivativa: calcula a velocidade de mudança do erro (quanto ele está variando). Ajuda a suavizar a resposta, reduzindo oscilações.
  previous_error = error;                           // Armazena o erro atual para usar no próximo ciclo (cálculo da derivada).

  float output = kp * error + ki * integrative + kd * derivative;   // Equação PID: combina as três componentes ponderadas por seus ganhos respectivos.
  return constrain(output, 0.0, 100.0);                             // Garante que o valor final da saída esteja entre 0% e 100% (representando a abertura da válvula).
}

void setValveOpening(float percent) {
  // Garante que a porcentagem esteja no intervalo de 0 a 100%
  percent = constrain(percent, 0.0, 100.0);

  // Mapeia a porcentagem de abertura para o intervalo do servo: 0° a 90°
  // Isso porque a válvula esfera abre completamente com apenas 90° de rotação
  int angle = map(percent, 0, 100, 0, 90);

  // Move o servo motor para o ângulo calculado
  valveServo.write(angle);
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;                     // Calcula o tempo decorrido desde a última iteração (em segundos).
  if (dt < 0.01) return;                                    // Evita executar o controle se ainda não passou tempo suficiente (amostragem mínima de 10ms).
  lastTime = now;                                           // Atualiza o marcador de tempo da última iteração.

  float pressure = readPressureBar();                       // Lê a pressão atual do sensor, já convertida para unidade de bar.
  float control = computePID(pressure, dt);                 // Calcula a saída do PID com base na pressão atual e no tempo decorrido.
  setValveOpening(control);                                 // Envia o valor de controle para a válvula (servo), ajustando a abertura.

  // Debug serial (para computador, via USB)
  Serial.print("Pressão: ");
  Serial.print(pressure, 2);
  Serial.print(" bar | Válvula: ");
  Serial.print(control, 1);
  Serial.println(" %");
  // Envia os mesmos dados via Bluetooth
  SerialBT.print("Pressão: ");
  SerialBT.print(pressure, 2);
  SerialBT.print(" bar | Válvula: ");
  SerialBT.print(control, 1);
  SerialBT.println(" %");

  delay(50);                                             // Amostragem de ~20 Hz

  if (pressure > 10.0) {                                 // Se a pressão passar de 10 bar, fecha completamente a válvula por segurança.
  setValveOpening(0);                                    // Fecha a válvula
  Serial.println("⚠️ Pressão acima do limite!");
  while (true);                                          // Trava o sistema


  // Leitura opcional de novo setpoint vindo do app Bluetooth
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    float novoSetpoint = cmd.toFloat();
    if (novoSetpoint > 0 && novoSetpoint < 12) {
      setpoint = novoSetpoint;
      SerialBT.print("✔️ Novo setpoint definido: ");
      SerialBT.println(setpoint);
    } else {
      SerialBT.println("❌ Setpoint inválido. Use valor entre 0 e 12 bar.");
    }
  }
  }
}
