#include <Arduino.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>


// Verifica se o Bluetooth está habilitado no firmware ESP32
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth não está habilitado! Selecione uma placa ESP32 e ative CONFIG_BT_ENABLED/CONFIG_BLUEDROID_ENABLED."
#endif

// === Instâncias Globais ===

// Servo (biblioteca ESP32Servo já configura o canal PWM internamente)
Servo valveServo;  // objeto para controlar o servo da válvula

// Bluetooth SPP (Serial-over-Bluetooth)
BluetoothSerial SerialBT;  // instancia a interface Serial Bluetooth

// === Pinos ===
#define PIN_PRESSURE 34   // Entrada analógica do sensor (ADC1_6)
#define PIN_SERVO    18   // Saída PWM para o servo

// === PID ===
float kp = 5.0;
float ki = 1.0;
float kd = 0.2;

float setpoint       = 5.0;   // Pressão alvo [bar]
float integrative    = 0.0;   // Acumulador da parte integral
float previous_error = 0.0;   // Erro anterior (para derivada)
unsigned long lastTime = 0;   // Timestamp da última iteração

// === Funções Auxiliares ===

// Lê o sensor de pressão e converte para bar
float readPressureBar() {
  int   adc_raw          = analogRead(PIN_PRESSURE);           // 0–4095 (ADC 12-bit)
  float voltage_adc      = adc_raw * (3.3 / 4095.0);           // 0–3.3 V (esp32)
  float voltage_original = voltage_adc * (5.0 / 3.3);          // reverte divisor (sensor entrega 0–5 V)

  // Supondo sensor 0–5 V → 0–1,2 MPa (linear)
  float pressure_mpa = (voltage_original / 5.0) * 1.2;        
  float pressure_bar = pressure_mpa * 10.0;  // 1 MPa = 10 bar

  return pressure_bar;
}

// Calcula saída PID (percentual de abertura da válvula)
float computePID(float pressure, float dt) {
  float error      = setpoint - pressure;              
  integrative    += error * dt;                       
  float derivative = (error - previous_error) / dt;   
  previous_error  = error;                            

  float output = kp * error + ki * integrative + kd * derivative;
  // Garante saída entre 0 e 100 (%)
  return constrain(output, 0.0, 100.0);
}

// Move o servo de acordo com porcentagem [0–100] mapeada para 0°–90°
void setValveOpening(float percent) {
  percent = constrain(percent, 0.0, 100.0);
  int angle = map((int)percent, 0, 100, 0, 90);
  valveServo.write(angle);
}

// === Setup e Loop ===

void setup() {
  Serial.begin(115200);           // Debug via USB
  SerialBT.begin("ESP32_Rocket"); // Inicializa Bluetooth SPP com nome visível

  // Prepara o servo (ESP32Servo cuida de ledcSetup/ledcAttachPin internamente)
  valveServo.attach(PIN_SERVO);

  // Pino do transdutor como entrada analógica
  pinMode(PIN_PRESSURE, INPUT);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;        // Tempo decorrido (segundos)
  if (dt < 0.01) return;                     // Amostragem mínima de 10 ms
  lastTime = now;

  // 1) Lê a pressão
  float pressure = readPressureBar();

  // 2) Calcula saída PID [0–100%]
  float control = computePID(pressure, dt);

  // 3) Move servo de válvula
  setValveOpening(control);

  // 4) Debug via Serial USB
  Serial.print("Pressão: ");
  Serial.print(pressure, 2);
  Serial.print(" bar | Válvula: ");
  Serial.print(control, 1);
  Serial.println(" %");

  // 5) Envia os mesmos dados via Bluetooth
  SerialBT.print("Pressão: ");
  SerialBT.print(pressure, 2);
  SerialBT.print(" bar | Válvula: ");
  SerialBT.print(control, 1);
  SerialBT.println(" %");

  // 6) Segurança: se ultrapassar 10 bar, fecha a válvula
  if (pressure > 10.0) {
    setValveOpening(0);
    Serial.println("⚠️ Pressão acima do limite! Sistema travado.");
    SerialBT.println("⚠️ Pressão acima do limite! Sistema travado.");
    while (true) {
      // trava tudo em caso de sobrepressão
      delay(1000);
    }
  }

  // 7) Verifica se chegou novo setpoint via Bluetooth
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    float novoSetpoint = cmd.toFloat();
    if (novoSetpoint > 0.0 && novoSetpoint <= 12.0) {
      setpoint = novoSetpoint;
      SerialBT.print("✔️ Novo setpoint definido: ");
      SerialBT.println(setpoint);
    } else {
      SerialBT.println("❌ Setpoint inválido. Use valor entre 0 e 12 bar.");
    }
  }

  // Amostragem de ~20 Hz
  delay(1000);
}
