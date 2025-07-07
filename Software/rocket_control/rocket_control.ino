#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// === FreeRTOS ===
// Handles para as tarefas
TaskHandle_t taskControlHandle;
TaskHandle_t taskCommandHandle;
TaskHandle_t taskDisplayHandle;

// Mutex para proteger o acesso a dados compartilhados entre as tarefas
SemaphoreHandle_t sharedDataMutex;

// === Instâncias Globais ===
Servo valveServo;
BluetoothSerial SerialBT;

// === Pinos ===
#define PIN_PRESSURE 34   // Entrada analógica do sensor (ADC1_6)
#define PIN_SERVO    18   // Saída PWM para o servo

// === Calibração e Filtro do Sensor ===
#define ADC_VAL_LOW 62      // Valor ADC lido em 2.0 bar (0.05V)
#define PRESSURE_VAL_LOW 2.0  // Valor em bar da pressão baixa de referência
#define ADC_VAL_HIGH 447    // Valor ADC lido em 7.0 bar (0.36V)
#define PRESSURE_VAL_HIGH 7.0 // Valor em bar da pressão alta de referência
#define MOVING_AVG_SIZE 10 // Número de amostras para o filtro de média móvel

// === Máquina de Estados ===
enum SystemState { STATE_IDLE, STATE_RUNNING, STATE_SAFETY_LOCK };
volatile SystemState currentState = STATE_IDLE; // O sistema inicia em modo ocioso

// === Estrutura de Dados Compartilhados ===
// Agrupa todos os dados que podem ser acessados por múltiplas tarefas
struct SharedData {
  float kp = 6.0;
  float ki = 1.0;
  float kd = 0.2;
  float setpoint = 3.0; // Setpoint inicial padrão, mas o controle só inicia após um novo ser enviado
  float integrative = 0.0;
  float previous_error = 0.0;
  float current_pressure = 0.0;
  float current_control = 0.0;
};

SharedData pidData;

// === Protótipos das Funções Auxiliares ===
float readPressureBar();
void setValveOpening(float percent);

// === Definições das Tarefas (Tasks) do FreeRTOS ===

/**
 * @brief Tarefa de alta frequência para o loop de controle principal.
 * Lê o sensor, executa a máquina de estados e o PID, e controla a válvula.
 */
void taskControlLoop(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // Executa a cada 50ms (20Hz)
  xLastWakeTime = xTaskGetTickCount();
  float dt = (float)xFrequency / configTICK_RATE_HZ; // Delta T constante

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency); // Garante a frequência exata

    // --- Máquina de Estados ---
    switch (currentState) {
      case STATE_IDLE:
        // No estado ocioso, apenas garante que a válvula permaneça fechada.
        // Nenhuma ação de controle é executada.
        setValveOpening(0);
        break;

      case STATE_RUNNING:
        // Tenta obter o mutex para acessar os dados do PID
        if (xSemaphoreTake(sharedDataMutex, (TickType_t)10) == pdTRUE) {
          
          pidData.current_pressure = readPressureBar();
          
          // Lógica do PID
          float error = pidData.setpoint - pidData.current_pressure;
          pidData.integrative += error * dt;
          pidData.integrative = constrain(pidData.integrative, -100.0, 100.0);
          float derivative = (error - pidData.previous_error) / dt;
          pidData.previous_error = error;
          
          pidData.current_control = pidData.kp * error + pidData.ki * pidData.integrative + pidData.kd * derivative;
          pidData.current_control = constrain(pidData.current_control, 0.0, 100.0);

          // Libera o mutex
          xSemaphoreGive(sharedDataMutex);
          
          setValveOpening(pidData.current_control);

          // Verificação de segurança (Evento de Sobrepressão)
          if (pidData.current_pressure > 15.0) {
            currentState = STATE_SAFETY_LOCK;
            Serial.println("PERIGO: Pressao acima de 15 bar! Entrando em modo de seguranca.");
            SerialBT.println("PERIGO: Pressao acima de 15 bar! Entrando em modo de seguranca.");
          }
        }
        break;

      case STATE_SAFETY_LOCK:
        setValveOpening(0); // Mantém a válvula fechada
        // O sistema permanece neste estado até ser reiniciado
        break;
    }
  }
}

/**
 * @brief Tarefa para lidar com comandos recebidos via Bluetooth.
 * Responsável por disparar o evento de início do controle.
 */
void taskCommandHandler(void *parameter) {
  for (;;) {
    if (SerialBT.available()) {
      String cmd = SerialBT.readStringUntil('\n');
      cmd.trim();

      if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
        if (cmd.equalsIgnoreCase("home")) {
          setValveOpening(100); // Comando manual direto
          String msg = "OK: Comando HOME recebido. Valvula 100% aberta.";
          Serial.println(msg);
          SerialBT.println(msg);
        } else {
          float novoSetpoint = cmd.toFloat();
          if (novoSetpoint > 0.0 && novoSetpoint < 14.5) {
            pidData.setpoint = novoSetpoint;
            pidData.integrative = 0; // Zera o termo integral ao mudar o setpoint
            
            String msg;
            // Evento: Recebimento de Setpoint Válido
            // Se o sistema estava ocioso, este evento o inicia.
            if (currentState == STATE_IDLE) {
              currentState = STATE_RUNNING;
              msg = "Setpoint inicial recebido. Iniciando controle para " + String(pidData.setpoint, 2) + " bar.";
            } else {
              msg = "Novo setpoint definido: " + String(pidData.setpoint, 2) + " bar.";
            }
            Serial.println(msg);
            SerialBT.println("OK: " + msg);

          } else {
            String msg = "Erro: Comando ou Setpoint invalido ("+ cmd +").";
            Serial.println(msg);
            SerialBT.println(msg);
          }
        }
        xSemaphoreGive(sharedDataMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Verifica por novos comandos 10x por segundo
  }
}

/**
 * @brief Tarefa de baixa frequência para exibir dados no Serial e Bluetooth.
 */
void taskDisplay(void *parameter) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(500)); // Envia dados a cada 500ms

    if (xSemaphoreTake(sharedDataMutex, (TickType_t)10) == pdTRUE) {
      int adc_val = analogRead(PIN_PRESSURE);

      // Envia dados para o Serial USB
      Serial.print("Setpoint: "); Serial.print(pidData.setpoint, 2);
      Serial.print(" bar | Pressao: "); Serial.print(pidData.current_pressure, 2);
      Serial.print(" bar | Valvula: "); Serial.print(pidData.current_control, 1);
      Serial.print(" % | ADC: "); Serial.println(adc_val);

      // Envia dados para o Bluetooth
      String bt_data = "P:" + String(pidData.current_pressure, 2) + "|S:" + String(pidData.setpoint, 2) + "|V:" + String(pidData.current_control, 1);
      SerialBT.println(bt_data);

      xSemaphoreGive(sharedDataMutex);
    }
  }
}


// === Funções Auxiliares ===
float readPressureBar() {
  static float readings[MOVING_AVG_SIZE];
  static int readIndex = 0;
  static float total = 0;
  static bool isFilterInitialized = false;

  int adc_raw = analogRead(PIN_PRESSURE);
  float current_pressure = (float)(adc_raw - ADC_VAL_LOW) * (PRESSURE_VAL_HIGH - PRESSURE_VAL_LOW) / (float)(ADC_VAL_HIGH - ADC_VAL_LOW) + PRESSURE_VAL_LOW;
  current_pressure = constrain(current_pressure, 0.0, 60.0);

  if (!isFilterInitialized) {
    for (int i = 0; i < MOVING_AVG_SIZE; i++) readings[i] = current_pressure;
    total = current_pressure * MOVING_AVG_SIZE;
    isFilterInitialized = true;
  }

  total -= readings[readIndex];
  readings[readIndex] = current_pressure;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % MOVING_AVG_SIZE;

  return total / MOVING_AVG_SIZE;
}

void setValveOpening(float percent) {
  percent = constrain(percent, 0.0, 100.0);
  int angle = map((long)percent, 0, 100, 90, 0);
  valveServo.write(angle);
}


// === Setup ===
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Pressao_RTOS");

  valveServo.attach(PIN_SERVO);
  pinMode(PIN_PRESSURE, INPUT);
  
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PRESSURE, ADC_11db);

  Serial.println("Sistema RTOS iniciado. Aguardando setpoint via Bluetooth para iniciar o controle.");
  setValveOpening(0); // Garante que a válvula comece fechada

  // Cria o mutex para sincronização
  sharedDataMutex = xSemaphoreCreateMutex();
  if (sharedDataMutex == NULL) {
    Serial.println("Erro ao criar o Mutex");
    while(1);
  }

  // Cria as tarefas
  xTaskCreate(taskControlLoop, "Control Loop", 4096, NULL, 3, &taskControlHandle);
  xTaskCreate(taskCommandHandler, "Command Handler", 2048, NULL, 2, &taskCommandHandle);
  xTaskCreate(taskDisplay, "Display", 2048, NULL, 1, &taskDisplayHandle);
  
  // A função setup termina e o escalonador do FreeRTOS assume.
}


void loop() {
  vTaskDelete(NULL);
}