#include <Arduino.h>
#include "BluetoothSerial.h"

// Verifica se o Bluetooth está habilitado no core do ESP32
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth nao esta habilitado!
#endif

BluetoothSerial SerialBT;

// ======================
// PARÂMETROS FÍSICOS E DE TESTE
// ======================
const float RAIO_RODA = 0.030;    
const float L = 0.130;            

const float REDUCAO = 27.0 / 18.0; 
const float PULSOS_POR_VOLTA = 40.0 * REDUCAO;
const float DIST_POR_PULSO = (2.0 * PI * RAIO_RODA) / PULSOS_POR_VOLTA;

const unsigned long TEMPO_MOVIMENTO = 3000; 
const int VELOCIDADE_PWM = 150;             

// ======================
// SETUP PINOS ESP32 (Ajustar)
// ======================
const int ENCODER_ESQ_PIN = 32;
const int ENCODER_DIR_PIN = 33;

const int MOTOR_ESQ_IN1 = 25;
const int MOTOR_ESQ_IN2 = 26;
const int MOTOR_ESQ_EN  = 27;

const int MOTOR_DIR_IN1 = 14;
const int MOTOR_DIR_IN2 = 12;
const int MOTOR_DIR_EN  = 13;

// Variáveis globais
volatile long ticks_esq = 0;
volatile long ticks_dir = 0;

float pose_x = 0.0;
float pose_y = 0.0;
float pose_theta = 0.0;

unsigned long start_time = 0;
unsigned long last_odo_time = 0;
const unsigned long DT_MS = 5; 

bool running = true;

// ======================
// INTERRUPÇÕES PARA ENCODERS
// ======================
void IRAM_ATTR callback_esq() {
  ticks_esq++;
}

void IRAM_ATTR callback_dir() {
  ticks_dir++;
}

// ======================
// CONTROLE DE MOTORES
// ======================
void parar_motores() {
  analogWrite(MOTOR_ESQ_EN, 0);
  analogWrite(MOTOR_DIR_EN, 0);
  digitalWrite(MOTOR_ESQ_IN1, LOW);
  digitalWrite(MOTOR_ESQ_IN2, LOW);
  digitalWrite(MOTOR_DIR_IN1, LOW);
  digitalWrite(MOTOR_DIR_IN2, LOW);
}

void motores_para_frente(int velocidade) {
  digitalWrite(MOTOR_ESQ_IN1, HIGH);
  digitalWrite(MOTOR_ESQ_IN2, LOW);
  digitalWrite(MOTOR_DIR_IN1, HIGH);
  digitalWrite(MOTOR_DIR_IN2, LOW);
  analogWrite(MOTOR_ESQ_EN, velocidade);
  analogWrite(MOTOR_DIR_EN, velocidade);
}

// ======================
// MAIN (SETUP & LOOP)
// ======================
void setup() {
  // Mantém a serial USB ativa para debug de hardware
  Serial.begin(115200);
  
  // Inicializa o Bluetooth com o nome visível para o notebook
  SerialBT.begin("ESP32_Odometria"); 
  Serial.println("Bluetooth Iniciado. Pareie com 'ESP32_Odometria'.");

  pinMode(ENCODER_ESQ_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DIR_PIN, INPUT_PULLUP);

  pinMode(MOTOR_ESQ_IN1, OUTPUT);
  pinMode(MOTOR_ESQ_IN2, OUTPUT);
  pinMode(MOTOR_ESQ_EN, OUTPUT);
  
  pinMode(MOTOR_DIR_IN1, OUTPUT);
  pinMode(MOTOR_DIR_IN2, OUTPUT);
  pinMode(MOTOR_DIR_EN, OUTPUT);


  // RISING - Lê apenas quando o sinal vai de LOW para HIGH | CHANGE - Lê borda de subida e descida
  // attachInterrupt(digitalPinToInterrupt(ENCODER_ESQ_PIN), callback_esq, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_DIR_PIN), callback_dir, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCODER_ESQ_PIN), callback_esq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DIR_PIN), callback_dir, CHANGE);

  // Aguarda 5 segundos para você ter tempo de conectar o terminal via Bluetooth
  // antes de o robô começar a se mover e gerar dados.
  delay(5000); 

  // Envia cabeçalho via Bluetooth
  SerialBT.println("Tempo,X,Y,Theta");

  start_time = millis();
  last_odo_time = start_time;

  motores_para_frente(VELOCIDADE_PWM);
}

void loop() {
  if (!running) return;

  unsigned long current_time = millis();
  unsigned long elapsed = current_time - start_time;

  if (elapsed >= TEMPO_MOVIMENTO) {
    parar_motores();
    running = false;
    SerialBT.println("--- FIM DO TESTE ---");
    SerialBT.printf("Ticks Esq: %ld\n", ticks_esq);
    SerialBT.printf("Ticks Dir: %ld\n", ticks_dir);
    return;
  }

  if (current_time - last_odo_time >= DT_MS) {
    last_odo_time = current_time;

    noInterrupts();
    long current_ticks_esq = ticks_esq;
    long current_ticks_dir = ticks_dir;
    interrupts();

    static long last_ticks_esq = 0;
    static long last_ticks_dir = 0;

    long delta_esq = current_ticks_esq - last_ticks_esq;
    long delta_dir = current_ticks_dir - last_ticks_dir;
    
    last_ticks_esq = current_ticks_esq;
    last_ticks_dir = current_ticks_dir;

    float delta_s_esq = delta_esq * DIST_POR_PULSO;
    float delta_s_dir = delta_dir * DIST_POR_PULSO;

    float delta_s = (delta_s_dir + delta_s_esq) / 2.0;
    float delta_theta = (delta_s_dir - delta_s_esq) / L;

    pose_x += delta_s * cos(pose_theta + delta_theta);
    pose_y += delta_s * sin(pose_theta + delta_theta);
    pose_theta += delta_theta;

    // Envia dados para o notebook via Bluetooth
    SerialBT.print(elapsed / 1000.0, 3);
    SerialBT.print(",");
    SerialBT.print(pose_x, 4);
    SerialBT.print(",");
    SerialBT.print(pose_y, 4);
    SerialBT.print(",");
    SerialBT.println(pose_theta, 4);
  }
}