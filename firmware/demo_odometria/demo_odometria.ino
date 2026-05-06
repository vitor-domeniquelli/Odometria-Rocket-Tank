#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Wire.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth nao esta habilitado!
#endif

BluetoothSerial SerialBT;

// ======================
// PARÂMETROS FÍSICOS (FIXOS)
// ======================
const float RAIO_RODA = 0.068;//0.030;    
const float REDUCAO = 1.5; 
const float PULSOS_POR_VOLTA_BASE = 40.0; // 20 furos * 2 (CHANGE)
const float DIST_POR_PULSO = (2.0 * PI * RAIO_RODA) / (PULSOS_POR_VOLTA_BASE * REDUCAO);

// ======================
// PARÂMETROS AJUSTÁVEIS VIA BLUETOOTH
// ======================
float L = 0.130;            // Bitola (ajustável com 'L:valor')[cite: 1, 2]
unsigned long TEMPO_RETA = 2000; 
int VEL_RETA = 160;
int VEL_CURVA = 200; 
float ANGULO_ALVO = PI / 2.0;

// ======================
// PINOS E HARDWARE
// ======================
const int ENCODER_ESQ_PIN = 32;
const int ENCODER_DIR_PIN = 33;
const int MOTOR_ESQ_IN1 = 25, MOTOR_ESQ_IN2 = 26, MOTOR_ESQ_EN = 27;
const int MOTOR_DIR_IN1 = 14, MOTOR_DIR_IN2 = 12, MOTOR_DIR_EN = 13;
const int MPU_ADDR = 0x68;

// ======================
// VARIÁVEIS DE ESTADO E ODOMETRIA
// ======================
enum EstadoRobo { IDLE, LINEAR, QUADRADO };
EstadoRobo estadoAtual = IDLE;

volatile long ticks_esq = 0, ticks_dir = 0;
float px_imu = 0, py_imu = 0, pt_imu = 0;
int sinal_esq = 0, sinal_dir = 0;
float gyro_z_bias = 0;
long last_tE = 0, last_tD = 0; 

unsigned long last_odo_time = 0;
unsigned long tempo_missao_inicio = 0;
int volta_quadrado = 0;
int sub_etapa_quadrado = 0; // 0: Reta, 1: Curva
float theta_inicio_curva = 0;

// ======================
// INTERRUPÇÕES
// ======================
void IRAM_ATTR callback_esq() { ticks_esq++; }
void IRAM_ATTR callback_dir() { ticks_dir++; }

// ======================
// FUNÇÕES DE APOIO
// ======================
float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return ((float)raw / 131.0) - gyro_z_bias;
}

void motores(int ve, int vd) {
  digitalWrite(MOTOR_ESQ_IN1, ve > 0 ? HIGH : (ve < 0 ? LOW : LOW));
  digitalWrite(MOTOR_ESQ_IN2, ve > 0 ? LOW : (ve < 0 ? HIGH : LOW));
  digitalWrite(MOTOR_DIR_IN1, vd > 0 ? HIGH : (vd < 0 ? LOW : LOW));
  digitalWrite(MOTOR_DIR_IN2, vd > 0 ? LOW : (vd < 0 ? HIGH : LOW));
  analogWrite(MOTOR_ESQ_EN, abs(ve));
  analogWrite(MOTOR_DIR_EN, abs(vd));
  sinal_esq = (ve > 0) ? 1 : (ve < 0 ? -1 : 0);
  sinal_dir = (vd > 0) ? 1 : (vd < 0 ? -1 : 0);
}

void resetOdometria() {
  px_imu = 0; py_imu = 0; pt_imu = 0;
  noInterrupts();
  ticks_esq = 0; ticks_dir = 0;
  last_tE = 0; last_tD = 0;
  interrupts();
  SerialBT.println("Odometria resetada. Tempo,X,Y,Theta,TicksE,TicksD,Gz");
  SerialBT.printf("%.3f,%.3f,%.3f,%ld,%ld\n", px_imu, py_imu, pt_imu, ticks_esq, ticks_dir);
}

void processarComando(String msg) {
  msg.trim();
  if (msg.length() == 0) return;
  char cmd = toupper(msg[0]);

  if (msg.length() == 1) {
    if (cmd == 'L') { resetOdometria(); tempo_missao_inicio = millis(); estadoAtual = LINEAR; }
    else if (cmd == 'Q') { resetOdometria(); volta_quadrado = 0; sub_etapa_quadrado = 0; estadoAtual = QUADRADO; }
    else if (cmd == 'S') { estadoAtual = IDLE; motores(0, 0); SerialBT.println("STOP"); }
    return;
  }

  if (msg.indexOf(':') != -1) {
    float valor = msg.substring(msg.indexOf(':') + 1).toFloat();
    switch (cmd) {
      case 'L': L = valor; SerialBT.printf("Ajuste: L = %.3f\n", L); break;
      case 'T': TEMPO_RETA = (unsigned long)valor; SerialBT.printf("Ajuste: Tempo Reta = %lu\n", TEMPO_RETA); break;
      case 'V': VEL_RETA = (int)valor; SerialBT.printf("Ajuste: Vel Reta = %d\n", VEL_RETA); break;
      case 'C': VEL_CURVA = (int)valor; SerialBT.printf("Ajuste: Vel Curva = %d\n", VEL_CURVA); break;
    }
  }
}

// ======================
// SETUP E LOOP
// ======================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Tanque_Odometria");
  Wire.begin();
  
  // Setup MPU6050
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

  pinMode(ENCODER_ESQ_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DIR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_ESQ_PIN), callback_esq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DIR_PIN), callback_dir, CHANGE);

  pinMode(MOTOR_ESQ_IN1, OUTPUT); pinMode(MOTOR_ESQ_IN2, OUTPUT); pinMode(MOTOR_ESQ_EN, OUTPUT);
  pinMode(MOTOR_DIR_IN1, OUTPUT); pinMode(MOTOR_DIR_IN2, OUTPUT); pinMode(MOTOR_DIR_EN, OUTPUT);

  // Calibração IMU
  float soma = 0;
  for(int i=0; i<100; i++) { soma += (readGyroZ() + gyro_z_bias); delay(10); }
  gyro_z_bias = soma / 100.0;

  SerialBT.println("Robo pronto. L: Linear | Q: Quadrado | S: Stop");
}

void loop() {
  unsigned long now = millis();

  if (SerialBT.available()) {
    String entrada = SerialBT.readStringUntil('\n');
    processarComando(entrada);
  }

  // Bloco de Odometria (20Hz)[cite: 1, 2]
  if (now - last_odo_time >= 50) {
    float dt = (now - last_odo_time) / 1000.0;
    last_odo_time = now;

    noInterrupts();
    long tE = ticks_esq; long tD = ticks_dir;
    interrupts();

    long dE = (tE - last_tE) * sinal_esq;
    long dD = (tD - last_tD) * sinal_dir;
    last_tE = tE; last_tD = tD;

    float ds = ((dE + dD) * DIST_POR_PULSO) / 2.0;
    float gz_rad = readGyroZ() * (PI / 180.0);
    float dth = gz_rad * dt;

    px_imu += ds * cos(pt_imu + (dth / 2.0));
    py_imu += ds * sin(pt_imu + (dth / 2.0));
    pt_imu += dth;

    if (estadoAtual != IDLE) {
      SerialBT.printf("%.2f,%.3f,%.3f,%.3f,%ld,%ld,%.2f\n", 
                      now/1000.0, px_imu, py_imu, pt_imu, tE, tD, readGyroZ());
    }
  }

  // Máquina de Estados
  switch (estadoAtual) {
    case LINEAR:
      if (now - tempo_missao_inicio < TEMPO_RETA) {
        motores(VEL_RETA, VEL_RETA);
      } else {
        motores(0, 0); estadoAtual = IDLE;
        SerialBT.println("Fim do Teste Linear.");
      }
      break;

    case QUADRADO:
      if (volta_quadrado < 4) {
        if (sub_etapa_quadrado == 0) { // Reta
          motores(VEL_RETA, VEL_RETA);
          if (tempo_missao_inicio == 0) tempo_missao_inicio = now;
          if (now - tempo_missao_inicio > TEMPO_RETA) {
            motores(0, 0); delay(500);
            sub_etapa_quadrado = 1; theta_inicio_curva = pt_imu;
          }
        } else { // Curva
          motores(VEL_CURVA, -VEL_CURVA);
          if (abs(pt_imu - theta_inicio_curva) >= ANGULO_ALVO) {
            motores(0, 0); delay(500);
            sub_etapa_quadrado = 0; tempo_missao_inicio = 0; volta_quadrado++;
          }
        }
      } else {
        estadoAtual = IDLE;
        SerialBT.println("Fim do Quadrado.");
      }
      break;
    
    case IDLE:
      break;
  }
}