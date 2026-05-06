#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Wire.h>

BluetoothSerial SerialBT;

// ======================
// PARÂMETROS FÍSICOS E DE TRAJETÓRIA
// ======================
const float RAIO_RODA = 0.030;    
const float L = 0.130;              
const float REDUCAO = 1.5; 
const float PULSOS_POR_VOLTA = 40.0 * REDUCAO; // 60 pulsos (usando CHANGE)
const float DIST_POR_PULSO = (2.0 * PI * RAIO_RODA) / PULSOS_POR_VOLTA;

const unsigned long TEMPO_RETA = 2000; 
const int VELOCIDADE_PWM = 200;
const float ANGULO_ALVO = PI / 2.0; // 90 graus

// ======================
// SETUP IMU (MPU6050)
// ======================
const int MPU_ADDR = 0x68;
float gyro_z_bias = 0;

void setupIMU() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Registro de energia
  Wire.write(0);    // Acorda MPU
  Wire.endTransmission(true);
  
  // Calibração de Bias
  float soma = 0;
  for(int i=0; i<200; i++) {
    soma += readGyroZRaw();
    delay(5);
  }
  gyro_z_bias = soma / 200.0;
}

float readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // Registro Gyro Z Out High
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return (float)raw / 131.0; // Conversão para graus/s
}

// ======================
// SETUP HARDWARE
// ======================
const int ENCODER_ESQ_PIN = 32;
const int ENCODER_DIR_PIN = 33;
const int MOTOR_ESQ_IN1 = 25, MOTOR_ESQ_IN2 = 26, MOTOR_ESQ_EN = 27;
const int MOTOR_DIR_IN1 = 14, MOTOR_DIR_IN2 = 12, MOTOR_DIR_EN = 13;

volatile long ticks_esq = 0, ticks_dir = 0;
int sinal_esq = 1, sinal_dir = 1;

void IRAM_ATTR callback_esq() { ticks_esq++; }
void IRAM_ATTR callback_dir() { ticks_dir++; }

// ======================
// POSES (COMPARATIVAS)
// ======================
float px_enc = 0, py_enc = 0, pt_enc = 0;
float px_imu = 0, py_imu = 0, pt_imu = 0;

void motores(int ve, int vd) {
  digitalWrite(MOTOR_ESQ_IN1, ve > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_ESQ_IN2, ve > 0 ? LOW : HIGH);
  digitalWrite(MOTOR_DIR_IN1, vd > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_DIR_IN2, vd > 0 ? LOW : HIGH);
  analogWrite(MOTOR_ESQ_EN, abs(ve));
  analogWrite(MOTOR_DIR_EN, abs(vd));
  sinal_esq = ve > 0 ? 1 : (ve < 0 ? -1 : 0);
  sinal_dir = vd > 0 ? 1 : (vd < 0 ? -1 : 0);
}

// ======================
// MAIN
// ======================
unsigned long last_odo_time = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Tanque_Quadrado");
  setupIMU();

  pinMode(ENCODER_ESQ_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DIR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_ESQ_PIN), callback_esq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DIR_PIN), callback_dir, CHANGE);
  
  pinMode(MOTOR_ESQ_IN1, OUTPUT); pinMode(MOTOR_ESQ_IN2, OUTPUT); pinMode(MOTOR_ESQ_EN, OUTPUT);
  pinMode(MOTOR_DIR_IN1, OUTPUT); pinMode(MOTOR_DIR_IN2, OUTPUT); pinMode(MOTOR_DIR_EN, OUTPUT);

  delay(10000); // Espera para conectar bluetooth
  SerialBT.println("Tempo,X_enc,Y_enc,Theta_enc,X_imu,Y_imu,Theta_imu");
}

void loop() {
  static int etapa = 0;
  static int volta = 0;
  static unsigned long tempo_etapa = 0;
  static float theta_inicio_curva = 0;

  unsigned long now = millis();
  float dt = (now - last_odo_time) / 1000.0;

  // LOOP ODOMETRIA (Frequência DT_MS)
  if (now - last_odo_time >= 10) {
    last_odo_time = now;
    
    noInterrupts();
    long tE = ticks_esq; long tD = ticks_dir;
    interrupts();
    
    static long lE = 0, lD = 0;
    long dE = (tE - lE) * sinal_esq;
    long dD = (tD - lD) * sinal_dir;
    lE = tE; lD = tD;

    float ds = ((dD + dE) * DIST_POR_PULSO) / 2.0;

    // 1. Odometria pura (Encoder)
    float dt_enc = ((dD - dE) * DIST_POR_PULSO) / L;
    px_enc += ds * cos(pt_enc + dt_enc/2.0);
    py_enc += ds * sin(pt_enc + dt_enc/2.0);
    pt_enc += dt_enc;

    // 2. Odometria Híbrida (Encoder + IMU)
    float gz = (readGyroZRaw() - gyro_z_bias) * (PI / 180.0);
    float dt_imu = gz * dt;
    px_imu += ds * cos(pt_imu + dt_imu/2.0);
    py_imu += ds * sin(pt_imu + dt_imu/2.0);
    pt_imu += dt_imu;

    // Log Bluetooth
    SerialBT.printf("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", 
                    now/1000.0, px_enc, py_enc, pt_enc, px_imu, py_imu, pt_imu);
  }

  // MÁQUINA DE ESTADOS DO QUADRADO
  if (volta < 4) {
    if (etapa == 0) { // Reta
      motores(VELOCIDADE_PWM, VELOCIDADE_PWM);
      if (tempo_etapa == 0) tempo_etapa = now;
      if (now - tempo_etapa > TEMPO_RETA) {
        motores(0, 0); delay(500);
        etapa = 1; tempo_etapa = 0;
        theta_inicio_curva = pt_imu;
      }
    } 
    else if (etapa == 1) { // Curva baseada na IMU
      motores(VELOCIDADE_PWM, -VELOCIDADE_PWM);
      if (abs(pt_imu - theta_inicio_curva) >= ANGULO_ALVO) {
        motores(0, 0); delay(500);
        etapa = 0;
        volta++;
      }
    }
  } else {
    motores(0, 0);
  }
}