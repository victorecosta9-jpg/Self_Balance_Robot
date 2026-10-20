
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "driver/gpio.h"

// SEGURANCA
#define MOTORS_ENABLED 0
#define USE_MPU_INT 0
#define GYRO_FROM_DMP 1
#define STREAM_SERIAL 1

// PINOS
#define PIN_I2C_SCL 22
#define PIN_I2C_SDA 23
#define PIN_MPU_INT 4
#define PIN_LED 2
#define PIN_BOOT 0

#define PIN_M_ESQ_PWM 25
#define PIN_M_ESQ_IN1 27
#define PIN_M_ESQ_IN2 26
#define PIN_M_DIR_PWM 14
#define PIN_M_DIR_IN1 13
#define PIN_M_DIR_IN2 12

#define PIN_ENC_ESQ_A 33
#define PIN_ENC_ESQ_B 32
#define PIN_ENC_DIR_A 35
#define PIN_ENC_DIR_B 34

// CONTROLE
const float K_X = 0.0f;
const float K_THETA = -30.0f;
const float K_V = 0.0f;
const float K_W = -0.5f;

const float U_LIMIT_V = 11.1f;
const float RAMP_S = 0.15f;
const float TRIM_DEG = -1.5f;

#define WIN_V 4

const float TS = 0.01f;
const uint32_t TS_US = 10000;
const float ANGLE_SIGN = 1.0f;
const float GYRO_SIGN = 1.0f;

// MOTORES
const float VBAT_V = 11.1f;
const float WHEEL_DIAMETER_M = 0.0683f;
const float PULSES_PER_REV = 700.0f;

const float DIST_PER_PULSE =
    (PI * WHEEL_DIAMETER_M) / PULSES_PER_REV;

const float DEADBAND_V = 0.65f;
const float DEADBAND_RAMP_V = 0.2f;

#define PWM_FREQ_HZ 20000
#define PWM_BITS 10
#define PWM_MAX ((1 << PWM_BITS) - 1)

// IMU
#define I2C_FREQ_HZ 100000

const float DMP_GYRO_LSB_PER_DPS = 16.4f;
const uint32_t DMP_TIMEOUT_US = 50000;
const float ANGLE_FALL_LIMIT_DEG = 35.0f;
const float START_WINDOW_DEG = 5.0f;
const int START_OK_SAMPLES = 20;
const uint32_t START_TIMEOUT_MS = 5000;
const uint32_t DMP_SETTLE_MS = 2000;
const int REF_SAMPLES = 100;

#define NLOG 1500

enum SM_SM {
  SM_INICIALIZANDO,
  SM_ESPERA_VERTICAL,
  SM_AUTOMATICO,
  SM_PARADO
};

SM_SM estado = SM_INICIALIZANDO;

// CLASSE DO MOTOR
class Motor {
 public:
  Motor(uint8_t pwm, uint8_t in1, uint8_t in2, uint8_t canal)
      : pwm_(pwm), in1_(in1), in2_(in2), canal_(canal) {}

  void begin() {
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);

    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && \
    (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(pwm_, PWM_FREQ_HZ, PWM_BITS);
#else
    ledcSetup(canal_, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(pwm_, canal_);
#endif

    escreveDuty(0);
  }

  void setVoltage(float volts) {
#if !MOTORS_ENABLED
    (void)volts;
    stop();
    return;
#else
    volts = fmaxf(-VBAT_V, fminf(VBAT_V, volts));

    float a = fabsf(volts);

    if (a < 0.02f) {
      stop();
      return;
    }

    float eff =
        DEADBAND_V * fminf(1.0f, a / DEADBAND_RAMP_V)
        + (a / VBAT_V) * (VBAT_V - DEADBAND_V);

    if (volts > 0) {
      digitalWrite(in1_, HIGH);
      digitalWrite(in2_, LOW);
    } else {
      digitalWrite(in1_, LOW);
      digitalWrite(in2_, HIGH);
    }

    float duty = fminf(1.0f, eff / VBAT_V);
    escreveDuty((uint32_t)(duty * PWM_MAX));
#endif
  }

  void stop() {
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
    escreveDuty(0);
  }

 private:
  void escreveDuty(uint32_t d) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && \
    (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pwm_, d);
#else
    ledcWrite(canal_, d);
#endif
  }

  uint8_t pwm_, in1_, in2_, canal_;
};

Motor motorEsq(PIN_M_ESQ_PWM, PIN_M_ESQ_IN1, PIN_M_ESQ_IN2, 0);
Motor motorDir(PIN_M_DIR_PWM, PIN_M_DIR_IN1, PIN_M_DIR_IN2, 1);

// ENCODERS
volatile int32_t pulsosEsq = 0;
volatile int32_t pulsosDir = 0;

void IRAM_ATTR isrEncEsq() {
  if (gpio_get_level((gpio_num_t)PIN_ENC_ESQ_B))
    pulsosEsq++;
  else
    pulsosEsq--;
}

void IRAM_ATTR isrEncDir() {
  if (gpio_get_level((gpio_num_t)PIN_ENC_DIR_B))
    pulsosDir--;
  else
    pulsosDir++;
}

// MPU / DMP
MPU6050 mpu;

uint8_t devStatus = 255;
uint16_t packetSize = 0;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

#if USE_MPU_INT
volatile bool mpuInterrupt = false;

void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}
#endif

float dmp_roll_deg = 0.0f;
float dmp_gyro_dps = 0.0f;
float dmp_gyro_deriv_dps = 0.0f;
float dmp_roll_ant = 0.0f;

uint32_t dmp_ultimo_us = 0;
uint32_t dmp_pacotes = 0;
uint32_t fifo_overflow = 0;
uint32_t t_poll_us = 0;
uint32_t t_deriv_us = 0;

static inline float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// LEITURA FIFO DMP
bool lerFIFO() {
  if (packetSize == 0) return false;

  uint16_t n = mpu.getFIFOCount();

  if (n >= 1024) {
    mpu.resetFIFO();
    fifo_overflow++;
    return false;
  }

  if (n < packetSize) return false;

  // Descarta pacotes antigos e preserva o mais recente.
  while (n >= (uint16_t)(packetSize * 2)) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    n -= packetSize;
  }

  if (n < packetSize) return false;

  mpu.getFIFOBytes(fifoBuffer, packetSize);

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  int16_t g[3];
  mpu.dmpGetGyro(g, fifoBuffer);

  float roll = ypr[2] * 180.0f / PI;
  uint32_t agora = micros();

  if (dmp_pacotes > 0 && t_deriv_us != 0) {
    uint32_t delta = agora - t_deriv_us;

    if (delta > 0) {
      dmp_gyro_deriv_dps =
          wrap180(roll - dmp_roll_ant) * 1000000.0f / delta;
    }
  }

  dmp_roll_ant = roll;
  t_deriv_us = agora;
  dmp_roll_deg = roll;

  dmp_gyro_dps =
      (float)g[0] / DMP_GYRO_LSB_PER_DPS;

  dmp_ultimo_us = agora;
  dmp_pacotes++;

  return true;
}

bool atualizaIMU(bool forcar) {
#if USE_MPU_INT
  if (mpuInterrupt) {
    mpuInterrupt = false;
    uint8_t st = mpu.getIntStatus();

    if (st & 0x10) {
      mpu.resetFIFO();
      fifo_overflow++;
      return false;
    }
  } else if (!forcar) {
    return false;
  }
#else
  uint32_t agora = micros();

  if (!forcar &&
      (uint32_t)(agora - t_poll_us) < 1000) {
    return false;
  }

  t_poll_us = agora;
#endif

  return lerFIFO();
}

// RECUPERACAO DO BARRAMENTO I2C
void i2cBusRecover() {
  Serial.println("[I2C] Recuperando barramento...");

  pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_I2C_SDA, OUTPUT_OPEN_DRAIN);

  digitalWrite(PIN_I2C_SCL, HIGH);
  digitalWrite(PIN_I2C_SDA, HIGH);
  delayMicroseconds(20);

  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(10);
  }

  digitalWrite(PIN_I2C_SCL, LOW);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(10);

  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(10);

  digitalWrite(PIN_I2C_SDA, HIGH);
  delayMicroseconds(10);

  Serial.println("[I2C] Recuperacao concluida.");
}

bool iniciaI2cEMpu() {
  delay(300);
  i2cBusRecover();

  for (int tentativa = 0; tentativa < 3; tentativa++) {
    Serial.printf("[I2C] Inicializacao %d/3\n", tentativa + 1);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_FREQ_HZ);
    Wire.setTimeOut(100);

    mpu.initialize();

    Wire.beginTransmission(0x68);
    uint8_t erro = Wire.endTransmission();

    if (erro == 0) {
      Serial.println("[I2C] Dispositivo respondeu em 0x68.");

      Wire.beginTransmission(0x68);
      Wire.write(0x75);

      uint8_t erroReg = Wire.endTransmission(false);
      uint8_t who = 0xFF;

      if (erroReg == 0 &&
          Wire.requestFrom((uint8_t)0x68, (uint8_t)1) == 1) {
        who = Wire.read();
      }

      Serial.printf("[IMU] WHO_AM_I = 0x%02X\n", who);
      Serial.println("[IMU] Prosseguindo para testar o DMP.");

      return true;
    }

    Serial.printf("[I2C] Sem ACK; codigo %u\n", erro);

    Wire.end();
    delay(100);
    i2cBusRecover();
  }

  return false;
}

// VARIAVEIS DE CONTROLE E LOG
float angle_ref = 0.0f;

int32_t pe_prev = 0;
int32_t pd_prev = 0;

float pos = 0.0f;
float vel = 0.0f;

uint32_t k = 0;
uint32_t li = 0;

float hp[WIN_V];
uint32_t ht[WIN_V];

uint32_t t0_us = 0;
uint32_t prox_controle_us = 0;

int ok_vertical = 0;
uint32_t t_espera_ms = 0;

float dt_max = 0.0f;
uint32_t cyc_max = 0;
uint32_t slow = 0;
uint32_t t_entrada_ant_us = 0;

bool primeiro_ciclo = true;

float log_t[NLOG];
float log_e[NLOG];
float log_g[NLOG];
float log_u[NLOG];
float log_v[NLOG];
float log_x[NLOG];

char motivo_fim[48] = "usuario";
uint32_t t_fim_us = 0;
bool relatorio_feito = false;
uint32_t ultimo_status_ms = 0;

// PARADA
void parar(const char* motivo) {
  motorEsq.stop();
  motorDir.stop();

  strncpy(motivo_fim, motivo, sizeof(motivo_fim) - 1);
  motivo_fim[sizeof(motivo_fim) - 1] = '\0';

  t_fim_us = micros();
  estado = SM_PARADO;

  detachInterrupt(digitalPinToInterrupt(PIN_ENC_ESQ_A));
  detachInterrupt(digitalPinToInterrupt(PIN_ENC_DIR_A));

  digitalWrite(PIN_LED, LOW);
}

// RELATORIO
void relatorio() {
  relatorio_feito = true;

  Serial.println();
  Serial.printf("FIM: %s\n", motivo_fim);

  if (li == 0) return;

  Serial.printf("Amostras: %lu\n", (unsigned long)li);

  Serial.printf("Pacotes DMP: %lu | overflow: %lu\n",
                (unsigned long)dmp_pacotes,
                (unsigned long)fifo_overflow);

  Serial.println("t_ms,err_deg,gyro_dps,u_V,vel_mps,pos_m");

  uint32_t n = (li < NLOG) ? li : NLOG;
  uint32_t inicio = li - n;

  for (uint32_t i = 0; i < n; i++) {
    uint32_t j = (inicio + i) % NLOG;

    Serial.printf("%.0f,%.2f,%.1f,%.2f,%.3f,%.3f\n",
                  log_t[j],
                  log_e[j],
                  log_g[j],
                  log_u[j],
                  log_v[j],
                  log_x[j]);
  }
}

// INICIO DO CONTROLE
void iniciaControle() {
  Serial.println("[CONTROLE] Inicio da rotina a 100 Hz.");

  Serial.printf("[CONTROLE] Motores: %s\n",
                MOTORS_ENABLED ? "HABILITADOS" : "DESABILITADOS");

  pe_prev = pulsosEsq;
  pd_prev = pulsosDir;

  pos = 0.0f;
  vel = 0.0f;

  k = 0;
  li = 0;

  dt_max = 0.0f;
  cyc_max = 0;
  slow = 0;

  primeiro_ciclo = true;

  t0_us = micros();

  for (int i = 0; i < WIN_V; i++) {
    hp[i] = 0.0f;
    ht[i] = t0_us;
  }

  digitalWrite(PIN_LED, HIGH);

  prox_controle_us = t0_us + TS_US;
  estado = SM_AUTOMATICO;
}

// ROTINA DE CONTROLE
void controle() {
  const uint32_t t_entrada = micros();

  atualizaIMU(true);

  const uint32_t t_c = micros();

  if (digitalRead(PIN_BOOT) == LOW) {
    parar("botao_emergencia");
    return;
  }

  if (dmp_pacotes == 0 ||
      (uint32_t)(t_c - dmp_ultimo_us) > DMP_TIMEOUT_US) {
    parar("imu_sem_dados");
    return;
  }

  const float angulo = ANGLE_SIGN * dmp_roll_deg;

  if (estado == SM_ESPERA_VERTICAL) {
    if (fabsf(wrap180(angulo - angle_ref)) < START_WINDOW_DEG) {
      ok_vertical++;
    } else {
      ok_vertical = 0;
    }

    if (ok_vertical >= START_OK_SAMPLES) {
      iniciaControle();
    } else if ((uint32_t)(millis() - t_espera_ms) >
               START_TIMEOUT_MS) {
      Serial.println("[ERRO] Timeout esperando a vertical.");
      parar("timeout_vertical");
    }

    return;
  }

  if (!primeiro_ciclo) {
    float dt =
        (uint32_t)(t_entrada - t_entrada_ant_us) * 1e-6f;

    if (dt > dt_max) dt_max = dt;
    if (dt > 0.012f) slow++;
  }

  primeiro_ciclo = false;
  t_entrada_ant_us = t_entrada;

#if GYRO_FROM_DMP
  const float gyro_dps = GYRO_SIGN * dmp_gyro_dps;
#else
  const float gyro_dps = ANGLE_SIGN * dmp_gyro_deriv_dps;
#endif

  const float angle_err_deg =
      wrap180(angulo - (angle_ref + TRIM_DEG));

  if (fabsf(angle_err_deg) > ANGLE_FALL_LIMIT_DEG) {
    char buf[40];

    snprintf(buf, sizeof(buf), "queda (%.1f deg)", angle_err_deg);

    parar(buf);
    return;
  }

  // POSICAO DOS ENCODERS
  const int32_t pe = pulsosEsq;
  const int32_t pd = pulsosDir;

  pos += ((pe - pe_prev) + (pd - pd_prev))
         * 0.5f * DIST_PER_PULSE;

  pe_prev = pe;
  pd_prev = pd;

  // VELOCIDADE
  const uint32_t idx = k % WIN_V;

  if (k >= WIN_V) {
    const float span =
        (uint32_t)(t_c - ht[idx]) * 1e-6f;

    if (span > 0.0f) {
      vel = (pos - hp[idx]) / span;
    }
  }

  hp[idx] = pos;
  ht[idx] = t_c;

  // RAMPA INICIAL
  const float t_s =
      (uint32_t)(t_c - t0_us) * 1e-6f;

  const float g_ramp =
      (RAMP_S <= 0.0f)
          ? 1.0f
          : fminf(1.0f, t_s / RAMP_S);

  // TERMOS INDIVIDUAIS DO CONTROLADOR
  const float termoX = K_X * pos;

  const float termoTheta =
      K_THETA * (angle_err_deg * (PI / 180.0f));

  const float termoV = K_V * vel;

  const float termoW =
      K_W * (gyro_dps * (PI / 180.0f));

  // CALCULO ORIGINAL DO COMANDO
  float u = -(termoX + termoTheta + termoV + termoW);

  u *= g_ramp;

  // SATURACAO DO COMANDO
  u = fmaxf(-U_LIMIT_V, fminf(U_LIMIT_V, u));

  // MOTORES PERMANECEM DESABILITADOS PELO DEFINE
  motorEsq.setVoltage(-u);
  motorDir.setVoltage(u);

  // REGISTRO DAS AMOSTRAS
  const uint32_t j = li % NLOG;

  log_t[j] = t_s * 1000.0f;
  log_e[j] = angle_err_deg;
  log_g[j] = gyro_dps;
  log_u[j] = u;
  log_v[j] = vel;
  log_x[j] = pos;

  li++;
  k++;

#if STREAM_SERIAL
  Serial.printf(
      "err=%.2f gyro=%.2f pos=%.3f vel=%.3f "
      "th=%.3f w=%.3f u=%.2f\n",
      angle_err_deg,
      gyro_dps,
      pos,
      vel,
      termoTheta,
      termoW,
      u
  );
#endif

  const uint32_t cyc = micros() - t_entrada;

  if (cyc > cyc_max) {
    cyc_max = cyc;
  }
}

// COMANDOS SERIAIS
void trataSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'm') {
      if (estado == SM_ESPERA_VERTICAL ||
          estado == SM_AUTOMATICO) {
        parar("comando_manual");
      }
    } else if (c == 'a' && estado == SM_PARADO) {
      Serial.println("Reiniciando...");
      delay(100);
      ESP.restart();
    }
  }
}

// INICIALIZACAO DO ROBO
bool inicializaRobo() {
  if (!iniciaI2cEMpu()) {
    Serial.println("[ERRO] Dispositivo nao respondeu em 0x68.");
    return false;
  }

  Serial.println("[DMP] Inicializando...");

  devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    Serial.printf("[ERRO] dmpInitialize() = %u\n", devStatus);
    return false;
  }

  Serial.println("[DMP] dmpInitialize() = 0.");
  Serial.println("Mantenha o sensor imovel durante a calibracao.");

  for (int i = 3; i > 0; i--) {
    Serial.printf("Calibracao em %d s...\n", i);

    digitalWrite(PIN_LED, HIGH);
    delay(500);

    digitalWrite(PIN_LED, LOW);
    delay(500);
  }

  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  mpu.setDMPEnabled(true);

#if USE_MPU_INT
  pinMode(PIN_MPU_INT, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT),
                  dmpDataReady, RISING);

  mpu.getIntStatus();
#endif

  packetSize = mpu.dmpGetFIFOPacketSize();

  Serial.printf("[DMP] Pacote = %u bytes\n", packetSize);

  if (packetSize == 0 || packetSize > sizeof(fifoBuffer)) {
    Serial.println("[ERRO] Tamanho de pacote invalido.");

    mpu.setDMPEnabled(false);
    return false;
  }

  uint32_t t_ini = millis();

  while ((uint32_t)(millis() - t_ini) < DMP_SETTLE_MS) {
    atualizaIMU(false);
    delay(1);
  }

  float a0 = 0.0f;
  float soma = 0.0f;
  int validas = 0;

  t_ini = millis();

  while (validas < REF_SAMPLES &&
         (uint32_t)(millis() - t_ini) < 3000) {
    if (atualizaIMU(false)) {
      float a = ANGLE_SIGN * dmp_roll_deg;

      if (validas == 0) {
        a0 = a;
      }

      soma += wrap180(a - a0);
      validas++;
    }

    delay(1);
  }

  if (validas == 0) {
    Serial.println("[ERRO] Nenhum pacote DMP valido.");

    mpu.setDMPEnabled(false);
    return false;
  }

  angle_ref = wrap180(a0 + soma / validas);

  Serial.printf("[DMP] Referencia Roll = %.2f graus\n", angle_ref);

  pinMode(PIN_ENC_ESQ_A, INPUT);
  pinMode(PIN_ENC_ESQ_B, INPUT);
  pinMode(PIN_ENC_DIR_A, INPUT);
  pinMode(PIN_ENC_DIR_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_ESQ_A),
                  isrEncEsq, RISING);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_DIR_A),
                  isrEncDir, RISING);

  return true;
}

// SETUP
void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  motorEsq.begin();
  motorDir.begin();

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== PENDULO INVERTIDO ESP32 ===");

  Serial.printf("Motores: %s\n",
                MOTORS_ENABLED ? "HABILITADOS" : "DESABILITADOS");

  if (digitalRead(PIN_BOOT) == LOW) {
    Serial.println("[CANCELADO] Botao BOOT pressionado.");

    digitalWrite(PIN_LED, HIGH);

    relatorio_feito = true;
    estado = SM_PARADO;

    return;
  }

  Serial.println("[BOOT] Aguardando 2 segundos...");
  delay(2000);

  if (!inicializaRobo()) {
    motorEsq.stop();
    motorDir.stop();

    digitalWrite(PIN_LED, LOW);

    strncpy(motivo_fim, "falha_na_inicializacao",
            sizeof(motivo_fim) - 1);

    estado = SM_PARADO;
    relatorio_feito = false;

    return;
  }

  Serial.println("[DMP] Inicializacao concluida.");
  Serial.println("[BOOT] Segure o sensor/robo na vertical.");

  ok_vertical = 0;
  t_espera_ms = millis();

  prox_controle_us = micros() + TS_US;
  ultimo_status_ms = millis();

  estado = SM_ESPERA_VERTICAL;
}

// LOOP
void loop() {
  trataSerial();

  if (estado == SM_ESPERA_VERTICAL ||
      estado == SM_AUTOMATICO) {
    atualizaIMU(false);

    uint32_t agora = micros();

    if ((int32_t)(agora - prox_controle_us) >= 0) {
      prox_controle_us += TS_US;

      if ((int32_t)(agora - prox_controle_us) >
          (int32_t)(5 * TS_US)) {
        prox_controle_us = agora + TS_US;
      }

      controle();
    }

    if ((uint32_t)(millis() - ultimo_status_ms) >= 1000) {
      ultimo_status_ms = millis();

      Serial.printf(
          "[STATUS] roll=%.2f gyroDMP=%.2f gyroDer=%.2f "
          "pk=%lu ovf=%lu\n",
          ANGLE_SIGN * dmp_roll_deg,
          GYRO_SIGN * dmp_gyro_dps,
          ANGLE_SIGN * dmp_gyro_deriv_dps,
          (unsigned long)dmp_pacotes,
          (unsigned long)fifo_overflow
      );
    }
  } else if (estado == SM_PARADO && !relatorio_feito) {
    relatorio();
  }
}
