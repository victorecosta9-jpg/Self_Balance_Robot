#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "esp_timer.h"
#include "driver/pcnt.h"

// LEITURA BLE NATIVA DO ESP32
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================
// FW2 RESILIENTE - Velocidade Dinâmica + Curvas em Arco (Raio Ajustável)
// ============================================================

#define MOTORS_ENABLED 1

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

// PINOS
#define PIN_I2C_SCL 22
#define PIN_I2C_SDA 23
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

// CONTROLE: GANHOS LQR
const float K_X = -6.9220f;
const float K_THETA = -22.0804f;
const float K_V = -14.3991f;
const float K_W = -1.8563f;

// CONFIGURAÇÃO DE VELOCIDADE DINÂMICA E CURVAS
const float VEL_MAX_RAMPA = 1.80f;     // Velocidade máxima de cruzeiro (m/s)
const float ACEL_RAMPA = 2.50f;         // Aceleração da rampa (m/s²)
const float THETA_COMPENSACAO_DEG = 2.5f; // Inclinação intencional (feedforward) para aceleração rápida
const float PASSO_GIRO_BASE = 2.2f;     // Tensão de giro base [Volts]

const float U_LIMIT_V = 11.1f;
const float RAMP_S = 0.03f;
const float TRIM_DEG = 0.0f;

#define WIN_V 4

const float TS = 0.01f;
const uint32_t TS_US = 10000;

const float ANGLE_SIGN = -1.0f;
const float GYRO_SIGN = -1.0f;

const float VBAT_V = 11.1f;
const float WHEEL_DIAMETER_M = 0.0683f;
const float PULSES_PER_REV = 836.0f;
const float DIST_PER_PULSE = (PI * WHEEL_DIAMETER_M) / PULSES_PER_REV;

const float DEADBAND_V = 0.65f;
const float DEADBAND_RAMP_V = 1.5f;

#define PWM_FREQ_HZ 20000
#define PWM_BITS 10
#define PWM_MAX ((1 << PWM_BITS) - 1)

#define I2C_FREQ_HZ 400000

const uint32_t DMP_TIMEOUT_US = 100000;

const uint32_t EXT_MAX_CICLOS = 6;
const uint8_t GLITCH_REJ_MAX = 2;
const float OMEGA_MAX_RADS = 10.0f;
const float OMEGA_SALTO_RADS = 6.0f;
const float GLITCH_THETA_DEG = 8.0f;
const float VEL_MAX_MPS = 2.2f;
const float VEL_GLITCH_MPS = 0.8f;
const int32_t ENC_MAX_DELTA = 120;
const float POS_MAX_M = 10.0f; 
const uint32_t SAT_MAX_CICLOS = 30;
const uint32_t VEL_ALTA_MAX_CICLOS = 50;
const float START_WINDOW_DEG = 5.0f;
const int START_OK_SAMPLES = 20;
const uint32_t START_TIMEOUT_MS = 5000;
const uint32_t DMP_SETTLE_MS = 2000;
const int REF_SAMPLES = 100;

const uint32_t I2C_SUSPEITO_US = 4000;
const uint32_t I2C_CONSEC_MAX = 8;

const uint32_t WATCHDOG_US = 40000;

enum Estado { SM_INICIALIZANDO, SM_ESPERA_VERTICAL, SM_AUTOMATICO, SM_PARADO };
Estado estado = SM_INICIALIZANDO;

volatile float x_ref = 0.0f;
volatile float vel_alvo = 0.0f;          // Velocidade dinâmica atual
volatile float cmd_giro = 0.0f;
volatile int modo_movimento = 0;         // 0: Manter Posição, 1: Frente, 2: Ré, 3: Zero Absoluto
volatile uint32_t t_ultimo_pacote_ms = 0;

// CONTROLE DO TESTE DE QUEDA
volatile bool aguardando_queda_20deg = false;
uint32_t t_inicio_teste_ms = 0;

// ============================================================
// CALLBACKS BLE - PARSER DE COMANDOS COMBINADOS (CURVAS EM ARCO)
// ============================================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      modo_movimento = 0;
      cmd_giro = 0.0f;
      aguardando_queda_20deg = false;
      BLEDevice::startAdvertising();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      t_ultimo_pacote_ms = millis();

      uint8_t len = rxValue.length();
      uint8_t* data = (uint8_t*)rxValue.c_str();

      if (len >= 7 && data[0] == 0xFF) {
        uint8_t btn_forma = data[5]; // Botões geométricos (Quadrado, X)
        uint8_t btn_setas = data[6]; // Setas direcionais (Máscara de bits)

        // 1. CHECA BOTAO QUADRADO (TESTE DE QUEDA)
        if (btn_forma & 0x20) {
          aguardando_queda_20deg = true;
          Serial.println("\n--- INICIANDO TESTE: DEIXANDO CAIR ATE O ANGULO DEFINIDO ---");
        }
        // 2. CHECA BOTAO X (ZERO ABSOLUTO)
        else if (btn_forma & 0x10) {
          modo_movimento = 3;
          cmd_giro = 0.0f;
        } 
        // 3. PROCESSA COMBINAÇÕES DE SETAS (AVANÇO + GIRO SIMULTÂNEO)
        else {
          // Processamento do sentido longitudinal
          if (btn_setas & 0x01) {       // Cima
            modo_movimento = 1;
          } else if (btn_setas & 0x02) { // Baixo
            modo_movimento = 2;
          } else {
            modo_movimento = 0;
          }

          // Processamento do sentido de giro
          if (btn_setas & 0x04) {       // Esquerda
            cmd_giro = -PASSO_GIRO_BASE;
          } else if (btn_setas & 0x08) { // Direita
            cmd_giro = PASSO_GIRO_BASE;
          } else {
            cmd_giro = 0.0f;
          }
        }
      } 
      else if (len == 1) {
        char c = rxValue[0];
        if (c == 'S' || c == 'q') { aguardando_queda_20deg = true; }
        else if (c == 'F') { modo_movimento = 1; cmd_giro = 0.0f; }
        else if (c == 'B') { modo_movimento = 2; cmd_giro = 0.0f; }
        else if (c == 'L') { modo_movimento = 0; cmd_giro = -PASSO_GIRO_BASE; }
        else if (c == 'R') { modo_movimento = 0; cmd_giro = PASSO_GIRO_BASE; }
        else if (c == 'X') { modo_movimento = 3; cmd_giro = 0.0f; }
        else { modo_movimento = 0; cmd_giro = 0.0f; }
      }
    }
};

// ============================================================
// MOTOR
// ============================================================
class Motor {
 public:
  Motor(uint8_t pwm, uint8_t in1, uint8_t in2, uint8_t canal)
      : pwm_(pwm), in1_(in1), in2_(in2), canal_(canal) {}

  void begin() {
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
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
    const float a = fabsf(volts);

    if (a < 0.02f) {
      stop();
      return;
    }

    const float eff = DEADBAND_V * fminf(1.0f, a / DEADBAND_RAMP_V) +
                      (a / VBAT_V) * (VBAT_V - DEADBAND_V);

    if (volts > 0) {
      digitalWrite(in1_, HIGH);
      digitalWrite(in2_, LOW);
    } else {
      digitalWrite(in1_, LOW);
      digitalWrite(in2_, HIGH);
    }

    const float duty = fminf(1.0f, eff / VBAT_V);
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
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pwm_, d);
#else
    ledcWrite(canal_, d);
#endif
  }

  uint8_t pwm_, in1_, in2_, canal_;
};

Motor motorEsq(PIN_M_ESQ_PWM, PIN_M_ESQ_IN1, PIN_M_ESQ_IN2, 0);
Motor motorDir(PIN_M_DIR_PWM, PIN_M_DIR_IN1, PIN_M_DIR_IN2, 1);

// ============================================================
// WATCHDOG DE CICLO
// ============================================================
volatile uint32_t ultimo_ciclo_us = 0;
volatile bool watchdog_ativo = false;
volatile bool watchdog_disparou = false;

static void watchdogCb(void*) {
  if (watchdog_ativo && (uint32_t)(micros() - ultimo_ciclo_us) > WATCHDOG_US) {
    motorEsq.stop();
    motorDir.stop();
    watchdog_disparou = true;
  }
}

// ============================================================
// ENCODERS (PCNT)
// ============================================================
volatile int32_t pulsosEsq = 0;
volatile int32_t pulsosDir = 0;

static void pcntInit(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t c = {};

  c.pulse_gpio_num = pinA;
  c.ctrl_gpio_num = pinB;
  c.channel = PCNT_CHANNEL_0;
  c.unit = unit;
  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DEC;
  c.lctrl_mode = PCNT_MODE_REVERSE;
  c.hctrl_mode = PCNT_MODE_KEEP;
  c.counter_h_lim = 32767;
  c.counter_l_lim = -32768;
  pcnt_unit_config(&c);

  c.pulse_gpio_num = pinB;
  c.ctrl_gpio_num = pinA;
  c.channel = PCNT_CHANNEL_1;
  c.lctrl_mode = PCNT_MODE_KEEP;
  c.hctrl_mode = PCNT_MODE_REVERSE;
  pcnt_unit_config(&c);

  pcnt_set_filter_value(unit, 1023);
  pcnt_filter_enable(unit);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

static int16_t pcntUltEsq = 0;
static int16_t pcntUltDir = 0;

void atualizaEncoders() {
  int16_t ce = 0, cd = 0;

  pcnt_get_counter_value(PCNT_UNIT_0, &ce);
  pcnt_get_counter_value(PCNT_UNIT_1, &cd);

  const int16_t de = (int16_t)(ce - pcntUltEsq);
  const int16_t dd = (int16_t)(cd - pcntUltDir);

  pcntUltEsq = ce;
  pcntUltDir = cd;

  pulsosEsq = pulsosEsq + de;
  pulsosDir = pulsosDir - dd;
}

// ============================================================
// MPU6050 / DMP
// ============================================================
MPU6050 mpu;

uint8_t devStatus = 255;
uint16_t packetSize = 0;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

float dmp_roll_deg = 0.0f;
uint32_t dmp_ultimo_us = 0;
uint32_t dmp_pacotes = 0;
uint32_t t_poll_us = 0;

static inline float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

bool lerFIFO() {
  if (packetSize == 0) return false;

  uint16_t n = mpu.getFIFOCount();

  if (n >= 1024 || n < packetSize) {
    if (n >= 1024) mpu.resetFIFO();
    return false;
  }

  if (n >= (uint16_t)(packetSize * 3)) {
    mpu.resetFIFO();
    return false;
  }

  if (n >= (uint16_t)(packetSize * 2)) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    n -= packetSize;
  }

  const int8_t lidos = I2Cdev::readBytes(MPU6050_ADDRESS_AD0_LOW,
                                         MPU6050_RA_FIFO_R_W,
                                         (uint8_t)packetSize, fifoBuffer);

  if (lidos != (int8_t)packetSize) {
    mpu.resetFIFO();
    return false;
  }

  mpu.dmpGetQuaternion(&q, fifoBuffer);

  const float nq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

  if (nq < 0.95f || nq > 1.05f) {
    mpu.resetFIFO();
    return false;
  }

  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  dmp_roll_deg = ypr[2] * 180.0f / PI;
  dmp_ultimo_us = micros();
  dmp_pacotes++;

  return true;
}

bool atualizaIMU(bool forcar) {
  const uint32_t agora = micros();

  if (!forcar && (uint32_t)(agora - t_poll_us) < 1000) return false;

  t_poll_us = agora;
  return lerFIFO();
}

void i2cBusRecover() {
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
}

bool iniciaI2cEMpu() {
  delay(300);
  i2cBusRecover();

  for (int tentativa = 0; tentativa < 3; tentativa++) {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_FREQ_HZ);
    Wire.setTimeOut(2);

    mpu.initialize();

    Wire.beginTransmission(0x68);
    const uint8_t erro = Wire.endTransmission();

    if (erro == 0) return true;

    Serial.printf("[I2C] Sem ACK; codigo %u\n", erro);

    Wire.end();
    delay(100);
    i2cBusRecover();
  }

  return false;
}

// ============================================================
// ESTADO DO CONTROLADOR
// ============================================================
float angle_ref = 0.0f;

float theta = 0.0f;
float omega = 0.0f;
float omega_prev_ctrl = 0.0f;
float omega_ok = 0.0f;
uint8_t rej_omega = 0;

float theta_valido = 0.0f;
bool theta_valido_ok = false;
uint8_t rej_theta = 0;

int32_t pe_prev = 0;
int32_t pd_prev = 0;
float pos = 0.0f;
float vel = 0.0f;

float pos_esq = 0.0f, pos_dir = 0.0f;
float vel_esq = 0.0f, vel_dir = 0.0f;

float hp_esq[WIN_V], hp_dir[WIN_V];
uint32_t ht[WIN_V];
uint32_t k = 0;

uint32_t pacote_usado = 0;
uint32_t ciclos_ext = 0;
uint32_t i2c_consec = 0;

uint32_t ciclos_sat = 0;
uint32_t ciclos_vel_alta = 0;

uint32_t t0_us = 0;
uint32_t prox_controle_us = 0;
uint32_t t_entrada_ant_us = 0;
bool primeiro_ciclo = true;

int ok_vertical = 0;
uint32_t t_espera_ms = 0;

char motivo_fim[48] = "usuario";
bool fim_impresso = false;

// ============================================================
// PARADA
// ============================================================
void parar(const char* motivo) {
  watchdog_ativo = false;
  motorEsq.stop();
  motorDir.stop();

  strncpy(motivo_fim, motivo, sizeof(motivo_fim) - 1);
  motivo_fim[sizeof(motivo_fim) - 1] = '\0';

  estado = SM_PARADO;
  digitalWrite(PIN_LED, LOW);
}

// ============================================================
// INICIO DO CONTROLE
// ============================================================
void iniciaControle() {
  atualizaEncoders();

  pe_prev = pulsosEsq;
  pd_prev = pulsosDir;

  pos = 0.0f;
  pos_esq = 0.0f;
  pos_dir = 0.0f;
  x_ref = 0.0f; 
  vel_alvo = 0.0f;
  
  vel = 0.0f;
  vel_esq = 0.0f;
  vel_dir = 0.0f;
  k = 0;

  theta = 0.0f;
  omega = 0.0f;
  omega_prev_ctrl = 0.0f;
  omega_ok = 0.0f;
  rej_omega = 0;

  theta_valido_ok = false;
  rej_theta = 0;

  ciclos_ext = 0;
  pacote_usado = dmp_pacotes;
  i2c_consec = 0;

  ciclos_sat = 0;
  ciclos_vel_alta = 0;

  primeiro_ciclo = true;

  t0_us = micros();

  for (int i = 0; i < WIN_V; i++) {
    hp_esq[i] = 0.0f;
    hp_dir[i] = 0.0f;
    ht[i] = t0_us;
  }

  digitalWrite(PIN_LED, HIGH);

  prox_controle_us = t0_us + TS_US;

  ultimo_ciclo_us = micros();
  watchdog_disparou = false;
  watchdog_ativo = true;

  estado = SM_AUTOMATICO;
}

// ============================================================
// ROTINA DE CONTROLE (100 Hz)
// ============================================================
void controle() {
  const uint32_t t_entrada = micros();

  const uint32_t idade_ciclo_us = t_entrada - ultimo_ciclo_us;
  ultimo_ciclo_us = t_entrada;

  if (watchdog_disparou) {
    char bufw[48];
    snprintf(bufw, sizeof(bufw), "watchdog (%lu us sem ciclo)",
             (unsigned long)idade_ciclo_us);
    parar(bufw);
    return;
  }

  atualizaIMU(true);

  const int16_t gx_raw = mpu.getRotationX();

  const uint32_t t_c = micros();

  const bool i2c_suspeito = (t_c - t_entrada) > I2C_SUSPEITO_US;

  if (i2c_suspeito) {
    i2c_consec++;
  } else {
    i2c_consec = 0;
  }

  if (i2c_consec > I2C_CONSEC_MAX) {
    parar("i2c_instavel");
    return;
  }

  if (digitalRead(PIN_BOOT) == LOW) {
    parar("botao_emergencia");
    return;
  }

  if (dmp_pacotes == 0 || (uint32_t)(t_c - dmp_ultimo_us) > DMP_TIMEOUT_US) {
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
    } else if ((uint32_t)(millis() - t_espera_ms) > START_TIMEOUT_MS) {
      parar("timeout_vertical");
    }

    return;
  }

  if (!primeiro_ciclo) {
    const float dt = (uint32_t)(t_entrada - t_entrada_ant_us) * 1e-6f;

    if (dt > 0.05f) {
      parar("ciclo_lento");
      return;
    }
  }

  primeiro_ciclo = false;
  t_entrada_ant_us = t_entrada;

  const bool pacote_novo = (dmp_pacotes != pacote_usado);
  pacote_usado = dmp_pacotes;

  float theta_novo = -(wrap180(angulo - (angle_ref + TRIM_DEG)) * (PI / 180.0f));

  if (!pacote_novo) {
    ciclos_ext++;

    if (ciclos_ext >= 2) {
      mpu.resetFIFO();
    }

    if (ciclos_ext > EXT_MAX_CICLOS) {
      parar("imu_sem_pacote");
      return;
    }

    if (theta_valido_ok) {
      theta_novo = theta_valido + omega_prev_ctrl * TS;
    }
  } else {
    ciclos_ext = 0;
  }

  if (pacote_novo && theta_valido_ok && rej_theta < GLITCH_REJ_MAX &&
      fabsf(theta_novo - theta_valido) > GLITCH_THETA_DEG * (PI / 180.0f)) {
    rej_theta++;
    theta_novo = theta_valido + omega_prev_ctrl * TS;
  } else if (pacote_novo) {
    rej_theta = 0;
  }

  theta = theta_novo;
  theta_valido = theta_novo;
  theta_valido_ok = true;

  // KAIQUE, RETIRE ESSA INFORMAÇÃO PARA O MQTT: ÂNGULO EM GRAUS (theta_deg)
  const float theta_deg = theta * (180.0f / PI);

  // ============================================================
  // LOGICA DO TESTE DE QUEDA LIVRE (|theta_deg| >= 20.0f)
  // ============================================================
  if (aguardando_queda_20deg) {
    motorEsq.stop();
    motorDir.stop();

    Serial.printf("MODO_TESTE,%.2f\n", theta_deg);

    if (fabsf(theta_deg) >= 20.0f) {
      aguardando_queda_20deg = false;
      t_inicio_teste_ms = millis();
      x_ref = pos; 
      vel_alvo = 0.0f;
      Serial.println("--- ANGULO LIMITE ATINGIDO! LQR REASSUMINDO ---");
      Serial.println("Tempo_ms,Theta_deg,Omega_deg_s,Posicao_m,Velocidade_m_s,Esforco_V");
    }
    return;
  }

  atualizaEncoders();

  const int32_t pe = pulsosEsq;
  const int32_t pd = pulsosDir;

  int32_t dpe = pe - pe_prev;
  int32_t dpd = pd - pd_prev;

  if (abs(dpe) > ENC_MAX_DELTA) dpe = 0;
  if (abs(dpd) > ENC_MAX_DELTA) dpd = 0;

  pos_esq += dpe * DIST_PER_PULSE;
  pos_dir += dpd * DIST_PER_PULSE;

  // KAIQUE, RETIRE ESSA INFORMAÇÃO PARA O MQTT: POSIÇÃO LINEAR EM METROS (pos)
  pos = (pos_esq + pos_dir) * 0.5f;

  pe_prev = pe;
  pd_prev = pd;

  // WATCHDOG BLE
  if (millis() - t_ultimo_pacote_ms > 400) {
    modo_movimento = 0;
    cmd_giro = 0.0f;
  }

  // ============================================================
  // PERFIL DE VELOCIDADE DINÂMICA ADAPTATIVA (ACELERAÇÃO SUAVE E RÁPIDA)
  // ============================================================
  float v_destino = 0.0f;
  float theta_bias = 0.0f;

  if (modo_movimento == 1) {        // Avançar (Frente)
    v_destino = VEL_MAX_RAMPA;
    theta_bias = -(THETA_COMPENSACAO_DEG * PI / 180.0f); // Inclina pra frente para acelerar
  } else if (modo_movimento == 2) { // Recuar (Ré)
    v_destino = -VEL_MAX_RAMPA;
    theta_bias = (THETA_COMPENSACAO_DEG * PI / 180.0f);  // Inclina pra trás para acelerar
  } else if (modo_movimento == 3) { // Zero Absoluto (Botão X)
    v_destino = 0.0f;
    x_ref = 0.0f;
  } else {                          // Parado / Girando
    v_destino = 0.0f;
  }

  // Rampa de aceleração/desaceleração da velocidade
  if (vel_alvo < v_destino) {
    vel_alvo += ACEL_RAMPA * TS;
    if (vel_alvo > v_destino) vel_alvo = v_destino;
  } else if (vel_alvo > v_destino) {
    vel_alvo -= ACEL_RAMPA * TS;
    if (vel_alvo < v_destino) vel_alvo = v_destino;
  }

  // Atualização da posição de referência
  if (modo_movimento != 3) {
    if (modo_movimento == 0 && cmd_giro != 0.0f) {
      x_ref = pos; // Trava a posição no eixo ao girar parado
    } else {
      x_ref += vel_alvo * TS;
    }
  }

  const uint32_t idx = k % WIN_V;

  if (k >= WIN_V) {
    const float span = (uint32_t)(t_c - ht[idx]) * 1e-6f;

    if (span > 0.0f) {
      float ve_nova = (pos_esq - hp_esq[idx]) / span;
      float vd_nova = (pos_dir - hp_dir[idx]) / span;

      if (fabsf(ve_nova - vel_esq) > VEL_GLITCH_MPS) ve_nova = vel_esq;
      if (fabsf(vd_nova - vel_dir) > VEL_GLITCH_MPS) vd_nova = vel_dir;

      vel_esq = fmaxf(-VEL_MAX_MPS, fminf(VEL_MAX_MPS, ve_nova));
      vel_dir = fmaxf(-VEL_MAX_MPS, fminf(VEL_MAX_MPS, vd_nova));

      // KAIQUE, RETIRE ESSA INFORMAÇÃO PARA O MQTT: VELOCIDADE LINEAR EM M/S (vel)
      vel = (vel_esq + vel_dir) * 0.5f;
    }
  }

  hp_esq[idx] = pos_esq;
  hp_dir[idx] = pos_dir;
  ht[idx] = t_c;

  const float omega_nova = -GYRO_SIGN * (gx_raw / 16.4f) * (PI / 180.0f);

  const bool omega_suspeita =
      (fabsf(omega_nova) > OMEGA_MAX_RADS ||
       fabsf(omega_nova - omega_ok) > OMEGA_SALTO_RADS) &&
      rej_omega < GLITCH_REJ_MAX;

  if (i2c_suspeito || omega_suspeita) {
    omega = omega_ok;
    if (omega_suspeita) rej_omega++;
  } else {
    omega = omega_nova;
    omega_ok = omega;
    rej_omega = 0;
  }

  omega_prev_ctrl = omega;

  // KAIQUE, RETIRE ESSA INFORMAÇÃO PARA O MQTT: VELOCIDADE ANGULAR EM GRAUS/S (omega_deg_s)
  const float omega_deg_s = omega * (180.0f / PI);

  const float t_s = (uint32_t)(t_c - t0_us) * 1e-6f;
  const float g_ramp = (RAMP_S <= 0.0f) ? 1.0f : fminf(1.0f, t_s / RAMP_S);

  const float erro_pos = pos - x_ref;

  // CÁLCULO DO LQR COM FEEDFORWARD DE INCLINAÇÃO (theta - theta_bias)
  float esforco = -(K_X * erro_pos + K_THETA * (theta - theta_bias) + K_V * vel + K_W * omega);
  esforco = fmaxf(-12.0f, fminf(12.0f, esforco));

  // KAIQUE, RETIRE ESSA INFORMAÇÃO PARA O MQTT: ESFORÇO DE CONTROLE EM VOLTS (u)
  float u = esforco * g_ramp;
  u = fmaxf(-U_LIMIT_V, fminf(U_LIMIT_V, u));

  // KAIQUE, RETIRE ESSA INFORMAÇÃO PARA O MQTT: IMPRESSÃO DOS DADOS PARA O DASHBOARD / MATLAB
  if (t_inicio_teste_ms > 0) {
    uint32_t t_rel = millis() - t_inicio_teste_ms;
    Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f\n", t_rel, theta_deg, omega_deg_s, pos, vel, u);

    if (t_rel > 3000) {
      t_inicio_teste_ms = 0;
      Serial.println("--- TESTE CONCLUIDO ---");
    }
  }

  // ============================================================
  // AJUSTE DINÂMICO DE GIRO EM ARCO (AJUSTE AUTOMÁTICO DE RAIO)
  // ============================================================
  float u_yaw = cmd_giro;
  
  // Se o robô estiver em movimento, suaviza o giro para fazer curva aberta e estável
  if (fabsf(vel_alvo) > 0.1f) {
    u_yaw *= 0.65f; 
  }

  const float u_esq = -u - u_yaw;
  const float u_dir =  u - u_yaw;

  ciclos_sat = (fabsf(u) >= U_LIMIT_V - 0.01f) ? ciclos_sat + 1 : 0;
  ciclos_vel_alta = (fabsf(vel) >= VEL_MAX_MPS - 0.01f) ? ciclos_vel_alta + 1 : 0;

  const char* falha = nullptr;

  if (fabsf(erro_pos) > POS_MAX_M) falha = "curso_excedido";
  else if (ciclos_sat > SAT_MAX_CICLOS) falha = "saturacao_prolongada";
  else if (ciclos_vel_alta > VEL_ALTA_MAX_CICLOS) falha = "velocidade_alta";

  if (falha) {
    parar(falha);
    return;
  }

  motorEsq.setVoltage(u_esq);
  motorDir.setVoltage(u_dir);

  k++;
}

void trataSerial() {
  while (Serial.available() > 0) {
    const char c = Serial.read();

    if (c == 'm') {
      if (estado == SM_ESPERA_VERTICAL || estado == SM_AUTOMATICO) {
        parar("comando_manual");
      }
    } else if (c == 'a' && estado == SM_PARADO) {
      Serial.println("Reiniciando...");
      delay(100);
      ESP.restart();
    }
  }
}

bool inicializaRobo() {
  if (!iniciaI2cEMpu()) {
    Serial.println("[ERRO] Dispositivo nao respondeu em 0x68.");
    return false;
  }

  devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    Serial.printf("[ERRO] dmpInitialize() = %u\n", devStatus);
    return false;
  }

  Serial.println("Mantenha o sensor imovel durante a calibracao.");

  for (int i = 3; i > 0; i--) {
    digitalWrite(PIN_LED, HIGH);
    delay(500);
    digitalWrite(PIN_LED, LOW);
    delay(500);
  }

  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);

  packetSize = mpu.dmpGetFIFOPacketSize();

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

  while (validas < REF_SAMPLES && (uint32_t)(millis() - t_ini) < 3000) {
    if (atualizaIMU(false)) {
      const float a = ANGLE_SIGN * dmp_roll_deg;

      if (validas == 0) a0 = a;

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

  pcntInit(PCNT_UNIT_0, PIN_ENC_ESQ_A, PIN_ENC_ESQ_B);
  pcntInit(PCNT_UNIT_1, PIN_ENC_DIR_A, PIN_ENC_DIR_B);

  return true;
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  motorEsq.begin();
  motorDir.begin();

  Serial.begin(115200);
  delay(500);

  BLEDevice::init("Robo_Pendulo_ESP32");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_TX,
                       BLECharacteristic::PROPERTY_NOTIFY
                     );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE
                                         );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println();
  Serial.printf("=== PENDULO INVERTIDO ESP32 | Velocidade Dinamica + Curvas em Arco ===\n");

  if (digitalRead(PIN_BOOT) == LOW) {
    Serial.println("[CANCELADO] Botao BOOT pressionado.");
    digitalWrite(PIN_LED, HIGH);
    fim_impresso = true;
    estado = SM_PARADO;
    return;
  }

  delay(2000);

  if (!inicializaRobo()) {
    motorEsq.stop();
    motorDir.stop();
    digitalWrite(PIN_LED, LOW);
    strncpy(motivo_fim, "falha_na_inicializacao", sizeof(motivo_fim) - 1);
    estado = SM_PARADO;
    return;
  }

  esp_timer_create_args_t wdArgs = {};
  wdArgs.callback = &watchdogCb;
  wdArgs.name = "wd_ciclo";
  esp_timer_handle_t wdTimer;
  esp_timer_start_periodic(wdTimer, 10000);

  Serial.println("[BOOT] Segure o robo na vertical.");

  ok_vertical = 0;
  t_espera_ms = millis();
  prox_controle_us = micros() + TS_US;

  estado = SM_ESPERA_VERTICAL;
}

void loop() {
  trataSerial();

  if (estado == SM_ESPERA_VERTICAL || estado == SM_AUTOMATICO) {
    const uint32_t agora = micros();

    atualizaIMU(false);

    if ((int32_t)(agora - prox_controle_us) >= 0) {
      prox_controle_us += TS_US;

      if ((int32_t)(agora - prox_controle_us) > (int32_t)(5 * TS_US)) {
        prox_controle_us = agora + TS_US;
      }

      controle();
    }
  } else if (estado == SM_PARADO && !fim_impresso) {
    fim_impresso = true;
    Serial.printf("FIM: %s\n", motivo_fim);
  }
}
