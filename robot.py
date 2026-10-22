
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/pcnt.h"

// ============================================================
// FW2 - CHAO COM PROTECOES (EXECUTE SOMENTE APOS FW1 APROVADO)
// - Usar com cabo de seguranca (tether) e superficie de atrito.
// - Ganhos completos K_X..K_W do LQR, sem escala 0.925 (u = esforco).
// - Rampa 30 ms; zona morta suavizada (rampa 1.5 V) para manter o
//   ganho de pequeno sinal dentro da janela de ganho do laco.
// - Filtro de glitch em theta e velocidade; paradas de seguranca.
// - V3.1: encoder em quadratura 4x por HARDWARE (PCNT, sem interrupcao,
//   com filtro de glitch de 12.8 us), omega pelo giroscopio bruto,
//   watchdog de ciclo (para os motores se o laco ficar 40 ms parado).
// - V3.2: I2C com timeout curto em operacao (8 ms em vez de 100 ms),
//   deteccao de ciclos com I2C lento, omega mantida nesses ciclos e
//   parada se o I2C ficar instavel.
// - NAO reduza o ganho abaixo de ~0.8x: a margem inferior e 0.52x.
// ============================================================
#define AIR_TEST 1

// SEGURANCA (1 = motores ligados; use SOMENTE com robo suspenso)
#define MOTORS_ENABLED 1
#define USE_MPU_INT 0
#define GYRO_FROM_DMP 1
#define STREAM_SERIAL 0  // prints durante o controle estouram o ciclo (16 ms)

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

// CONTROLE: SEUS GANHOS
const float K_X = -6.9220f;
const float K_THETA = -22.0804f;
const float K_V = -14.3991f;
const float K_W = -1.8563f;

// Ganhos efetivos: no teste no ar, X e V sao zerados
const float KX_USE = AIR_TEST ? 0.0f : K_X;
const float KV_USE = AIR_TEST ? 0.0f : K_V;

const float U_LIMIT_V = 11.1f;
const float RAMP_S = 0.03f;
const float TRIM_DEG = 0.0f;  // V2: -1.5 era chute; medir o ponto de equilibrio real

#define WIN_V 4

const float TS = 0.01f;
const uint32_t TS_US = 10000;

// V2: sinal invertido apos teste no ar (realimentacao positiva via torque de reacao)
const float ANGLE_SIGN = -1.0f;
const float GYRO_SIGN = -1.0f;  // acompanha ANGLE_SIGN

// MOTORES
const float VBAT_V = 11.1f;
const float WHEEL_DIAMETER_M = 0.0683f;
const float PULSES_PER_REV = 836.0f;  // 209 (so borda de subida de A) x 4 (quadratura)

const float DIST_PER_PULSE =
    (PI * WHEEL_DIAMETER_M) / PULSES_PER_REV;

const float DEADBAND_V = 0.65f;
const float DEADBAND_RAMP_V = 1.5f;

#define PWM_FREQ_HZ 20000
#define PWM_BITS 10
#define PWM_MAX ((1 << PWM_BITS) - 1)

// IMU
#define I2C_FREQ_HZ 100000

const float DMP_GYRO_LSB_PER_DPS = 16.4f;
const uint32_t DMP_TIMEOUT_US = 50000;
const float ANGLE_FALL_LIMIT_DEG = 25.0f;

// Protecoes (FW2)
const float GLITCH_THETA_DEG = 8.0f;     // salto maximo de theta em 1 ciclo
const float VEL_MAX_MPS = 1.5f;          // fisico max ~1.77 m/s
const float VEL_GLITCH_MPS = 0.6f;       // salto maximo de vel em 1 ciclo
const int32_t ENC_MAX_DELTA = 120;       // contagens/ciclo; fisico max ~74 (530 rpm, 836 cont/rev, 10 ms)
const float POS_MAX_M = 1.0f;            // curso maximo de seguranca
const uint32_t SAT_MAX_CICLOS = 30;      // 300 ms em saturacao => para
const uint32_t VEL_ALTA_MAX_CICLOS = 50; // 500 ms acima de VEL_MAX
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

// WATCHDOG DE CICLO: se controle() nao roda por >40 ms (I2C travado, etc.)
// os motores sao parados por uma tarefa independente do loop().
volatile uint32_t ultimo_ciclo_us = 0;
volatile bool watchdog_ativo = false;
volatile bool watchdog_disparou = false;
const uint32_t WATCHDOG_US = 40000;

static void watchdogCb(void*) {
  if (watchdog_ativo &&
      (uint32_t)(micros() - ultimo_ciclo_us) > WATCHDOG_US) {
    motorEsq.stop();
    motorDir.stop();
    watchdog_disparou = true;
  }
}

// ENCODERS
volatile int32_t pulsosEsq = 0;
volatile int32_t pulsosDir = 0;

// NOVO: temporizador do log individual dos encoders
uint32_t ultimoLogEnc_ms = 0;

// Encoder em quadratura 4x pelo periférico PCNT do ESP32:
//  - sem interrupcao (a versao por ISR travava o laco quando o ruido do
//    PWM disparava dezenas de milhares de interrupcoes por segundo);
//  - filtro de glitch: pulsos < 1023 ciclos de APB (12.8 us) sao ignorados;
//    o encoder real gera no maximo ~7.4 kHz (pulso minimo > 130 us).
// Regra de contagem (mesma convencao anterior: esquerda +1 quando A sobe
// com B alto; direita com sinal oposto):
//  canal 0: pulso=A, controle=B (B baixo inverte)
//  canal 1: pulso=B, controle=A (A alto inverte)
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

// Chamar a cada ciclo (o contador de hardware e de 16 bits).
void atualizaEncoders() {
  int16_t ce = 0;
  int16_t cd = 0;

  pcnt_get_counter_value(PCNT_UNIT_0, &ce);
  pcnt_get_counter_value(PCNT_UNIT_1, &cd);

  const int16_t de = (int16_t)(ce - pcntUltEsq);
  const int16_t dd = (int16_t)(cd - pcntUltDir);

  pcntUltEsq = ce;
  pcntUltDir = cd;

  pulsosEsq = pulsosEsq + de;
  pulsosDir = pulsosDir - dd;  // direita: sinal oposto
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

// Estados do controlador
float theta = 0.0f;
float theta_ant = 0.0f;
float omega = 0.0f;
bool theta_inicializada = false;
uint32_t theta_t_prev_us = 0;

int32_t pe_prev = 0;
int32_t pd_prev = 0;

float pos = 0.0f;
float vel = 0.0f;
uint32_t enc_glitches = 0;  // rajadas de encoder rejeitadas

// Saude do I2C (V3.2). Normal: ~1.1 ms por ciclo (FIFO + giroscopio).
uint32_t i2c_max_us = 0;
uint32_t i2c_lentos = 0;
uint32_t i2c_consec = 0;
const uint32_t I2C_SUSPEITO_US = 3000;
const uint32_t I2C_CONSEC_MAX = 5;

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
float log_th[NLOG];   // theta (rad) usado no controle
float log_wd[NLOG];   // omega por derivada de theta (rad/s)
float log_wg[NLOG];   // omega pelo giroscopio DMP, na mesma convencao de theta (rad/s)
float log_es[NLOG];   // esforco apos clamp (antes da rampa)
float log_dt[NLOG];   // periodo real entre execucoes (ms)
float dt_ciclo_ms = 0.0f;

char motivo_fim[48] = "usuario";
uint32_t t_fim_us = 0;
bool relatorio_feito = false;
uint32_t ultimo_status_ms = 0;

// PARADA
void parar(const char* motivo) {
  watchdog_ativo = false;
  motorEsq.stop();
  motorDir.stop();

  strncpy(motivo_fim, motivo, sizeof(motivo_fim) - 1);
  motivo_fim[sizeof(motivo_fim) - 1] = '\0';

  t_fim_us = micros();
  estado = SM_PARADO;


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

  Serial.printf("TEMPO: dt_max=%.2f ms | ciclo_max=%lu us | ciclos>12ms: %lu\n",
                dt_max * 1000.0f,
                (unsigned long)cyc_max,
                (unsigned long)slow);

  Serial.printf("GLITCH encoder: %lu rajadas rejeitadas\n",
                (unsigned long)enc_glitches);

  Serial.printf("I2C: max=%lu us | ciclos>3ms=%lu\n",
                (unsigned long)i2c_max_us,
                (unsigned long)i2c_lentos);

  Serial.printf("MODO: %s | K_X_uso=%.3f K_V_uso=%.3f | PPR=%.0f\n",
                AIR_TEST ? "AR (X=V=0)" : "CHAO",
                KX_USE, KV_USE, PULSES_PER_REV);

  Serial.println("t_ms,dt_ms,err_deg,theta_rad,omega_diff,omega_gyro,esforco,u_V,vel_mps,pos_m");

  uint32_t n = (li < NLOG) ? li : NLOG;
  uint32_t inicio = li - n;

  for (uint32_t i = 0; i < n; i++) {
    uint32_t j = (inicio + i) % NLOG;

    Serial.printf("%.0f,%.2f,%.2f,%.4f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f\n",
                  log_t[j],
                  log_dt[j],
                  log_e[j],
                  log_th[j],
                  log_wd[j],
                  log_wg[j],
                  log_es[j],
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

  atualizaEncoders();

  pe_prev = pulsosEsq;
  pd_prev = pulsosDir;

  pos = 0.0f;
  vel = 0.0f;

  k = 0;
  li = 0;
  enc_glitches = 0;
  i2c_max_us = 0;
  i2c_lentos = 0;
  i2c_consec = 0;

  dt_max = 0.0f;
  cyc_max = 0;
  slow = 0;

  primeiro_ciclo = true;
  theta_inicializada = false;
  theta = 0.0f;
  theta_ant = 0.0f;
  omega = 0.0f;
  theta_t_prev_us = 0;

  t0_us = micros();

  for (int i = 0; i < WIN_V; i++) {
    hp[i] = 0.0f;
    ht[i] = t0_us;
  }

  digitalWrite(PIN_LED, HIGH);

  prox_controle_us = t0_us + TS_US;

  ultimo_ciclo_us = micros();
  watchdog_disparou = false;
  watchdog_ativo = true;

  estado = SM_AUTOMATICO;
}

// ROTINA DE CONTROLE
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

  // Giroscopio bruto, eixo X (+-2000 dps => 16.4 LSB/dps). Validado no
  // teste no ar: razao 1.015 com a derivada de theta, correlacao 0.99.
  const int16_t gx_raw = mpu.getRotationX();

  const uint32_t t_c = micros();

  // SAUDE DO I2C: tempo gasto em FIFO + giroscopio neste ciclo
  const uint32_t t_i2c_us = t_c - t_entrada;

  if (t_i2c_us > i2c_max_us) i2c_max_us = t_i2c_us;

  const bool i2c_suspeito = (t_i2c_us > I2C_SUSPEITO_US);

  if (i2c_suspeito) {
    i2c_lentos++;
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

    dt_ciclo_ms = dt * 1000.0f;

    if (dt > 0.05f) {
      char bufd[40];

      snprintf(bufd, sizeof(bufd), "ciclo_lento (%.0f ms)", dt * 1000.0f);

      parar(bufd);
      return;
    }
    if (dt > dt_max) dt_max = dt;
    if (dt > 0.012f) slow++;
  }

  primeiro_ciclo = false;
  t_entrada_ant_us = t_entrada;

  // GYRO ORIGINAL MANTIDO PARA DIAGNOSTICO
#if GYRO_FROM_DMP
  const float gyro_dps = GYRO_SIGN * dmp_gyro_dps;
#else
  const float gyro_dps = ANGLE_SIGN * dmp_gyro_deriv_dps;
#endif

  // ANGULO RELATIVO: MESMA CONVENCAO DO PROFESSOR
  // theta = -(roll - referencia), em radianos.
  const float angle_err_deg =
      wrap180(angulo - (angle_ref + TRIM_DEG));

  float theta_novo = -(angle_err_deg * (PI / 180.0f));

  // FILTRO DE GLITCH: rejeita salto impossivel de theta em 1 ciclo
  static float theta_valido = 0.0f;
  static bool theta_valido_ok = false;
  static uint32_t glitches_theta = 0;

  if (theta_valido_ok &&
      fabsf(theta_novo - theta_valido) >
          GLITCH_THETA_DEG * (PI / 180.0f)) {
    glitches_theta++;
    theta_novo = theta_valido;
  }

  theta = theta_novo;
  theta_valido = theta_novo;
  theta_valido_ok = true;

  // PROTECAO CONTRA QUEDA
  if (fabsf(angle_err_deg) > ANGLE_FALL_LIMIT_DEG) {
    char buf[40];

    snprintf(buf, sizeof(buf), "queda (%.1f deg)", angle_err_deg);

    parar(buf);
    return;
  }

  // POSICAO DOS ENCODERS
  atualizaEncoders();

  const int32_t pe = pulsosEsq;
  const int32_t pd = pulsosDir;

  // FILTRO DE RAJADA DO ENCODER: variacao acima do limite fisico por
  // ciclo e ruido (visto no teste no ar: +400 contagens em <100 ms).
  // Descarta no ORIGEM para nao integrar o erro em pos.
  int32_t dpe = pe - pe_prev;
  int32_t dpd = pd - pd_prev;

  if (abs(dpe) > ENC_MAX_DELTA) { dpe = 0; enc_glitches++; }
  if (abs(dpd) > ENC_MAX_DELTA) { dpd = 0; enc_glitches++; }

  pos += (dpe + dpd) * 0.5f * DIST_PER_PULSE;

  pe_prev = pe;
  pd_prev = pd;

  // VELOCIDADE LINEAR: VARIACAO DA POSICAO
  const uint32_t idx = k % WIN_V;

  if (k >= WIN_V) {
    const float span =
        (uint32_t)(t_c - ht[idx]) * 1e-6f;

    if (span > 0.0f) {
      float vel_nova = (pos - hp[idx]) / span;

      // rejeita salto de velocidade fisicamente impossivel
      if (fabsf(vel_nova - vel) > VEL_GLITCH_MPS) {
        vel_nova = vel;
      }

      vel = fmaxf(-VEL_MAX_MPS, fminf(VEL_MAX_MPS, vel_nova));
    }
  }

  hp[idx] = pos;
  ht[idx] = t_c;

  // VELOCIDADE ANGULAR: DERIVADA DO ANGULO
  // Usa o tempo real entre execucoes do controlador.
  if (!theta_inicializada) {
    theta_ant = theta;
    omega = 0.0f;
    theta_inicializada = true;
  } else {
    const uint32_t delta_us = t_entrada - theta_t_prev_us;
    const float dt_theta = delta_us * 1.0e-6f;

    if (dt_theta > 0.0f && dt_theta < 0.1f) {
      omega = (theta - theta_ant) / dt_theta;
    } else {
      omega = 0.0f;
    }

    theta_ant = theta;
  }

  theta_t_prev_us = t_entrada;

  // OMEGA USADA NO CONTROLE: giroscopio bruto (sem o atraso de 1 amostra
  // da derivada). A derivada continua calculada so para o log.
  const float omega_deriv = omega;
  omega = -GYRO_SIGN * (gx_raw / 16.4f) * (PI / 180.0f);

  // Ciclo com I2C lento: a leitura do giroscopio nao e confiavel;
  // mantem o ultimo valor bom.
  static float omega_ok = 0.0f;

  if (i2c_suspeito) {
    omega = omega_ok;
  } else {
    omega_ok = omega;
  }

  // RAMPA INICIAL
  const float t_s =
      (uint32_t)(t_c - t0_us) * 1e-6f;

  const float g_ramp =
      (RAMP_S <= 0.0f)
          ? 1.0f
          : fminf(1.0f, t_s / RAMP_S);

  // CONTROLADOR ESTATICO DE QUATRO ESTADOS
  // Mesma estrutura da equacao do professor, com seus ganhos.
  const float termoX = KX_USE * pos;
  const float termoTheta = K_THETA * theta;
  const float termoV = KV_USE * vel;
  const float termoW = K_W * omega;

  float esforco = -(termoX + termoTheta + termoV + termoW);

  // CONVERSAO DO ESFORCO PARA TENSAO
  esforco = fmaxf(-12.0f, fminf(12.0f, esforco));

  // Esforco do LQR ja esta em volts: sem escala 12 -> U_LIMIT_V
  float u = esforco;

  u *= g_ramp;

  // SATURACAO FINAL
  u = fmaxf(-U_LIMIT_V, fminf(U_LIMIT_V, u));

  // PARADAS DE SEGURANCA
  static uint32_t ciclos_sat = 0;
  static uint32_t ciclos_vel_alta = 0;

  ciclos_sat = (fabsf(u) >= U_LIMIT_V - 0.01f) ? ciclos_sat + 1 : 0;
  ciclos_vel_alta =
      (fabsf(vel) >= VEL_MAX_MPS - 0.01f) ? ciclos_vel_alta + 1 : 0;

  const char* falha = nullptr;

  // No teste no ar a roda gira livre e pos cresce sem limite: nao vale como curso.
  if (!AIR_TEST && fabsf(pos) > POS_MAX_M) falha = "curso_excedido";
  else if (ciclos_sat > SAT_MAX_CICLOS) falha = "saturacao_prolongada";
  else if (ciclos_vel_alta > VEL_ALTA_MAX_CICLOS) falha = "velocidade_alta";

  if (falha) {
    parar(falha);
    return;
  }

  // ACIONAMENTO DOS MOTORES
  // MOTORS_ENABLED permanece em 0.
  motorEsq.setVoltage(-u);
  motorDir.setVoltage(u);

  // LOGS
  const uint32_t j = li % NLOG;

  log_t[j] = t_s * 1000.0f;
  log_e[j] = angle_err_deg;
  log_g[j] = gyro_dps;
  log_u[j] = u;
  log_v[j] = vel;
  log_x[j] = pos;
  log_th[j] = theta;
  log_wd[j] = omega_deriv;
  // theta = -(roll - ref): o giroscopio equivalente tem sinal oposto
  log_wg[j] = omega;  // omega usada no controle (giroscopio bruto)
  log_es[j] = esforco;
  log_dt[j] = dt_ciclo_ms;

  li++;
  k++;

#if STREAM_SERIAL
  static uint8_t contadorSerial = 0;

  if (++contadorSerial >= 10) {
    contadorSerial = 0;

    Serial.printf(
        "err=%.2f theta=%.4f pos=%.3f vel=%.3f "
        "omega=%.4f tx=%.3f tth=%.3f tv=%.3f "
        "tw=%.3f esforco=%.2f u=%.2f\n",
        angle_err_deg,
        theta,
        pos,
        vel,
        omega,
        termoX,
        termoTheta,
        termoV,
        termoW,
        esforco,
        u
    );
  }
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

  pcntInit(PCNT_UNIT_0, PIN_ENC_ESQ_A, PIN_ENC_ESQ_B);
  pcntInit(PCNT_UNIT_1, PIN_ENC_DIR_A, PIN_ENC_DIR_B);

  // Em operacao um travamento do I2C deve custar poucos ms, nao 100 ms.
  Wire.setTimeOut(8);

  return true;
}

// SETUP
void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  motorEsq.begin();
  motorDir.begin();

  esp_timer_create_args_t wdArgs = {};
  wdArgs.callback = &watchdogCb;
  wdArgs.name = "wd_ciclo";
  esp_timer_handle_t wdTimer;
  esp_timer_create(&wdArgs, &wdTimer);
  esp_timer_start_periodic(wdTimer, 10000);

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
  ultimoLogEnc_ms = millis();

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

    if (estado == SM_ESPERA_VERTICAL &&
        (uint32_t)(millis() - ultimo_status_ms) >= 1000) {
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

  // NOVO: LEITURA INDIVIDUAL DOS ENCODERS
  // Imprime a contagem acumulada a cada 250 ms.
  if (estado == SM_ESPERA_VERTICAL &&
      (uint32_t)(millis() - ultimoLogEnc_ms) >= 250) {
    ultimoLogEnc_ms = millis();

    int32_t esq;
    int32_t dir;

    atualizaEncoders();

    noInterrupts();
    esq = pulsosEsq;
    dir = pulsosDir;
    interrupts();

    Serial.printf(
        "[ENCODERS] ESQ=%ld | DIR=%ld\n",
        (long)esq,
        (long)dir
    );
  }
}
