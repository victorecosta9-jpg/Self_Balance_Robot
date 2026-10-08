/*
  ======================================================================
  PENDULO INVERTIDO - ESP32 Dev Kit Module (Arduino IDE)
  ======================================================================
  Port do main.py + robot.py (MicroPython) para Arduino, usando como
  referencia a estrutura do codigo do Renan:

    - Aquisicao via DMP do MPU6050 (FIFO, pacote de 42 bytes, 100 Hz)
    - Controle em 100 Hz (10 ms)
    - Quaternion -> gravidade -> yaw/pitch/roll (angulo = roll, eixo X)

  Removido em relacao ao codigo do Renan:
    - Observador de estados (matrizes A, B, C, L, TinyMatrixMath)
    - Cadeira de rodas: servo, estagios sentado/em pe, swing-down
    - Modos de movimento (frente/tras/curvas) e PIDs de sincronismo

  Mantido do robot.py:
    - Lei de controle  u = -(Kx*pos + Kth*theta + Kv*vel + Kw*w)
    - Rampa de partida, trim, saturacao em 11.1 V
    - Compensacao de zona morta do motor, ponte H (PWM + IN1/IN2)
    - Encoders em quadratura simplificada (borda de subida do canal A)
    - Espera pela vertical antes de ativar, protecao de queda (35 graus)
    - Log em RAM e despejo em CSV ao final
    - Recuperacao do barramento I2C somente na inicializacao
    - Trava de emergencia do main.py (botao BOOT / GPIO 0)

  Bibliotecas (Gerenciador de Bibliotecas do Arduino IDE):
    - "MPU6050" de Electronic Cats (traz I2Cdev e MotionApps20)
  Placa: "ESP32 Dev Module"  (core esp32 da Espressif 2.x ou 3.x)

  Comandos pela Serial (115200 baud):
    m  -> para os motores (modo manual)
    a  -> reinicia a sequencia (equivale a um reset)
  ======================================================================
*/

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "driver/gpio.h"

// ======================================================================
// OPCOES DE COMPILACAO
// ======================================================================

// 1 = le o DMP quando o pino INT do MPU6050 sinaliza (como no Renan)
// 0 = faz polling do FIFO a cada 1 ms (nao precisa do fio de INT)
#define USE_MPU_INT      0

// 1 = velocidade angular vem do giroscopio dentro do pacote do DMP
// 0 = velocidade angular por derivada do angulo (como o Renan faz)
#define GYRO_FROM_DMP    1

// 1 = transmite uma linha CSV por ciclo de controle (como o enviar_serial
//     do Renan). 0 = so loga na RAM e despeja ao final (como o robot.py)
#define STREAM_SERIAL    0

// ======================================================================
// PINOS (iguais ao robot.py)
// ======================================================================

#define PIN_I2C_SCL      22
#define PIN_I2C_SDA      23
#define PIN_MPU_INT      4      // so e usado se USE_MPU_INT = 1

#define PIN_LED          2
#define PIN_BOOT         0      // botao BOOT = trava fisica de emergencia

#define PIN_M_ESQ_PWM    25
#define PIN_M_ESQ_IN1    27
#define PIN_M_ESQ_IN2    26

#define PIN_M_DIR_PWM    14
#define PIN_M_DIR_IN1    13
#define PIN_M_DIR_IN2    12

#define PIN_ENC_ESQ_A    33
#define PIN_ENC_ESQ_B    32
#define PIN_ENC_DIR_A    35
#define PIN_ENC_DIR_B    34

// ======================================================================
// CONTROLADOR (mesmos valores do robot.py)
// ======================================================================

const float K_X        =  0.0f;
const float K_THETA    = -30.0f;
const float K_V        =  0.0f;
const float K_W        = -0.5f;

const float U_LIMIT_V  = 11.1f;
const float RAMP_S     = 0.15f;
const float TRIM_DEG   = -1.5f;

// Janela da velocidade. No robot.py eram 10 amostras a 250 Hz (~40 ms).
// A 100 Hz, 4 amostras mantem os mesmos ~40 ms.
#define WIN_V            4

// Controle e aquisicao = 100 Hz (nos moldes do Renan)
const float    TS        = 0.01f;
const uint32_t TS_US     = 10000;

// Sinais. Se o robo "fugir" para o lado errado ou o amortecimento
// parecer positivo, inverta aqui (+1 / -1).
const float ANGLE_SIGN   = 1.0f;   // angulo = ANGLE_SIGN * roll do DMP
const float GYRO_SIGN    = 1.0f;   // gyro   = GYRO_SIGN  * gx do DMP

// ======================================================================
// PARAMETROS MECANICOS E ELETRICOS
// ======================================================================

const float VBAT_V             = 11.1f;
const float WHEEL_DIAMETER_M   = 0.0683f;
const float PULSES_PER_REV     = 700.0f;
const float DIST_PER_PULSE     = (PI * WHEEL_DIAMETER_M) / PULSES_PER_REV;

const float DEADBAND_V         = 0.65f;
const float DEADBAND_RAMP_V    = 0.2f;

#define PWM_FREQ_HZ      20000
#define PWM_BITS         10
#define PWM_MAX          ((1 << PWM_BITS) - 1)

// ======================================================================
// IMU / PROTECOES
// ======================================================================

#define I2C_FREQ_HZ      400000

// O DMP configura o giroscopio em +/-2000 dps -> 16.4 LSB/(grau/s)
const float DMP_GYRO_LSB_PER_DPS = 16.4f;

// Sem pacote novo do DMP por mais que isso -> para (equivale aos 10 holds
// de 4 ms do robot.py, ~40 ms)
const uint32_t DMP_TIMEOUT_US = 50000;

const float ANGLE_FALL_LIMIT_DEG = 35.0f;
const float START_WINDOW_DEG     = 5.0f;
const int   START_OK_SAMPLES     = 20;
const uint32_t START_TIMEOUT_MS  = 5000;

const uint32_t DMP_SETTLE_MS     = 2000;   // descarta pacotes iniciais
const int      REF_SAMPLES       = 100;    // media do angulo de referencia

// ======================================================================
// LOG
// ======================================================================

#define NLOG 1500   // 15 s a 100 Hz

float log_t[NLOG];
float log_e[NLOG];
float log_g[NLOG];
float log_u[NLOG];
float log_v[NLOG];
float log_x[NLOG];

// ======================================================================
// MAQUINA DE ESTADOS (a do Renan, sem os estados da cadeira)
// ======================================================================

typedef enum {
  SM_INICIALIZANDO,
  SM_ESPERA_VERTICAL,
  SM_AUTOMATICO,
  SM_PARADO
} SM_SM;

SM_SM estado = SM_INICIALIZANDO;

// ======================================================================
// MOTOR
// ======================================================================

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
    volts = fmaxf(-VBAT_V, fminf(VBAT_V, volts));
    float a = fabsf(volts);

    if (a < 0.02f) {
      stop();
      return;
    }

    // Compensacao da zona morta (igual ao robot.py)
    float eff = DEADBAND_V * fminf(1.0f, a / DEADBAND_RAMP_V)
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

// ======================================================================
// ENCODERS
// ======================================================================

volatile int32_t pulsosEsq = 0;
volatile int32_t pulsosDir = 0;

void IRAM_ATTR isrEncEsq() {
  if (gpio_get_level((gpio_num_t)PIN_ENC_ESQ_B)) pulsosEsq = pulsosEsq + 1;
  else                                           pulsosEsq = pulsosEsq - 1;
}

void IRAM_ATTR isrEncDir() {
  if (gpio_get_level((gpio_num_t)PIN_ENC_DIR_B)) pulsosDir = pulsosDir - 1;
  else                                           pulsosDir = pulsosDir + 1;
}

// ======================================================================
// MPU6050 + DMP
// ======================================================================

MPU6050 mpu;

uint8_t  devStatus;
uint16_t packetSize;
uint8_t  fifoBuffer[64];

Quaternion  q;
VectorFloat gravity;
float       ypr[3];

#if USE_MPU_INT
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }
#endif

// Ultima amostra valida do DMP
float    dmp_roll_deg       = 0.0f;   // roll bruto (graus)
float    dmp_gyro_dps       = 0.0f;   // gx do pacote do DMP (graus/s)
float    dmp_gyro_deriv_dps = 0.0f;   // d(roll)/dt (graus/s)
float    dmp_roll_ant       = 0.0f;
uint32_t dmp_ultimo_us      = 0;
uint32_t dmp_pacotes        = 0;
uint32_t fifo_overflow      = 0;
uint32_t t_poll_us          = 0;

static inline float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// Le o FIFO do DMP sem bloquear. Descarta pacotes velhos e fica com o
// mais recente. Retorna true se chegou amostra nova.
bool lerFIFO() {
  uint16_t n = mpu.getFIFOCount();

  if (n >= 1024) {                 // overflow (igual ao Renan)
    mpu.resetFIFO();
    fifo_overflow++;
    return false;
  }
  if (n < packetSize) return false;

  while (n >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    n -= packetSize;
  }

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  int16_t g[3];
  mpu.dmpGetGyro(g, fifoBuffer);

  float roll = ypr[2] * 180.0f / PI;

  if (dmp_pacotes > 0) {
    dmp_gyro_deriv_dps = wrap180(roll - dmp_roll_ant) / TS;
  }
  dmp_roll_ant = roll;

  dmp_roll_deg  = roll;
  dmp_gyro_dps  = (float)g[0] / DMP_GYRO_LSB_PER_DPS;
  dmp_ultimo_us = micros();
  dmp_pacotes++;
  return true;
}

// Chamada a cada volta do loop (forcar = false) e no inicio de cada
// ciclo de controle (forcar = true).
bool atualizaIMU(bool forcar) {
#if USE_MPU_INT
  if (mpuInterrupt) {
    mpuInterrupt = false;
    uint8_t st = mpu.getIntStatus();
    if (st & 0x10) {               // overflow do FIFO
      mpu.resetFIFO();
      fifo_overflow++;
      return false;
    }
  } else if (!forcar) {
    return false;
  }
#else
  uint32_t agora = micros();
  if (!forcar && (uint32_t)(agora - t_poll_us) < 1000) return false;
  t_poll_us = agora;
#endif
  return lerFIFO();
}

// Liberacao do barramento (somente na inicializacao, como no robot.py)
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

  // STOP
  digitalWrite(PIN_I2C_SCL, LOW);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_I2C_SDA, HIGH);
  delayMicroseconds(10);
  Serial.println("[I2C] Barramento liberado.");
}

bool iniciaI2cEMpu() {
  delay(300);
  i2cBusRecover();

  for (int tentativa = 0; tentativa < 3; tentativa++) {
    Serial.printf("[I2C] Inicializacao %d/3\n", tentativa + 1);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_FREQ_HZ);
    Wire.setTimeOut(20);

    mpu.initialize();
    if (mpu.testConnection()) {
      Serial.println("[IMU] MPU6050 conectado.");
      return true;
    }

    Serial.println("[IMU] MPU6050 nao respondeu.");
    Wire.end();
    delay(100);
    i2cBusRecover();
  }
  return false;
}

// ======================================================================
// ESTADO DO CONTROLE
// ======================================================================

float    angle_ref = 0.0f;

int32_t  pe_prev = 0, pd_prev = 0;
float    pos = 0.0f, vel = 0.0f;
uint32_t k = 0;                      // amostras em SM_AUTOMATICO
uint32_t li = 0;                     // indice do log

float    hp[WIN_V];
uint32_t ht[WIN_V];

uint32_t t0_us = 0;                  // inicio do controle
uint32_t prox_controle_us = 0;

int      ok_vertical = 0;
uint32_t t_espera_ms = 0;

// Diagnostico
float    dt_max = 0.0f;
uint32_t cyc_max = 0;
uint32_t slow = 0;
uint32_t t_entrada_ant_us = 0;
bool     primeiro_ciclo = true;

char     motivo_fim[48] = "usuario";
uint32_t t_fim_us = 0;
bool     relatorio_feito = false;

uint32_t ultimo_status_ms = 0;

// ======================================================================
// PARADA
// ======================================================================

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

void relatorio() {
  relatorio_feito = true;

  Serial.println();
  Serial.println("==================================================");
  Serial.printf(" FIM: %s\n", motivo_fim);
  Serial.println("==================================================");

  if (li == 0) return;

  Serial.printf("Amostras: %lu | duracao ~%.2f s\n",
                (unsigned long)li, (t_fim_us - t0_us) * 1e-6f);
  Serial.printf("Intervalo maximo entre ciclos: %.2f ms\n", dt_max * 1000.0f);
  Serial.printf("Tempo maximo de processamento: %lu us\n", (unsigned long)cyc_max);
  Serial.printf("Ciclos > 12 ms: %lu\n", (unsigned long)slow);
  Serial.printf("Pacotes DMP: %lu | overflow FIFO: %lu\n",
                (unsigned long)dmp_pacotes, (unsigned long)fifo_overflow);

  uint32_t n = (li < NLOG) ? li : NLOG;
  uint32_t inicio = li - n;

  Serial.println();
  Serial.printf("LOG (CSV, ultimas %lu amostras):\n", (unsigned long)n);
  Serial.println("t_ms,err_deg,gyro_dps,u_V,vel_mps,pos_m");

  for (uint32_t i = 0; i < n; i++) {
    uint32_t j = (inicio + i) % NLOG;
    Serial.printf("%.0f,%.2f,%.1f,%.2f,%.3f,%.3f\n",
                  log_t[j], log_e[j], log_g[j], log_u[j], log_v[j], log_x[j]);
  }
}

// ======================================================================
// INICIO DO CONTROLE (apos achar a vertical)
// ======================================================================

void iniciaControle() {
  Serial.println();
  Serial.println("==================================================");
  Serial.println(" GO! Controle ativado.");
  Serial.println("==================================================");
  Serial.printf("Kx     = %.4f\n", K_X);
  Serial.printf("Ktheta = %.4f\n", K_THETA);
  Serial.printf("Kv     = %.4f\n", K_V);
  Serial.printf("Kw     = %.4f\n", K_W);
  Serial.printf("RAMP_S = %.3f s\n", RAMP_S);
  Serial.printf("Controle/DMP = %d Hz\n", (int)(1.0f / TS));
  Serial.printf("Gyro = %s\n", GYRO_FROM_DMP ? "DMP" : "derivada do angulo");

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

// ======================================================================
// CONTROLE (100 Hz)
// ======================================================================

void controle() {
  const uint32_t t_entrada = micros();

  atualizaIMU(true);

  const uint32_t t_c = micros();

  // Botao BOOT = parada de emergencia
  if (digitalRead(PIN_BOOT) == LOW) {
    parar("botao_emergencia");
    return;
  }

  // Dados do DMP precisam estar frescos
  if ((uint32_t)(t_c - dmp_ultimo_us) > DMP_TIMEOUT_US) {
    parar("imu_sem_dados");
    return;
  }

  const float angulo = ANGLE_SIGN * dmp_roll_deg;

  // --------------------------------------------------------------
  // Espera a vertical
  // --------------------------------------------------------------
  if (estado == SM_ESPERA_VERTICAL) {
    if (fabsf(wrap180(angulo - angle_ref)) < START_WINDOW_DEG) ok_vertical++;
    else                                                       ok_vertical = 0;

    if (ok_vertical >= START_OK_SAMPLES) {
      iniciaControle();
    } else if ((uint32_t)(millis() - t_espera_ms) > START_TIMEOUT_MS) {
      Serial.println("Timeout esperando a vertical.");
      parar("timeout_vertical");
    }
    return;
  }

  // --------------------------------------------------------------
  // SM_AUTOMATICO
  // --------------------------------------------------------------

  // Diagnostico do periodo do ciclo
  if (!primeiro_ciclo) {
    float dt = (uint32_t)(t_entrada - t_entrada_ant_us) * 1e-6f;
    if (dt > dt_max) dt_max = dt;
    if (dt > 0.012f) slow++;
  }
  primeiro_ciclo = false;
  t_entrada_ant_us = t_entrada;

  // Velocidade angular
#if GYRO_FROM_DMP
  const float gyro_dps = GYRO_SIGN * dmp_gyro_dps;
#else
  const float gyro_dps = ANGLE_SIGN * dmp_gyro_deriv_dps;
#endif

  // Erro angular
  const float angle_err_deg = wrap180(angulo - (angle_ref + TRIM_DEG));

  // Protecao de queda
  if (fabsf(angle_err_deg) > ANGLE_FALL_LIMIT_DEG) {
    char buf[40];
    snprintf(buf, sizeof(buf), "queda (%.1f deg)", angle_err_deg);
    parar(buf);
    return;
  }

  // Encoders
  const int32_t pe = pulsosEsq;
  const int32_t pd = pulsosDir;

  pos += ((pe - pe_prev) + (pd - pd_prev)) * 0.5f * DIST_PER_PULSE;
  pe_prev = pe;
  pd_prev = pd;

  // Velocidade (janela deslizante)
  const uint32_t idx = k % WIN_V;
  if (k >= WIN_V) {
    float span = (uint32_t)(t_c - ht[idx]) * 1e-6f;
    if (span > 0.0f) vel = (pos - hp[idx]) / span;
  }
  hp[idx] = pos;
  ht[idx] = t_c;

  // Tempo desde o GO
  const float t_s = (uint32_t)(t_c - t0_us) * 1e-6f;

  // Rampa
  const float g_ramp = (RAMP_S <= 0.0f) ? 1.0f : fminf(1.0f, t_s / RAMP_S);

  // Lei de controle (identica ao robot.py)
  float u = -( K_X * pos
             + K_THETA * (angle_err_deg * (PI / 180.0f))
             + K_V * vel
             + K_W * (gyro_dps * (PI / 180.0f)) );

  u *= g_ramp;
  u = fmaxf(-U_LIMIT_V, fminf(U_LIMIT_V, u));

  // Motores (esquerdo invertido, como no robot.py)
  motorEsq.setVoltage(-u);
  motorDir.setVoltage(u);

  // Log
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
  Serial.printf("%.2f,%.2f,%.3f,%.3f,%.2f\n", angle_err_deg, gyro_dps, pos, vel, u);
#endif

  const uint32_t cyc = micros() - t_entrada;
  if (cyc > cyc_max) cyc_max = cyc;
}

// ======================================================================
// COMANDOS PELA SERIAL
// ======================================================================

void trataSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'm') {
      if (estado == SM_ESPERA_VERTICAL || estado == SM_AUTOMATICO) {
        Serial.println("MANUAL");
        parar("comando_manual");
      }
    } else if (c == 'a') {
      if (estado == SM_PARADO) {
        Serial.println("Reiniciando...");
        delay(100);
        ESP.restart();
      }
    }
  }
}

// ======================================================================
// SEQUENCIA DE PARTIDA (robot.py: run())
// ======================================================================

bool inicializaRobo() {

  // ---------------- IMU ----------------
  if (!iniciaI2cEMpu()) {
    Serial.println("[ERRO] MPU6050 nao respondeu.");
    return false;
  }

  Serial.println("[DMP] Inicializando DMP...");
  devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    Serial.printf("[ERRO] Falha ao iniciar o DMP (codigo %d)\n", devStatus);
    return false;
  }

  // ---------------- Calibracao ----------------
  Serial.println();
  Serial.println("--------------------------------------------------");
  Serial.println(" MANTENHA O ROBO PARADO NA VERTICAL");
  Serial.println("--------------------------------------------------");

  for (int i = 3; i > 0; i--) {
    Serial.printf("Calibrando em %ds...\n", i);
    digitalWrite(PIN_LED, HIGH);
    delay(500);
    digitalWrite(PIN_LED, LOW);
    delay(500);
  }

  Serial.println("[IMU] Calibrando gyro...");
  mpu.CalibrateGyro(6);           // offsets de hardware do giroscopio
  mpu.PrintActiveOffsets();

  // ---------------- Liga o DMP ----------------
  mpu.setDMPEnabled(true);

#if USE_MPU_INT
  pinMode(PIN_MPU_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), dmpDataReady, RISING);
  mpu.getIntStatus();
#endif

  packetSize = mpu.dmpGetFIFOPacketSize();
  Serial.printf("[DMP] Pronto. Pacote = %u bytes\n", packetSize);

  // Descarta os pacotes iniciais (DMP ainda convergindo)
  uint32_t t_ini = millis();
  while ((uint32_t)(millis() - t_ini) < DMP_SETTLE_MS) {
    atualizaIMU(false);
  }

  // ---------------- Referencia angular ----------------
  float a0 = 0.0f;
  float soma = 0.0f;
  int   validas = 0;

  t_ini = millis();
  while (validas < REF_SAMPLES && (uint32_t)(millis() - t_ini) < 3000) {
    if (atualizaIMU(false)) {
      float a = ANGLE_SIGN * dmp_roll_deg;
      if (validas == 0) a0 = a;
      soma += wrap180(a - a0);
      validas++;
    }
  }

  if (validas == 0) {
    Serial.println("[ERRO] Nenhuma leitura valida do DMP.");
    return false;
  }

  angle_ref = wrap180(a0 + soma / validas);
  Serial.printf("Referencia = %.2f deg\n", angle_ref);

  // ---------------- Encoders ----------------
  pinMode(PIN_ENC_ESQ_A, INPUT);
  pinMode(PIN_ENC_ESQ_B, INPUT);
  pinMode(PIN_ENC_DIR_A, INPUT);
  pinMode(PIN_ENC_DIR_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_ESQ_A), isrEncEsq, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_DIR_A), isrEncDir, RISING);

  return true;
}

// ======================================================================
// SETUP / LOOP
// ======================================================================

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  // Motores sempre comecam parados
  motorEsq.begin();
  motorDir.begin();

  Serial.begin(115200);
  delay(200);

  // ---- trava de emergencia (main.py) ----
  for (int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(50);
    digitalWrite(PIN_LED, LOW);
    delay(50);
  }

  Serial.println();
  Serial.println("==================================================");
  Serial.println("[BOOT] Checando botao de emergencia (GPIO 0)...");
  Serial.println("==================================================");

  if (digitalRead(PIN_BOOT) == LOW) {
    Serial.println();
    Serial.println("[CANCELADO] Botao BOOT pressionado! Robo desativado.");
    digitalWrite(PIN_LED, HIGH);
    relatorio_feito = true;
    estado = SM_PARADO;
    return;
  }

  Serial.println("[BOOT] Botao solto. Aguardando 2s para iniciar o robo...");
  delay(2000);

  if (!inicializaRobo()) {
    motorEsq.stop();
    motorDir.stop();
    digitalWrite(PIN_LED, LOW);
    snprintf(motivo_fim, sizeof(motivo_fim), "falha_na_inicializacao");
    estado = SM_PARADO;
    return;
  }

  // Vai para a espera da vertical; o controle assume a 100 Hz
  Serial.println("Segure o robo na vertical...");
  ok_vertical = 0;
  t_espera_ms = millis();
  prox_controle_us = micros() + TS_US;
  ultimo_status_ms = millis();
  estado = SM_ESPERA_VERTICAL;
}

void loop() {
  trataSerial();

  if (estado == SM_ESPERA_VERTICAL || estado == SM_AUTOMATICO) {

    // Aquisicao: drena o FIFO do DMP (pacotes de 100 Hz)
    atualizaIMU(false);

    // Controle: base de tempo de 10 ms (100 Hz)
    uint32_t agora = micros();
    if ((int32_t)(agora - prox_controle_us) >= 0) {
      prox_controle_us += TS_US;
      if ((int32_t)(agora - prox_controle_us) > (int32_t)(5 * TS_US)) {
        prox_controle_us = agora + TS_US;     // atrasou demais: ressincroniza
      }
      controle();
    }

    // Status 1 Hz
    if ((uint32_t)(millis() - ultimo_status_ms) >= 1000) {
      ultimo_status_ms = millis();
      Serial.printf("[STATUS] ang=%.2f gyroDMP=%.1f gyroDer=%.1f pk=%lu ovf=%lu\n",
                    ANGLE_SIGN * dmp_roll_deg, GYRO_SIGN * dmp_gyro_dps,
                    ANGLE_SIGN * dmp_gyro_deriv_dps,
                    (unsigned long)dmp_pacotes, (unsigned long)fifo_overflow);
    }

  } else if (estado == SM_PARADO && !relatorio_feito) {
    relatorio();
  }
}
