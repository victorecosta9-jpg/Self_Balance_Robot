"""
Controlador LQR (discreto) para robo autobalanceado de 2 rodas,
usando MicroPython no ESP32.

============================================================
CONFIGURACAO DE GPIO (mesma dos scripts anteriores)
============================================================
Motor Esquerdo (Driver 1)
    PWMA+PWMB (unidos) -> GPIO25
    AIN1+BIN1 (unidos) -> GPIO26
    AIN2+BIN2 (unidos) -> GPIO27
    Encoder A (amarelo) -> GPIO32
    Encoder B (verde)   -> GPIO33

Motor Direito (Driver 2)
    PWMA+PWMB (unidos) -> GPIO14
    AIN1+BIN1 (unidos) -> GPIO12
    AIN2+BIN2 (unidos) -> GPIO13
    Encoder A (amarelo) -> GPIO34
    Encoder B (verde)   -> GPIO35

IMU (necessaria para "angle" e "angular_velocity" do controlador,
NAO fazia parte da fiacao descrita anteriormente -> precisa ser
adicionada). Assumindo um MPU6050 no barramento I2C padrao do ESP32:
    VCC -> 3V3
    GND -> GND geral
    SDA -> GPIO21
    SCL -> GPIO22

STBY dos dois drivers: fixo no 3V3 (fora do controle por software).
VM dos dois drivers: fonte externa (mesma tensao usada em VBAT_V abaixo).

============================================================
NAO PRECISA DE BIBLIOTECA EXTERNA
============================================================
O driver do MPU6050 abaixo foi escrito direto com a classe machine.I2C
(que ja vem no firmware do MicroPython), entao nao e necessario instalar
nada. Se preferir usar uma biblioteca pronta em vez do driver embutido,
veja a secao de instalacao no final deste arquivo.
"""

from machine import Pin, PWM, I2C
import time
import math

# ============================================================
# 1) PARAMETROS DO CONTROLADOR (extraidos do MATLAB)
# ============================================================
K_x = -11.1056       # ganho posicao
K_theta = -87.4898   # ganho angulo
K_v = -20.1484        # ganho velocidade linear
K_w = -15.1198        # ganho velocidade angular

Ts = 0.002            # 500 Hz (2 ms) -> tempo de amostragem do MATLAB

# ============================================================
# 2) PARAMETROS FISICOS -- AJUSTE PARA O SEU ROBO
# ============================================================
VBAT_V = 11.1              # tensao nominal da bateria/fonte ligada no VM (V)
WHEEL_DIAMETER_M = 0.0683   # diametro da roda em metros (ajuste!)
PULSES_PER_REV = 700       # pulsos do encoder por volta da roda (ajuste!
                           # depende da reducao da caixa do JGB37-520 e do
                           # numero de pulsos por volta do encoder)
WHEEL_CIRC_M = math.pi * WHEEL_DIAMETER_M
DIST_PER_PULSE = WHEEL_CIRC_M / PULSES_PER_REV

ANGLE_FALL_LIMIT_DEG = 35  # acima disso, desliga os motores por seguranca
GYRO_FS_DPS = 250          # faixa do giroscopio configurada no MPU6050
ACCEL_FS_G = 2             # faixa do acelerometro configurada
COMP_FILTER_ALPHA = 0.98   # peso do giroscopio no filtro complementar

# ============================================================
# 3) DRIVER DO MOTOR (TB6612FNG, canais A+B unidos)
# ============================================================
PWM_FREQ = 20000  # 20 kHz: acima do audivel, reduz ruido no motor


class Motor:
    def __init__(self, pin_pwm, pin_in1, pin_in2, freq=PWM_FREQ):
        self.in1 = Pin(pin_in1, Pin.OUT)
        self.in2 = Pin(pin_in2, Pin.OUT)
        self.pwm = PWM(Pin(pin_pwm), freq=freq)
        self.pwm.duty(0)

    def set_voltage(self, volts, vbat=VBAT_V):
        """Aplica uma tensao equivalente (-vbat a +vbat) no motor,
        convertendo para duty cycle de PWM."""
        volts = max(-vbat, min(vbat, volts))
        duty_pct = abs(volts) / vbat * 100

        if volts > 0:
            self.in1.value(1)
            self.in2.value(0)
        elif volts < 0:
            self.in1.value(0)
            self.in2.value(1)
        else:
            self.in1.value(0)
            self.in2.value(0)

        self.pwm.duty(int(duty_pct * 1023 / 100))

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)


# ============================================================
# 4) ENCODER (posicao/velocidade da roda)
# ============================================================
class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN)
        self.pin_b = Pin(pin_b, Pin.IN)
        self.pulses = 0
        self.pin_a.irq(trigger=Pin.IRQ_RISING, handler=self._on_pulse)

    def _on_pulse(self, pin):
        if self.pin_b.value():
            self.pulses -= 1
        else:
            self.pulses += 1

    def read_pulses(self):
        return self.pulses


# ============================================================
# 5) IMU MPU6050 (angulo de inclinacao + velocidade angular)
# ============================================================
class MPU6050:
    ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    def __init__(self, i2c, addr=ADDR):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, self.PWR_MGMT_1, b"\x00")  # acorda o sensor
        time.sleep_ms(100)
        self.gyro_bias = 0.0
        self.accel_scale = 16384.0 if ACCEL_FS_G == 2 else 8192.0
        self.gyro_scale = 131.0 if GYRO_FS_DPS == 250 else 65.5

    def _read_word(self, reg):
        high = self.i2c.readfrom_mem(self.addr, reg, 1)[0]
        low = self.i2c.readfrom_mem(self.addr, reg + 1, 1)[0]
        val = (high << 8) | low
        if val >= 0x8000:
            val -= 0x10000
        return val

    def read_accel_angle_deg(self):
        """Angulo de inclinacao (pitch) estimado apenas pelo acelerometro."""
        ax = self._read_word(self.ACCEL_XOUT_H) / self.accel_scale
        az = self._read_word(self.ACCEL_XOUT_H + 4) / self.accel_scale
        return math.degrees(math.atan2(ax, az))

    def read_gyro_dps(self):
        """Velocidade angular (graus/s) no eixo de rotacao do pendulo."""
        gy = self._read_word(self.GYRO_XOUT_H + 2) / self.gyro_scale
        return gy - self.gyro_bias

    def calibrate_gyro(self, samples=200):
        """Mede o offset do giroscopio com o robo parado e na vertical."""
        total = 0.0
        for _ in range(samples):
            total += self._read_word(self.GYRO_XOUT_H + 2) / self.gyro_scale
            time.sleep_ms(3)
        self.gyro_bias = total / samples


# ============================================================
# 6) FILTRO COMPLEMENTAR (funde acelerometro + giroscopio)
# ============================================================
class ComplementaryFilter:
    def __init__(self, alpha=COMP_FILTER_ALPHA):
        self.alpha = alpha
        self.angle_deg = 0.0

    def update(self, accel_angle_deg, gyro_dps, dt):
        gyro_angle = self.angle_deg + gyro_dps * dt
        self.angle_deg = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle_deg
        return self.angle_deg


# ============================================================
# 7) INSTANCIAS DE HARDWARE
# ============================================================
motor_esq = Motor(pin_pwm=25, pin_in1=26, pin_in2=27)
motor_dir = Motor(pin_pwm=14, pin_in1=12, pin_in2=13)

encoder_esq = Encoder(pin_a=32, pin_b=33)
encoder_dir = Encoder(pin_a=34, pin_b=35)

i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
imu = MPU6050(i2c)
comp_filter = ComplementaryFilter()


# ============================================================
# 8) LACO DE CONTROLE (LQR discreto a 500 Hz)
# ============================================================
def controlar_robo():
    print("Calibrando giroscopio - mantenha o robo parado e na vertical...")
    imu.calibrate_gyro()
    print("Calibrado. Iniciando controle.")

    pulses_esq_ant = encoder_esq.read_pulses()
    pulses_dir_ant = encoder_dir.read_pulses()
    position = 0.0
    angulo_referencia = comp_filter.angle_deg  # zera na inclinacao inicial

    next_tick = time.ticks_add(time.ticks_us(), int(Ts * 1_000_000))

    while True:
        # --- 1) Le sensores ---
        accel_angle = imu.read_accel_angle_deg()
        gyro_dps = imu.read_gyro_dps()
        angle_deg = comp_filter.update(accel_angle, gyro_dps, Ts)

        pulses_esq = encoder_esq.read_pulses()
        pulses_dir = encoder_dir.read_pulses()
        delta_esq = (pulses_esq - pulses_esq_ant) * DIST_PER_PULSE
        delta_dir = (pulses_dir - pulses_dir_ant) * DIST_PER_PULSE
        pulses_esq_ant, pulses_dir_ant = pulses_esq, pulses_dir

        delta_pos = (delta_esq + delta_dir) / 2.0
        position += delta_pos
        velocity = delta_pos / Ts

        # --- 2) Converte para unidades SI (rad, rad/s) ---
        angle = math.radians(angle_deg - angulo_referencia)
        angular_velocity = math.radians(gyro_dps)

        # --- 3) Seguranca: se caiu demais, desliga ---
        if abs(angle_deg - angulo_referencia) > ANGLE_FALL_LIMIT_DEG:
            motor_esq.stop()
            motor_dir.stop()
            print("Robo caiu -- motores desligados. Reinicie para tentar de novo.")
            return

        # --- 4) Lei de controle LQR ---
        # u = -(K_x*position + K_theta*angle + K_v*velocity + K_w*angular_velocity)
        u = -(K_x * position + K_theta * angle + K_v * velocity + K_w * angular_velocity)

        # --- 5) Aplica a mesma tensao nos dois motores (sem giro/guinada) ---
        motor_esq.set_voltage(u)
        motor_dir.set_voltage(u)

        # --- 6) Aguarda ate completar exatamente Ts (500 Hz) ---
        now = time.ticks_us()
        sleep_us = time.ticks_diff(next_tick, now)
        if sleep_us > 0:
            time.sleep_us(sleep_us)
        next_tick = time.ticks_add(next_tick, int(Ts * 1_000_000))


if __name__ == "__main__":
    try:
        controlar_robo()
    except KeyboardInterrupt:
        motor_esq.stop()
        motor_dir.stop()
        print("Interrompido pelo usuario -- motores desligados.")


"""
============================================================
INSTALACAO DE BIBLIOTECA (OPCIONAL)
============================================================
O script acima ja funciona sem nenhuma biblioteca externa, usando
apenas 'machine' (nativo do firmware MicroPython). Isso e o recomendado,
porque evita depender de um pacote de terceiros para algo tao simples
quanto ler registradores do MPU6050 via I2C.

Se mesmo assim voce preferir usar uma biblioteca MPU6050 pronta:

1) Com o ESP32 conectado ao WiFi, pelo REPL (Thonny, VS Code/MicroPico
   ou mpremote), rode:
       import network
       wlan = network.WLAN(network.STA_IF)
       wlan.active(True)
       wlan.connect("SEU_WIFI", "SUA_SENHA")

2) Depois, ainda no REPL, instale via mip (gerenciador de pacotes do
   MicroPython, disponivel a partir da v1.19):
       import mip
       mip.install("nome-do-pacote-no-index-do-micropython")

3) Caso o ESP32 nao tenha WiFi disponivel no local de uso, baixe o
   arquivo .py da biblioteca no seu PC e copie manualmente para a placa:
       mpremote connect <porta> cp mpu6050.py :mpu6050.py
   (ou arraste o arquivo pelo Thonny em File > Save As > MicroPython device)

Depois de instalada, basta substituir a classe MPU6050 deste script
pelas chamadas da biblioteca escolhida -- a logica do filtro complementar
e do controlador LQR continua igual.
"""
