"""
Controle de dois motores JGB37-520 (com encoder) via dois drivers TB6612FNG,
usando MicroPython no ESP32.

Configuracao de GPIO (conforme etiquetas soldadas na placa):

    Motor Esquerdo (Driver 1)
    --------------------------------
    PWMA + PWMB (unidos)  -> GPIO25  (PWM)
    AIN1 + BIN1 (unidos)  -> GPIO26  (direcao 1)
    AIN2 + BIN2 (unidos)  -> GPIO27  (direcao 2)
    Encoder canal A (amarelo) -> GPIO32
    Encoder canal B (verde)   -> GPIO33
    Encoder VCC (azul)  -> barramento 3V3
    Encoder GND (preto) -> barramento GND geral

    Motor Direito (Driver 2)
    --------------------------------
    PWMA + PWMB (unidos)  -> GPIO14  (PWM)
    AIN1 + BIN1 (unidos)  -> GPIO12  (direcao 1)
    AIN2 + BIN2 (unidos)  -> GPIO13  (direcao 2)
    Encoder canal A (amarelo) -> GPIO34  (entrada apenas)
    Encoder canal B (verde)   -> GPIO35  (entrada apenas)
    Encoder VCC (azul)  -> barramento 3V3
    Encoder GND (preto) -> barramento GND geral

Observacoes:
    - STBY de cada driver deve ser ligado direto no 3V3 (fisicamente),
      ja que nao ha um GPIO dedicado para habilitar/desabilitar.
    - VM de cada driver vem da fonte externa (6-12V), com GND comum
      entre fonte, ESP32 e os dois drivers.
    - Como AIN1+BIN1 e AIN2+BIN2 estao unidos, cada driver move os
      dois canais (A e B) juntos como se fossem um motor so, entao
      o controle e identico ao de um motor unico por lado.
"""

from machine import Pin, PWM
import time

PWM_FREQ = 1000  # 1 kHz


class Motor:
    """Controla um motor ligado a um TB6612FNG com AIN1+BIN1, AIN2+BIN2
    e PWMA+PWMB unidos (STBY fixo em 3V3, fora do controle por software)."""

    def __init__(self, pin_pwm, pin_in1, pin_in2, freq=PWM_FREQ):
        self.in1 = Pin(pin_in1, Pin.OUT)
        self.in2 = Pin(pin_in2, Pin.OUT)
        self.pwm = PWM(Pin(pin_pwm), freq=freq)
        self.pwm.duty(0)

    def _set_duty(self, speed):
        speed = max(0, min(100, speed))
        self.pwm.duty(int(speed * 1023 / 100))

    def forward(self, speed=70):
        self.in1.value(1)
        self.in2.value(0)
        self._set_duty(speed)

    def backward(self, speed=70):
        self.in1.value(0)
        self.in2.value(1)
        self._set_duty(speed)

    def brake(self):
        self.in1.value(1)
        self.in2.value(1)
        self._set_duty(0)

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self._set_duty(0)


class Encoder:
    """Leitura de encoder em quadratura por interrupcao (contagem x1)."""

    def __init__(self, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN)
        self.pin_b = Pin(pin_b, Pin.IN)
        self.position = 0
        self.pin_a.irq(trigger=Pin.IRQ_RISING, handler=self._on_pulse)

    def _on_pulse(self, pin):
        # Se B estiver em nivel diferente de A na borda de subida de A,
        # o sentido de rotacao e um; caso contrario, e o outro.
        if self.pin_b.value():
            self.position -= 1
        else:
            self.position += 1

    def read(self):
        return self.position

    def reset(self):
        self.position = 0


# ---------- Instancias ----------
motor_esq = Motor(pin_pwm=25, pin_in1=26, pin_in2=27)
motor_dir = Motor(pin_pwm=14, pin_in1=12, pin_in2=13)

encoder_esq = Encoder(pin_a=32, pin_b=33)
encoder_dir = Encoder(pin_a=34, pin_b=35)


def andar_para_frente(speed=60, duracao=2):
    motor_esq.forward(speed)
    motor_dir.forward(speed)
    time.sleep(duracao)


def andar_para_tras(speed=60, duracao=2):
    motor_esq.backward(speed)
    motor_dir.backward(speed)
    time.sleep(duracao)


def girar_direita(speed=60, duracao=1):
    motor_esq.forward(speed)
    motor_dir.backward(speed)
    time.sleep(duracao)


def girar_esquerda(speed=60, duracao=1):
    motor_esq.backward(speed)
    motor_dir.forward(speed)
    time.sleep(duracao)


def parar():
    motor_esq.brake()
    motor_dir.brake()


# ---------- Exemplo de uso ----------
if __name__ == "__main__":
    print("Andando para frente...")
    andar_para_frente(speed=60, duracao=2)

    print("Parando...")
    parar()
    time.sleep(1)

    print("Girando a direita...")
    girar_direita(speed=60, duracao=1)

    print("Parando...")
    parar()

    print("Encoder esquerdo:", encoder_esq.read())
    print("Encoder direito:", encoder_dir.read())