# from machine import Pin, I2C
# i2c = I2C(0, scl=Pin(22), sda=Pin(23))
# print(i2c.scan())


"""
Leitura e impressao dos valores do MPU6050, posicionado proximo ao
centro de massa do carrinho (perto dos motores).

Mesma fiacao do script anterior:
    VCC -> 3V3
    GND -> GND geral
    SDA -> GPIO21
    SCL -> GPIO22

Este script NAO controla os motores -- serve apenas para validar
se o sensor esta lendo corretamente e se o eixo de inclinacao
escolhido (accel X/Z) corresponde ao sentido real de "cair para
frente/para tras" do carrinho, antes de rodar o controlador LQR.
"""

from machine import Pin, I2C
import time
import math

GYRO_FS_DPS = 250   # faixa do giroscopio configurada no sensor
ACCEL_FS_G = 2       # faixa do acelerometro configurada


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
        self.accel_scale = 16384.0 if ACCEL_FS_G == 2 else 8192.0
        self.gyro_scale = 131.0 if GYRO_FS_DPS == 250 else 65.5

    def _read_word(self, reg):
        high = self.i2c.readfrom_mem(self.addr, reg, 1)[0]
        low = self.i2c.readfrom_mem(self.addr, reg + 1, 1)[0]
        val = (high << 8) | low
        if val >= 0x8000:
            val -= 0x10000
        return val

    def read_accel_g(self):
        """Aceleracao nos 3 eixos, em g (1g = 9.81 m/s^2)."""
        ax = self._read_word(self.ACCEL_XOUT_H) / self.accel_scale
        ay = self._read_word(self.ACCEL_XOUT_H + 2) / self.accel_scale
        az = self._read_word(self.ACCEL_XOUT_H + 4) / self.accel_scale
        return ax, ay, az

    def read_gyro_dps(self):
        """Velocidade angular nos 3 eixos, em graus/s."""
        gx = self._read_word(self.GYRO_XOUT_H) / self.gyro_scale
        gy = self._read_word(self.GYRO_XOUT_H + 2) / self.gyro_scale
        gz = self._read_word(self.GYRO_XOUT_H + 4) / self.gyro_scale
        return gx, gy, gz

    def read_temp_c(self):
        raw = self._read_word(0x41)
        return raw / 340.0 + 36.53

    def read_pitch_deg(self):
        """Angulo de inclinacao (pitch) estimado so pelo acelerometro."""
        ax, _, az = self.read_accel_g()
        return math.degrees(math.atan2(ax, az))


# ---------- Instancia ----------
i2c = I2C(0, scl=Pin(22), sda=Pin(23), freq=400000)
imu = MPU6050(i2c)

print("Dispositivos I2C encontrados:", [hex(a) for a in i2c.scan()])

# ---------- Loop de leitura ----------
if __name__ == "__main__":
    while True:
        ax, ay, az = imu.read_accel_g()
        gx, gy, gz = imu.read_gyro_dps()
        pitch = imu.read_pitch_deg()
        temp = imu.read_temp_c()

        print(
            "accel(g) x={:+.3f} y={:+.3f} z={:+.3f} | "
            "gyro(dps) x={:+.2f} y={:+.2f} z={:+.2f} | "
            "pitch={:+.2f} deg | temp={:.1f} C".format(
                ax, ay, az, gx, gy, gz, pitch, temp
            )
        )

        time.sleep_ms(200)  # 5 leituras por segundo, so para debug