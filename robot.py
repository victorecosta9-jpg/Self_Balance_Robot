    import gc
    from machine import Pin, PWM, I2C, Timer
    import time
    import math

    gc.collect()

    # ============================================================
    # 1) GANHOS LQR (SUAVES MAS COM RESISTÊNCIA À QUEDA)
    # ============================================================
    K_x     = -0.0500   # Contra a queda lenta
    K_theta = -36.0000  # Valor que manteve a suavidade e eliminou os solavancos
    K_v     = -0.1000   # Amortecimento de roda contido
    K_w     = -2.5000   # Amortecimento angular suave da MPU

    Ts = 0.004  # Loop de controle em 250 Hz (4 ms)

    # Parâmetros Mecânicos e Elétricos
    VBAT_V = 11.1              # Tensão nominal da bateria
    WHEEL_DIAMETER_M = 0.0683  # Diâmetro da roda em metros
    PULSES_PER_REV = 200       # Resolução do encoder
    WHEEL_CIRC_M = math.pi * WHEEL_DIAMETER_M
    DIST_PER_PULSE = WHEEL_CIRC_M / PULSES_PER_REV

    # Segurança e IMU
    ANGLE_FALL_LIMIT_DEG = 35.0  
    GYRO_FS_DPS = 250          
    ACCEL_FS_G = 2             
    COMP_FILTER_ALPHA = 0.995    

    # Correção de Centro de Gravidade em Graus (ajustado por você)
    OFFSET_CENTRO_GRAVIDADE = -1.5  

    # ============================================================
    # 2) DRIVERS DE HARDWARE
    # ============================================================
    class Motor:
        def __init__(self, pin_pwm, pin_in1, pin_in2, freq=20000):
            self.in1 = Pin(pin_in1, Pin.OUT)
            self.in2 = Pin(pin_in2, Pin.OUT)
            self.pwm = PWM(Pin(pin_pwm), freq=freq)
            self.pwm.duty_u16(0)

        def set_voltage(self, volts, vbat=VBAT_V):
            volts = max(-vbat, min(vbat, volts))
            
            if abs(volts) < 0.05:
                self.stop()
                return

            # DEADBAND CIVILIZADA: 0.35V para acionamento limpo sem dar "soco"
            DEADBAND_V = 0.35  
            
            if volts > 0:
                eff_volts = DEADBAND_V + (volts / vbat) * (vbat - DEADBAND_V)
                self.in1.value(1)
                self.in2.value(0)
            else:
                eff_volts = -DEADBAND_V + (volts / vbat) * (vbat - DEADBAND_V)
                self.in1.value(0)
                self.in2.value(1)

            duty_ratio = min(1.0, abs(eff_volts) / vbat)
            self.pwm.duty_u16(int(duty_ratio * 65535))

        def stop(self):
            self.in1.value(0)
            self.in2.value(0)
            self.pwm.duty_u16(0)


    class EncoderEsq:
        def __init__(self, pin_a, pin_b):
            self.pin_a = Pin(pin_a, Pin.IN)
            self.pin_b = Pin(pin_b, Pin.IN)
            self.pulses = 0
            self.pin_a.irq(trigger=Pin.IRQ_RISING, handler=self._on_pulse)

        def _on_pulse(self, pin):
            if self.pin_b.value():
                self.pulses += 1
            else:
                self.pulses -= 1

        def read_pulses(self):
            return self.pulses

        def disable_irq(self):
            self.pin_a.irq(handler=None)


    class EncoderDir:
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

        def disable_irq(self):
            self.pin_a.irq(handler=None)


    class MPU6050:
        ADDR = 0x68
        PWR_MGMT_1 = 0x6B
        ACCEL_XOUT_H = 0x3B

        def __init__(self, i2c, addr=ADDR):
            self.i2c = i2c
            self.addr = addr
            self.i2c.writeto_mem(self.addr, self.PWR_MGMT_1, b"\x00") 
            time.sleep_ms(100)
            self.gyro_bias = 0.0
            self.accel_scale = 16384.0 if ACCEL_FS_G == 2 else 8192.0
            self.gyro_scale = 131.0 if GYRO_FS_DPS == 250 else 65.5

        def read_sensors(self):
            data = self.i2c.readfrom_mem(self.addr, self.ACCEL_XOUT_H, 14)
            ay = (data[2] << 8) | data[3]
            if ay >= 0x8000: ay -= 0x10000
            az = (data[4] << 8) | data[5]
            if az >= 0x8000: az -= 0x10000
            gx = (data[8] << 8) | data[9]
            if gx >= 0x8000: gx -= 0x10000

            accel_angle = math.degrees(math.atan2(ay / self.accel_scale, az / self.accel_scale))
            gyro_dps = (gx / self.gyro_scale) - self.gyro_bias

            return accel_angle, gyro_dps

        def calibrate_gyro(self, samples=200):
            total = 0.0
            for _ in range(samples):
                _, gyro_dps_raw = self.read_sensors()
                total += gyro_dps_raw + self.gyro_bias
                time.sleep_ms(3)
            self.gyro_bias = total / samples


    class ComplementaryFilter:
        def __init__(self, alpha=COMP_FILTER_ALPHA):
            self.alpha = alpha
            self.angle_deg = 0.0

        def update(self, accel_angle_deg, gyro_dps, dt):
            gyro_angle = self.angle_deg + gyro_dps * dt
            self.angle_deg = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle_deg
            return self.angle_deg

    # ============================================================
    # 3) EXECUÇÃO DO CONTROLE LQR
    # ============================================================
    def run():
        led = Pin(2, Pin.OUT)
        
        motor_esq = Motor(pin_pwm=25, pin_in1=27, pin_in2=26)
        motor_dir = Motor(pin_pwm=14, pin_in1=13, pin_in2=12)
        encoder_esq = EncoderEsq(pin_a=33, pin_b=32)
        encoder_dir = EncoderDir(pin_a=35, pin_b=34)

        i2c = I2C(0, scl=Pin(22), sda=Pin(23), freq=400000)
        imu = MPU6050(i2c)
        comp_filter = ComplementaryFilter()

        print("\n--------------------------------------------------")
        print(" MANTENHA O ROBÔ PARADO NA VERTICAL PARA CALIBRAR ")
        print("--------------------------------------------------")
        for i in range(3, 0, -1):
            print(f"Calibrando em {i}s...")
            led.value(1); time.sleep(0.5)
            led.value(0); time.sleep(0.5)

        imu.calibrate_gyro()
        print("--- LQR COM RESISTÊNCIA DE POSIÇÃO ATIVADO ---")

        pulses_esq_ant = encoder_esq.read_pulses()
        pulses_dir_ant = encoder_dir.read_pulses()
        position = 0.0
        velocity = 0.0

        accel_init, _ = imu.read_sensors()
        comp_filter.angle_deg = accel_init

        control_active = True
        timer = Timer(0)

        def control_loop(t):
            nonlocal pulses_esq_ant, pulses_dir_ant, position, velocity, control_active

            if not control_active:
                return

            try:
                accel_angle, gyro_dps = imu.read_sensors()
            except OSError:
                return

            angle_deg = comp_filter.update(accel_angle, gyro_dps, Ts)

            # Leitura dos Encoders
            pulses_esq = encoder_esq.read_pulses()
            pulses_dir = encoder_dir.read_pulses()
            delta_esq = (pulses_esq - pulses_esq_ant) * DIST_PER_PULSE
            delta_dir = (pulses_dir - pulses_dir_ant) * DIST_PER_PULSE
            pulses_esq_ant, pulses_dir_ant = pulses_esq, pulses_dir

            delta_pos = (delta_esq + delta_dir) / 2.0
            position += delta_pos

            # Média móvel ultra rápida de 2 amostras (Sem delay de fase)
            instant_velocity = delta_pos / Ts
            velocity = (velocity + instant_velocity) / 2.0  

            # Cálculo do Erro Angular
            angle_err_deg = angle_deg - OFFSET_CENTRO_GRAVIDADE
            angle_rad = math.radians(angle_err_deg)
            angular_velocity_rad = math.radians(gyro_dps)

            # Corte de Segurança
            if abs(angle_err_deg) > ANGLE_FALL_LIMIT_DEG:
                motor_esq.stop()
                motor_dir.stop()
                control_active = False
                timer.deinit()
                print(f"\nQUEDA DETECTADA ({angle_err_deg:.1f}°)! Motores desligados.")
                return

            # LEI DE CONTROLE LQR
            u = -(K_x * position + K_theta * angle_rad + K_v * velocity + K_w * angular_velocity_rad)

            # Aplica tensão aos motores
            motor_esq.set_voltage(-u)
            motor_dir.set_voltage(u)

        timer.init(period=4, mode=Timer.PERIODIC, callback=control_loop)

        try:
            while control_active:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nCONTROLE INTERROMPIDO PELO USUÁRIO.")
        finally:
            timer.deinit()
            motor_esq.stop()
            motor_dir.stop()
            encoder_esq.disable_irq()
            encoder_dir.disable_irq()

    if __name__ == "__main__":
        run()


