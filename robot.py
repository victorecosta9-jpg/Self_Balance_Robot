import gc
import math
import time
import machine
from array import array
from machine import I2C, PWM, Pin, Timer


# ============================================================
# CONFIGURAÇÕES DO CONTROLADOR
# ============================================================

K_X = 0.0
K_THETA = -30.0
K_V = 0.0
K_W = -0.5

U_LIMIT_V = 11.1

RAMP_S = 0.15
TRIM_DEG = -1.5

WIN_V = 10

# Controle = 250 Hz
TS = 0.004


# ============================================================
# PARÂMETROS MECÂNICOS E ELÉTRICOS
# ============================================================

VBAT_V = 11.1

WHEEL_DIAMETER_M = 0.0683
PULSES_PER_REV = 700

WHEEL_CIRC_M = math.pi * WHEEL_DIAMETER_M
DIST_PER_PULSE = WHEEL_CIRC_M / PULSES_PER_REV

DEADBAND_V = 0.65
DEADBAND_RAMP_V = 0.2


# ============================================================
# I2C / IMU
# ============================================================

I2C_SCL_PIN = 22
I2C_SDA_PIN = 23

# Mantendo a configuração do código antigo:
# SoftI2C 200 kHz primeiro.
I2C_SOFT_FIRST = True

I2C_HW_FREQ = 400000
I2C_SOFT_FREQ = 200000

# IMPORTANTE:
# Mesmo comportamento do código antigo:
# mantém o último valor válido por 10 falhas.
MAX_I2C_HOLD = 10

# Dados idênticos consecutivos.
# No antigo era 8.
STUCK_LIMIT = 8


# ============================================================
# PROTEÇÕES
# ============================================================

ANGLE_FALL_LIMIT_DEG = 35.0
START_WINDOW_DEG = 5.0

GYRO_FS_DPS = 250
ACCEL_FS_G = 2

COMP_FILTER_ALPHA = 0.995


# ============================================================
# LOG
# ============================================================

NLOG = 1500


# ============================================================
# MOTOR
# ============================================================

class Motor:

    def __init__(self, pin_pwm, pin_in1, pin_in2, freq=20000):

        self.in1 = Pin(pin_in1, Pin.OUT)
        self.in2 = Pin(pin_in2, Pin.OUT)

        self.pwm = PWM(
            Pin(pin_pwm),
            freq=freq
        )

        self.pwm.duty_u16(0)


    def set_voltage(self, volts, vbat=VBAT_V):

        volts = max(
            -vbat,
            min(vbat, volts)
        )

        a = abs(volts)

        if a < 0.02:
            self.stop()
            return


        # Compensação da zona morta
        eff = (
            DEADBAND_V *
            min(
                1.0,
                a / DEADBAND_RAMP_V
            )
            +
            (a / vbat) *
            (vbat - DEADBAND_V)
        )


        if volts > 0:

            self.in1.value(1)
            self.in2.value(0)

        else:

            self.in1.value(0)
            self.in2.value(1)


        duty_ratio = min(
            1.0,
            eff / vbat
        )


        self.pwm.duty_u16(
            int(
                duty_ratio * 65535
            )
        )


    def stop(self):

        self.in1.value(0)
        self.in2.value(0)

        self.pwm.duty_u16(0)


# ============================================================
# ENCODER ESQUERDO
# ============================================================

class EncoderEsq:

    def __init__(self, pin_a, pin_b):

        self.pin_a = Pin(
            pin_a,
            Pin.IN
        )

        self.pin_b = Pin(
            pin_b,
            Pin.IN
        )

        self.pulses = 0

        self.pin_a.irq(
            trigger=Pin.IRQ_RISING,
            handler=self._on_pulse
        )


    def _on_pulse(self, pin):

        if self.pin_b.value():

            self.pulses += 1

        else:

            self.pulses -= 1


    def read_pulses(self):

        return self.pulses


    def disable_irq(self):

        self.pin_a.irq(
            handler=None
        )


# ============================================================
# ENCODER DIREITO
# ============================================================

class EncoderDir:

    def __init__(self, pin_a, pin_b):

        self.pin_a = Pin(
            pin_a,
            Pin.IN
        )

        self.pin_b = Pin(
            pin_b,
            Pin.IN
        )

        self.pulses = 0

        self.pin_a.irq(
            trigger=Pin.IRQ_RISING,
            handler=self._on_pulse
        )


    def _on_pulse(self, pin):

        if self.pin_b.value():

            self.pulses -= 1

        else:

            self.pulses += 1


    def read_pulses(self):

        return self.pulses


    def disable_irq(self):

        self.pin_a.irq(
            handler=None
        )


# ============================================================
# MPU6050
# ============================================================

class MPU6050:

    ADDR = 0x68

    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B


    def __init__(self, i2c, addr=ADDR):

        self.i2c = i2c
        self.addr = addr

        self.gyro_bias = 0.0

        self.raw = (
            0,
            0,
            0
        )


        if ACCEL_FS_G == 2:

            self.accel_scale = 16384.0

        else:

            self.accel_scale = 8192.0


        if GYRO_FS_DPS == 250:

            self.gyro_scale = 131.0

        else:

            self.gyro_scale = 65.5


        # ----------------------------------------------------
        # Inicialização do MPU
        # ----------------------------------------------------

        self.i2c.writeto_mem(
            self.addr,
            self.PWR_MGMT_1,
            b"\x00"
        )

        time.sleep_ms(100)


        # DLPF
        self.i2c.writeto_mem(
            self.addr,
            0x1A,
            b"\x02"
        )

        # Gyro +/-250 dps
        self.i2c.writeto_mem(
            self.addr,
            0x1B,
            b"\x00"
        )

        # Accel +/-2g
        self.i2c.writeto_mem(
            self.addr,
            0x1C,
            b"\x00"
        )

        time.sleep_ms(20)


    def read_sensors(self):

        # ----------------------------------------------------
        # IMPORTANTE:
        # leitura direta igual ao código antigo
        # ----------------------------------------------------

        data = self.i2c.readfrom_mem(
            self.addr,
            self.ACCEL_XOUT_H,
            14
        )


        # ----------------------------------------------------
        # Acelerômetro Y
        # ----------------------------------------------------

        ay = (
            (data[2] << 8)
            |
            data[3]
        )

        if ay >= 0x8000:

            ay -= 0x10000


        # ----------------------------------------------------
        # Acelerômetro Z
        # ----------------------------------------------------

        az = (
            (data[4] << 8)
            |
            data[5]
        )

        if az >= 0x8000:

            az -= 0x10000


        # ----------------------------------------------------
        # Giroscópio X
        # ----------------------------------------------------

        gx = (
            (data[8] << 8)
            |
            data[9]
        )

        if gx >= 0x8000:

            gx -= 0x10000


        # ----------------------------------------------------
        # Dados brutos
        # ----------------------------------------------------

        self.raw = (
            ay,
            az,
            gx
        )


        # ----------------------------------------------------
        # Ângulo do acelerômetro
        # ----------------------------------------------------

        accel_angle = math.degrees(
            math.atan2(
                ay / self.accel_scale,
                az / self.accel_scale
            )
        )


        # ----------------------------------------------------
        # Giro
        # ----------------------------------------------------

        gyro_dps = (
            gx / self.gyro_scale
        ) - self.gyro_bias


        return (
            accel_angle,
            gyro_dps
        )


    def calibrate_gyro(self, samples=200):

        total = 0.0

        for _ in range(samples):

            _, gyro_raw = self.read_sensors()

            total += (
                gyro_raw
                +
                self.gyro_bias
            )

            time.sleep_ms(3)


        self.gyro_bias = (
            total / samples
        )


# ============================================================
# FILTRO COMPLEMENTAR
# ============================================================

class ComplementaryFilter:

    def __init__(
        self,
        alpha=COMP_FILTER_ALPHA
    ):

        self.alpha = alpha

        self.angle_deg = 0.0


    def update(
        self,
        accel_angle_deg,
        gyro_dps,
        dt
    ):

        gyro_angle = (
            self.angle_deg
            +
            gyro_dps * dt
        )


        self.angle_deg = (
            self.alpha * gyro_angle
            +
            (1.0 - self.alpha)
            *
            accel_angle_deg
        )


        return self.angle_deg


# ============================================================
# RECUPERAÇÃO DO BARRAMENTO
#
# SOMENTE USADA NA INICIALIZAÇÃO.
#
# NÃO USAR DURANTE O CONTROLE.
# ============================================================

def i2c_bus_recover():

    print(
        "[I2C] Recuperando barramento..."
    )


    try:

        scl = Pin(
            I2C_SCL_PIN,
            Pin.OPEN_DRAIN,
            value=1
        )

        sda = Pin(
            I2C_SDA_PIN,
            Pin.OPEN_DRAIN,
            value=1
        )


        time.sleep_us(20)


        # 9 clocks
        for _ in range(9):

            scl.value(0)

            time.sleep_us(10)

            scl.value(1)

            time.sleep_us(10)


        # STOP

        scl.value(0)
        sda.value(0)

        time.sleep_us(10)

        scl.value(1)

        time.sleep_us(10)

        sda.value(1)

        time.sleep_us(10)


        print(
            "[I2C] Barramento liberado."
        )


    except Exception as e:

        print(
            "[I2C] Aviso na recuperacao:",
            e
        )


# ============================================================
# CRIAÇÃO DO I2C
#
# SoftI2C primeiro, igual ao código antigo.
# ============================================================

def create_i2c():

    op_hw = (
        "I2C hardware 400k",
        lambda: I2C(
            0,
            scl=Pin(I2C_SCL_PIN),
            sda=Pin(I2C_SDA_PIN),
            freq=I2C_HW_FREQ
        )
    )


    op_sw = (
        "SoftI2C 200k",
        lambda: machine.SoftI2C(
            scl=Pin(I2C_SCL_PIN),
            sda=Pin(I2C_SDA_PIN),
            freq=I2C_SOFT_FREQ
        )
    )


    if I2C_SOFT_FIRST:

        opcoes = (
            op_sw,
            op_hw
        )

    else:

        opcoes = (
            op_hw,
            op_sw
        )


    for nome, fabrica in opcoes:

        try:

            i2c = fabrica()

            devs = i2c.scan()


            print(
                "[I2C] %s: dispositivos %s"
                %
                (
                    nome,
                    [
                        hex(d)
                        for d in devs
                    ]
                )
            )


            if (
                0x68 in devs
                or
                0x69 in devs
            ):

                return (
                    i2c,
                    nome
                )


        except Exception as e:

            print(
                "[I2C] %s erro: %s"
                %
                (
                    nome,
                    e
                )
            )


    return (
        None,
        None
    )


# ============================================================
# SETUP IMU
# ============================================================

def setup_imu():

    time.sleep_ms(300)

    i2c_bus_recover()


    for tentativa in range(3):

        print(
            "[I2C] Inicializacao %d/3"
            %
            (
                tentativa + 1
            )
        )


        i2c, nome = create_i2c()


        if i2c is None:

            time.sleep_ms(100)

            i2c_bus_recover()

            continue


        devs = i2c.scan()


        for addr in (
            0x68,
            0x69
        ):

            if addr not in devs:

                continue


            for tentativa_imu in range(3):

                try:

                    imu = MPU6050(
                        i2c,
                        addr
                    )


                    # Teste real
                    imu.read_sensors()


                    print(
                        "[IMU] MPU6050 iniciado "
                        "em 0x%02X via %s"
                        %
                        (
                            addr,
                            nome
                        )
                    )


                    return (
                        imu,
                        i2c
                    )


                except OSError as e:

                    print(
                        "[IMU] tentativa %d falhou: %s"
                        %
                        (
                            tentativa_imu + 1,
                            e
                        )
                    )

                    time.sleep_ms(200)


                except Exception as e:

                    print(
                        "[IMU] erro: %s"
                        % e
                    )

                    time.sleep_ms(200)


        i2c_bus_recover()


    print()
    print(
        "[ERRO] MPU6050 nao respondeu."
    )


    return (
        None,
        None
    )


# ============================================================
# ESTADO
# ============================================================

class State:
    pass


# ============================================================
# PROGRAMA
# ============================================================

def run():

    led = Pin(
        2,
        Pin.OUT
    )


    # ========================================================
    # IMU
    # ========================================================

    imu, i2c = setup_imu()


    if imu is None:

        led.value(0)

        return


    # ========================================================
    # MOTORES
    # ========================================================

    motor_esq = Motor(
        pin_pwm=25,
        pin_in1=27,
        pin_in2=26
    )


    motor_dir = Motor(
        pin_pwm=14,
        pin_in1=13,
        pin_in2=12
    )


    # ========================================================
    # ENCODERS
    # ========================================================

    encoder_esq = EncoderEsq(
        pin_a=33,
        pin_b=32
    )


    encoder_dir = EncoderDir(
        pin_a=35,
        pin_b=34
    )


    # ========================================================
    # FILTRO
    # ========================================================

    comp_filter = ComplementaryFilter()


    # ========================================================
    # CALIBRAÇÃO
    # ========================================================

    print()
    print(
        "--------------------------------------------------"
    )

    print(
        " MANTENHA O ROBO PARADO NA VERTICAL"
    )

    print(
        "--------------------------------------------------"
    )


    for i in range(3, 0, -1):

        print(
            "Calibrando em %ds..."
            % i
        )

        led.value(1)

        time.sleep(0.5)

        led.value(0)

        time.sleep(0.5)


    print(
        "[IMU] Calibrando gyro..."
    )


    try:

        imu.calibrate_gyro()


    except Exception as e:

        print(
            "[ERRO] Calibracao:",
            e
        )

        motor_esq.stop()
        motor_dir.stop()

        encoder_esq.disable_irq()
        encoder_dir.disable_irq()

        return


    print(
        "[IMU] Bias gyro = %.4f dps"
        %
        imu.gyro_bias
    )


    # ========================================================
    # REFERÊNCIA ANGULAR
    # ========================================================

    s = 0.0
    valid = 0


    for _ in range(100):

        try:

            a, _g = imu.read_sensors()

            s += a

            valid += 1


        except OSError:

            pass


        time.sleep_ms(4)


    if valid == 0:

        print(
            "[ERRO] Nenhuma leitura valida."
        )

        motor_esq.stop()
        motor_dir.stop()

        return


    angle_ref = (
        s / valid
    )


    comp_filter.angle_deg = angle_ref


    print(
        "Referencia = %.2f deg | bias = %.3f dps"
        %
        (
            angle_ref,
            imu.gyro_bias
        )
    )


    # ========================================================
    # ESPERA POSIÇÃO
    # ========================================================

    print(
        "Segure o robo na vertical..."
    )


    ok = 0

    t_wait = time.ticks_ms()


    while ok < 20:

        try:

            acc, gyr = imu.read_sensors()


            ang = comp_filter.update(
                acc,
                gyr,
                TS
            )


            if abs(
                ang - angle_ref
            ) < START_WINDOW_DEG:

                ok += 1

            else:

                ok = 0


        except OSError:

            time.sleep_ms(4)

            continue


        if time.ticks_diff(
            time.ticks_ms(),
            t_wait
        ) > 5000:

            print(
                "Timeout esperando a vertical."
            )

            motor_esq.stop()
            motor_dir.stop()

            encoder_esq.disable_irq()
            encoder_dir.disable_irq()

            return


        time.sleep_ms(4)


    gc.collect()


    # ========================================================
    # ESTADO
    # ========================================================

    S = State()


    S.active = True

    S.reason = "usuario"


    S.pe_prev = (
        encoder_esq.read_pulses()
    )

    S.pd_prev = (
        encoder_dir.read_pulses()
    )


    S.pos = 0.0
    S.vel = 0.0

    S.k = 0


    # ========================================================
    # I2C
    #
    # IGUAL AO CÓDIGO ANTIGO
    # ========================================================

    S.i2c_fail = 0
    S.i2c_total = 0

    S.raw_prev = None
    S.stuck = 0


    # Última leitura válida
    S.last_acc = angle_ref
    S.last_gyr = 0.0


    # ========================================================
    # CONTROLE
    # ========================================================

    S.slow = 0

    S.t0 = time.ticks_us()
    S.last = S.t0


    # ========================================================
    # VELOCIDADE
    # ========================================================

    S.hp = [
        0.0
    ] * WIN_V

    S.ht = [
        S.t0
    ] * WIN_V


    # ========================================================
    # DIAGNÓSTICO
    # ========================================================

    S.dt_max = 0.0
    S.cyc_max = 0
    S.li = 0


    # ========================================================
    # LOG
    # ========================================================

    log_t = array(
        "f",
        [0.0] * NLOG
    )

    log_e = array(
        "f",
        [0.0] * NLOG
    )

    log_g = array(
        "f",
        [0.0] * NLOG
    )

    log_u = array(
        "f",
        [0.0] * NLOG
    )

    log_v = array(
        "f",
        [0.0] * NLOG
    )

    log_x = array(
        "f",
        [0.0] * NLOG
    )


    # ========================================================
    # TIMER
    # ========================================================

    timer = Timer(0)


    # ========================================================
    # CALLBACK DO CONTROLADOR
    #
    # AQUI ESTÁ A DIFERENÇA IMPORTANTE:
    #
    # A IMU É LIDA AQUI, COMO NO CÓDIGO ANTIGO.
    #
    # NÃO EXISTE:
    # - recuperação I2C
    # - criação de novo I2C
    # - criação de novo MPU
    # - recalibração
    #
    # Se perder I2C:
    # -> mantém último acc/gyr válido.
    # ========================================================

    def control_loop(t):

        if not S.active:

            return


        t_c = time.ticks_us()


        try:

            # =================================================
            # DT
            # =================================================

            dt = (
                time.ticks_diff(
                    t_c,
                    S.last
                )
                *
                1e-6
            )


            S.last = t_c


            if S.k > 0:

                if dt > S.dt_max:

                    S.dt_max = dt


                if dt > 0.007:

                    S.slow += 1


            if (
                dt < 0.001
                or
                dt > 0.02
            ):

                dt = TS


            # =================================================
            # IMU
            #
            # MESMO COMPORTAMENTO DO CÓDIGO ANTIGO
            # =================================================

            try:

                accel_angle, gyro_dps = (
                    imu.read_sensors()
                )


                # ---------------------------------------------
                # Leitura válida
                # ---------------------------------------------

                S.i2c_fail = 0


                S.last_acc = accel_angle
                S.last_gyr = gyro_dps


                # ---------------------------------------------
                # Detecta IMU travada
                # ---------------------------------------------

                if imu.raw == S.raw_prev:

                    S.stuck += 1


                    if S.stuck >= STUCK_LIMIT:

                        motor_esq.stop()
                        motor_dir.stop()


                        S.reason = (
                            "imu_travado"
                        )


                        S.active = False

                        return


                else:

                    S.stuck = 0


                S.raw_prev = imu.raw


            except OSError:

                # =============================================
                # I2C FALHOU
                #
                # NÃO RECALIBRA
                # NÃO REINICIA
                # NÃO RECUPERA
                #
                # APENAS USA O ÚLTIMO VALOR VÁLIDO.
                # =============================================

                S.i2c_fail += 1
                S.i2c_total += 1


                accel_angle = (
                    S.last_acc
                )

                gyro_dps = (
                    S.last_gyr
                )


                # =============================================
                # Só para depois de 10 falhas consecutivas.
                # =============================================

                if (
                    S.i2c_fail
                    >
                    MAX_I2C_HOLD
                ):

                    motor_esq.stop()
                    motor_dir.stop()


                    S.reason = (
                        "i2c"
                    )


                    S.active = False

                    return


            # =================================================
            # FILTRO COMPLEMENTAR
            #
            # IMPORTANTE:
            # mesmo durante erro I2C, o filtro continua usando
            # o último gyro válido.
            #
            # Isso é exatamente a lógica do antigo.
            # =================================================

            angle_deg = (
                comp_filter.update(
                    accel_angle,
                    gyro_dps,
                    dt
                )
            )


            # =================================================
            # ERRO ANGULAR
            # =================================================

            angle_err_deg = (
                angle_deg
                -
                (
                    angle_ref
                    +
                    TRIM_DEG
                )
            )


            # =================================================
            # PROTEÇÃO DE QUEDA
            # =================================================

            if (
                abs(angle_err_deg)
                >
                ANGLE_FALL_LIMIT_DEG
            ):

                motor_esq.stop()
                motor_dir.stop()


                S.reason = (
                    "queda (%.1f deg)"
                    %
                    angle_err_deg
                )


                S.active = False

                return


            # =================================================
            # ENCODERS
            # =================================================

            pe = (
                encoder_esq.read_pulses()
            )

            pd = (
                encoder_dir.read_pulses()
            )


            S.pos += (
                (
                    (pe - S.pe_prev)
                    +
                    (pd - S.pd_prev)
                )
                *
                0.5
                *
                DIST_PER_PULSE
            )


            S.pe_prev = pe
            S.pd_prev = pd


            # =================================================
            # VELOCIDADE
            # =================================================

            idx = (
                S.k % WIN_V
            )


            if S.k >= WIN_V:

                span = (
                    time.ticks_diff(
                        t_c,
                        S.ht[idx]
                    )
                    *
                    1e-6
                )


                if span > 0:

                    S.vel = (
                        S.pos
                        -
                        S.hp[idx]
                    ) / span


            S.hp[idx] = S.pos
            S.ht[idx] = t_c


            # =================================================
            # TEMPO
            # =================================================

            t_s = (
                time.ticks_diff(
                    t_c,
                    S.t0
                )
                *
                1e-6
            )


            # =================================================
            # RAMPA
            # =================================================

            if RAMP_S <= 0.0:

                g_ramp = 1.0

            else:

                g_ramp = min(
                    1.0,
                    t_s / RAMP_S
                )


            # =================================================
            # CONTROLADOR
            # =================================================

            u = -(
                K_X * S.pos
                +
                K_THETA *
                math.radians(
                    angle_err_deg
                )
                +
                K_V * S.vel
                +
                K_W *
                math.radians(
                    gyro_dps
                )
            )


            u *= g_ramp


            # =================================================
            # SATURAÇÃO
            # =================================================

            u = max(
                -U_LIMIT_V,
                min(
                    U_LIMIT_V,
                    u
                )
            )


            # =================================================
            # MOTORES
            # =================================================

            motor_esq.set_voltage(-u)
            motor_dir.set_voltage(u)


            # =================================================
            # LOG
            # =================================================

            j = (
                S.li % NLOG
            )


            log_t[j] = (
                t_s * 1000.0
            )

            log_e[j] = (
                angle_err_deg
            )

            log_g[j] = (
                gyro_dps
            )

            log_u[j] = u

            log_v[j] = S.vel

            log_x[j] = S.pos


            S.li += 1
            S.k += 1


            # =================================================
            # TEMPO DE PROCESSAMENTO
            # =================================================

            cyc = time.ticks_diff(
                time.ticks_us(),
                t_c
            )


            if cyc > S.cyc_max:

                S.cyc_max = cyc


        except Exception as e:

            # =================================================
            # QUALQUER ERRO FORA DO I2C
            # =================================================

            motor_esq.stop()
            motor_dir.stop()


            S.reason = (
                "excecao: "
                +
                str(e)
            )


            S.active = False


    # ========================================================
    # INÍCIO
    # ========================================================

    print()

    print(
        "=================================================="
    )

    print(
        " GO! Controle ativado."
    )

    print(
        "=================================================="
    )

    print(
        "Kx     = %.4f" % K_X
    )

    print(
        "Ktheta = %.4f" % K_THETA
    )

    print(
        "Kv     = %.4f" % K_V
    )

    print(
        "Kw     = %.4f" % K_W
    )

    print(
        "RAMP_S = %.3f s" % RAMP_S
    )

    print(
        "I2C hold = %d leituras"
        % MAX_I2C_HOLD
    )

    print(
        "I2C sem recuperacao durante controle"
    )

    print(
        "IMU = 250 Hz"
    )


    led.value(1)


    S.t0 = time.ticks_us()
    S.last = S.t0


    # ========================================================
    # TIMER 250 Hz
    # ========================================================

    timer.init(
        period=4,
        mode=Timer.PERIODIC,
        callback=control_loop
    )


    # ========================================================
    # LOOP PRINCIPAL
    # ========================================================

    last_status_ms = time.ticks_ms()
    last_gc_ms = time.ticks_ms()


    try:

        while S.active:

            now_ms = time.ticks_ms()


            # =================================================
            # STATUS
            # =================================================

            if (
                time.ticks_diff(
                    now_ms,
                    last_status_ms
                )
                >=
                1000
            ):

                last_status_ms = now_ms


                print(
                    "[STATUS] "
                    "i2c_fail=%d "
                    "i2c_total=%d "
                    "stuck=%d "
                    "angle=%.2f "
                    "gyro=%.2f"
                    %
                    (
                        S.i2c_fail,
                        S.i2c_total,
                        S.stuck,
                        S.last_acc,
                        S.last_gyr
                    )
                )


            # =================================================
            # GC
            # =================================================

            if (
                time.ticks_diff(
                    now_ms,
                    last_gc_ms
                )
                >=
                5000
            ):

                last_gc_ms = now_ms

                gc.collect()


            # =================================================
            # CPU
            # =================================================

            time.sleep_ms(1)


    except KeyboardInterrupt:

        S.reason = "usuario"


    finally:

        # ----------------------------------------------------
        # Timer
        # ----------------------------------------------------

        try:

            timer.deinit()

        except Exception:

            pass


        # ----------------------------------------------------
        # Motores
        # ----------------------------------------------------

        motor_esq.stop()
        motor_dir.stop()


        # ----------------------------------------------------
        # Encoders
        # ----------------------------------------------------

        encoder_esq.disable_irq()
        encoder_dir.disable_irq()


        # ----------------------------------------------------
        # LED
        # ----------------------------------------------------

        led.value(0)


    # ========================================================
    # RESULTADOS
    # ========================================================

    t_fim = (
        time.ticks_diff(
            time.ticks_us(),
            S.t0
        )
        *
        1e-6
    )


    print()

    print(
        "=================================================="
    )

    print(
        " FIM: %s"
        % S.reason
    )

    print(
        "=================================================="
    )


    print(
        "Amostras: %d | duracao ~%.2f s"
        %
        (
            S.li,
            t_fim
        )
    )


    print(
        "Intervalo maximo Timer: %.2f ms"
        %
        (
            S.dt_max * 1000.0
        )
    )


    print(
        "Tempo maximo processamento: %d us"
        %
        S.cyc_max
    )


    print(
        "Falhas I2C totais: %d"
        %
        S.i2c_total
    )


    print(
        "Falha I2C pendente: %d"
        %
        S.i2c_fail
    )


    print(
        "Intervalos Timer > 7 ms: %d"
        %
        S.slow
    )


    # ========================================================
    # LOG
    # ========================================================

    n = min(
        S.li,
        NLOG
    )


    start = (
        S.li - n
    )


    print()

    print(
        "LOG (CSV, ultimas %d amostras):"
        % n
    )

    print(
        "t_ms,err_deg,gyro_dps,u_V,vel_mps,pos_m"
    )


    for q in range(n):

        j = (
            start + q
        ) % NLOG


        print(
            "%.0f,%.2f,%.1f,%.2f,%.3f,%.3f"
            %
            (
                log_t[j],
                log_e[j],
                log_g[j],
                log_u[j],
                log_v[j],
                log_x[j]
            )
        )


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    gc.collect()

    run()
