import machine
import time

# Usa o pino do botão BOOT da ESP32 (GPIO 0) como trava física de emergência
boot_button = machine.Pin(0, machine.Pin.IN, machine.Pin.PULL_UP)
led = machine.Pin(2, machine.Pin.OUT)

# Pisca rápido indicando leitura do botão
for _ in range(5):
    led.value(1)
    time.sleep(0.05)
    led.value(0)
    time.sleep(0.05)

print("\n==================================================")
print("[BOOT] Checando botão de emergência (GPIO 0)...")
print("==================================================")

# Se o botão BOOT estiver pressionado ao ligar, cancela a execução e libera o terminal
if boot_button.value() == 0:
    print("\n[CANCELADO] Botão BOOT pressionado! Entrada no REPL liberada.")
    led.value(1)  # LED aceso fixo avisa que está seguro em REPL
else:
    print("[BOOT] Botão solto. Aguardando 2s para iniciar o robô...")
    time.sleep(2)
    try:
        from robot import run  # type: ignore
        run()
    except Exception as e:
        print(f"\n[ERRO DE EXECUÇÃO]: {e}")
