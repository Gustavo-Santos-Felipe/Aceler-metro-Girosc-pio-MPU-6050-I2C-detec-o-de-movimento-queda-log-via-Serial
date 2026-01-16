# Aceler-metro-Girosc-pio-MPU-6050-I2C-detec-o-de-movimento-queda-log-via-Serial
MCU (ex.: ESP32, STM32, ATmega/Arduino) conversa por I2C com o MPU-6050  Você implementa:init do sensor (acordar do sleep)  configuração de escala (±2g, ±4g etc.)  leitura de dados brutos (Accel/Gyro)  conversão para unidades físicas  Faz uma lógica simples:  detecção de pico (impacto) + mudança de orientação
Hardware mínimo

1x ESP32 (ou outro MCU)

1x módulo MPU-6050

jumpers

Ligações (ex. ESP32)

VCC → 3V3

GND → GND

SDA → GPIO 21 (padrão comum)

SCL → GPIO 22 (padrão comum)

Se o seu módulo MPU-6050 já tem resistores pull-up, ok.
Se não tiver, use 4.7k de SDA→3V3 e SCL→3V3 (clássico).
