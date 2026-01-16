#include <stdint.h>                                      // tipos inteiros fixos
#include <stdarg.h>                                      // variadicos para printf
#include <stdio.h>                                       // vsnprintf
#include "mpu6050.h"                                     // driver do sensor

// -------------------------------                         // separador visual
// HAL (adapte para seu microcontrolador)                 // comentario de bloco
// -------------------------------                         // separador visual

static int hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len); // TODO: implementar I2C write
static int hal_i2c_read (uint8_t addr, uint8_t reg,       uint8_t *data, uint16_t len); // TODO: implementar I2C read
static void hal_delay_ms(uint32_t ms);                    // TODO: implementar delay
static void hal_uart_printf(const char *fmt, ...);        // TODO: implementar log/serial

// -------------------------------                         // separador visual
// Parametros de amostragem e detecao                     // comentario de bloco
// -------------------------------                         // separador visual

#define SAMPLE_PERIOD_MS            (20u)                 // 50 Hz de amostragem
#define MA_WINDOW                   (8u)                  // janela da media movel (potencia de 2 facilita)
#define IMPACT_THRESHOLD_MG         (1800)                // impacto: magnitude > 1.8g (em mg)
#define TILT_CHANGE_THRESHOLD_MG    (700)                 // mudanca de orientacao (delta nos eixos) em mg
#define EVENT_HOLD_MS               (1500u)               // tempo mantendo estado EVENTO
#define I2C_RETRY_COUNT             (3u)                  // tentativas de leitura em erro

// -------------------------------                         // separador visual
// Conversoes (MPU6050)                                   // comentario de bloco
// -------------------------------                         // separador visual

// Para ACCEL em +-2g: 16384 LSB/g (datasheet)            // comentario: LSB por g
#define ACCEL_LSB_PER_G             (16384)               // 16384 LSB = 1g
#define MG_PER_G                    (1000)                // 1000 mg em 1g

// -------------------------------                         // separador visual
// Utilitarios                                               // comentario de bloco
// -------------------------------                         // separador visual

static int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; } // abs simples 32-bit

static int32_t raw_to_mg(int16_t raw)                      // converte leitura bruta para mg (aprox)
{                                                         // abre funcao
    int32_t mg = ((int32_t)raw * MG_PER_G) / ACCEL_LSB_PER_G; // mg = raw * 1000 / 16384
    return mg;                                            // retorna mg
}                                                         // fecha funcao

// Aproximacao barata de magnitude: |x|+|y|+|z| (sem sqrt)  // comentario: evita float e sqrt
static int32_t magnitude_approx_mg(int32_t ax_mg, int32_t ay_mg, int32_t az_mg) // magnitude aproximada
{                                                         // abre funcao
    return iabs32(ax_mg) + iabs32(ay_mg) + iabs32(az_mg);  // soma dos modulos (proxy de intensidade)
}                                                         // fecha funcao

// -------------------------------                         // separador visual
// Media movel (simples)                                   // comentario de bloco
// -------------------------------                         // separador visual

typedef struct                                            // estrutura do filtro MA
{                                                         // abre struct
    int32_t buf[MA_WINDOW];                               // buffer circular
    uint32_t idx;                                         // indice atual
    int64_t sum;                                          // soma acumulada (64-bit para nao estourar)
} ma_filter_t;                                            // nome do tipo

static void ma_init(ma_filter_t *f)                       // inicializa filtro
{                                                         // abre funcao
    uint32_t i = 0;                                       // contador
    f->idx = 0;                                           // zera indice
    f->sum = 0;                                           // zera soma
    for (i = 0; i < MA_WINDOW; i++) { f->buf[i] = 0; }     // zera buffer
}                                                         // fecha funcao

static int32_t ma_push(ma_filter_t *f, int32_t x)          // insere e retorna media
{                                                         // abre funcao
    f->sum -= f->buf[f->idx];                             // remove valor antigo da soma
    f->buf[f->idx] = x;                                   // coloca valor novo
    f->sum += x;                                          // adiciona valor novo na soma
    f->idx = (f->idx + 1u) % MA_WINDOW;                    // avanca circularmente
    return (int32_t)(f->sum / (int64_t)MA_WINDOW);         // retorna media
}                                                         // fecha funcao

// -------------------------------                         // separador visual
// Maquina de estados                                       // comentario de bloco
// -------------------------------                         // separador visual

typedef enum                                              // estados do sistema
{                                                         // abre enum
    ST_IDLE = 0,                                          // aguardando estabilizar
    ST_MONITOR,                                           // monitorando continuamente
    ST_EVENT                                              // evento detectado (impacto/queda)
} app_state_t;                                            // nome do tipo

// -------------------------------                         // separador visual
// Leitura robusta com retry                                // comentario de bloco
// -------------------------------                         // separador visual

static int read_raw_with_retry(mpu6050_bus_t *bus, mpu6050_raw_t *raw) // leitura com retentativas
{                                                         // abre funcao
    uint32_t t = 0;                                       // contador de tentativas
    for (t = 0; t < I2C_RETRY_COUNT; t++) {                // loop de retry
        if (mpu6050_read_raw(bus, raw) == 0) return 0;     // se sucesso, retorna
        hal_delay_ms(5);                                  // pequena espera antes de tentar de novo
    }                                                     // fecha loop
    return -1;                                            // falhou
}                                                         // fecha funcao

// -------------------------------                         // separador visual
// main                                                     // comentario de bloco
// -------------------------------                         // separador visual

int main(void)                                            // ponto de entrada
{                                                         // abre main
    mpu6050_bus_t bus;                                    // estrutura do barramento
    mpu6050_raw_t raw;                                    // leituras brutas
    app_state_t st = ST_IDLE;                             // estado inicial
    uint32_t idle_ms = 0;                                 // contador para sair do IDLE
    uint32_t event_ms = 0;                                // tempo em EVENTO

    int32_t ax_mg = 0;                                    // accel X em mg
    int32_t ay_mg = 0;                                    // accel Y em mg
    int32_t az_mg = 0;                                    // accel Z em mg

    int32_t ax_f = 0;                                     // accel X filtrado
    int32_t ay_f = 0;                                     // accel Y filtrado
    int32_t az_f = 0;                                     // accel Z filtrado

    int32_t ax_prev = 0;                                  // X anterior (para tilt)
    int32_t ay_prev = 0;                                  // Y anterior (para tilt)
    int32_t az_prev = 0;                                  // Z anterior (para tilt)

    int32_t mag = 0;                                      // magnitude aproximada
    int32_t tilt_delta = 0;                               // delta total de orientacao

    ma_filter_t fx;                                       // filtro para X
    ma_filter_t fy;                                       // filtro para Y
    ma_filter_t fz;                                       // filtro para Z

    ma_init(&fx);                                         // init filtro X
    ma_init(&fy);                                         // init filtro Y
    ma_init(&fz);                                         // init filtro Z

    bus.addr = MPU6050_I2C_ADDR_0x68;                      // endereco (ajuste para 0x69 se AD0=1)
    bus.write = hal_i2c_write;                             // pluga write
    bus.read  = hal_i2c_read;                              // pluga read

    hal_uart_printf("Boot: iniciando MPU6050...\r\n");     // log inicial

    if (mpu6050_init(&bus, MPU6050_ACCEL_2G, MPU6050_GYRO_250DPS) != 0) { // init sensor
        hal_uart_printf("ERRO: falha ao inicializar MPU6050.\r\n");       // log erro
        while (1) { hal_delay_ms(1000); }                  // trava seguro
    }                                                      // fim do if

    hal_uart_printf("OK: MPU6050 pronto. Monitorando...\r\n"); // log ok

    while (1)                                              // loop principal
    {                                                      // abre loop
        if (read_raw_with_retry(&bus, &raw) != 0) {         // tenta ler sensor
            hal_uart_printf("WARN: erro I2C ao ler sensor.\r\n"); // log warning
            hal_delay_ms(SAMPLE_PERIOD_MS);                 // espera proximo ciclo
            continue;                                       // pula processamento
        }                                                   // fim do if

        ax_mg = raw_to_mg(raw.ax);                          // converte AX para mg
        ay_mg = raw_to_mg(raw.ay);                          // converte AY para mg
        az_mg = raw_to_mg(raw.az);                          // converte AZ para mg

        ax_f = ma_push(&fx, ax_mg);                         // filtra AX
        ay_f = ma_push(&fy, ay_mg);                         // filtra AY
        az_f = ma_push(&fz, az_mg);                         // filtra AZ

        mag = magnitude_approx_mg(ax_f, ay_f, az_f);        // calcula "intensidade" aproximada

        tilt_delta = iabs32(ax_f - ax_prev)                 // delta X
                   + iabs32(ay_f - ay_prev)                 // delta Y
                   + iabs32(az_f - az_prev);                // delta Z

        ax_prev = ax_f;                                     // atualiza prev X
        ay_prev = ay_f;                                     // atualiza prev Y
        az_prev = az_f;                                     // atualiza prev Z

        switch (st)                                         // seleciona estado
        {                                                   // abre switch
            case ST_IDLE:                                   // estado IDLE
                idle_ms += SAMPLE_PERIOD_MS;                // acumula tempo
                if (idle_ms >= 800u) {                      // espera ~0.8s para estabilizar filtro
                    st = ST_MONITOR;                         // muda para monitorar
                    hal_uart_printf("ST: MONITOR\r\n");      // log estado
                }                                           // fim if
                break;                                      // sai do case

            case ST_MONITOR:                                // estado MONITOR
            {                                               // abre bloco do case
                int impact = (mag >= (IMPACT_THRESHOLD_MG * 3)); // mag aprox soma |x|+|y|+|z| ~ 3g em repouso
                int tilt   = (tilt_delta >= TILT_CHANGE_THRESHOLD_MG); // mudanca de orientacao

                // OBS: como mag aqui é soma de módulos, em repouso perto de 1000mg*3 = 3000 // comentario
                // Por isso comparamos com IMPACT_THRESHOLD_MG*3 para manter coerencia         // comentario

                if (impact && tilt) {                       // criterio simples: impacto + tilt
                    st = ST_EVENT;                           // entra em EVENTO
                    event_ms = 0;                            // zera tempo do evento
                    hal_uart_printf("EVENTO: impacto+tilt | mag=%ldmg(aprox) tilt=%ldmg\r\n", (long)mag, (long)tilt_delta); // log evento
                } else {                                    // caso nao evento
                    // Log leve (opcional): comente se quiser menos spam                      // comentario
                    // hal_uart_printf("ax=%ld ay=%ld az=%ld | mag=%ld tilt=%ld\r\n", (long)ax_f, (long)ay_f, (long)az_f, (long)mag, (long)tilt_delta); // debug
                }                                           // fim else

                break;                                      // sai do case
            }                                               // fecha bloco do case

            case ST_EVENT:                                  // estado EVENT
                event_ms += SAMPLE_PERIOD_MS;               // acumula tempo em evento
                if (event_ms >= EVENT_HOLD_MS) {            // depois de segurar, volta a monitorar
                    st = ST_MONITOR;                         // volta
                    hal_uart_printf("ST: MONITOR\r\n");      // log estado
                }                                           // fim if
                break;                                      // sai do case

            default:                                        // seguranca
                st = ST_IDLE;                               // reseta estado
                idle_ms = 0;                                // reseta contador
                break;                                      // sai do default
        }                                                   // fecha switch

        hal_delay_ms(SAMPLE_PERIOD_MS);                     // controla taxa de amostragem
    }                                                       // fecha loop

    return 0;                                               // nunca chega aqui
}                                                           // fecha main

// -------------------------------                         // separador visual
// Implementacao exemplo de UART printf                     // comentario de bloco
// -------------------------------                         // separador visual

static void hal_uart_printf(const char *fmt, ...)           // printf serial (exemplo)
{                                                           // abre funcao
    char buf[256];                                          // buffer local
    va_list ap;                                             // lista variadica
    va_start(ap, fmt);                                      // inicia lista
    vsnprintf(buf, sizeof(buf), fmt, ap);                    // formata string
    va_end(ap);                                             // fecha lista

    // TODO: envie 'buf' para a UART/Serial do seu MCU       // comentario
    // Ex.: HAL_UART_Transmit (STM32) / uart_write_bytes (ESP-IDF) / Serial.print (Arduino) // comentario
}                                                           // fecha funcao

// -------------------------------                         // separador visual
// Stubs (voce PRECISA adaptar)                             // comentario de bloco
// -------------------------------                         // separador visual

static int hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len) // stub write
{                                                           // abre funcao
    (void)addr;                                             // evita warning
    (void)reg;                                              // evita warning
    (void)data;                                             // evita warning
    (void)len;                                              // evita warning
    return -1;                                              // TODO: implementar e retornar 0 no sucesso
}                                                           // fecha funcao

static int hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) // stub read
{                                                           // abre funcao
    (void)addr;                                             // evita warning
    (void)reg;                                              // evita warning
    (void)data;                                             // evita warning
    (void)len;                                              // evita warning
    return -1;                                              // TODO: implementar e retornar 0 no sucesso
}                                                           // fecha funcao

static void hal_delay_ms(uint32_t ms)                        // stub delay
{                                                           // abre funcao
    (void)ms;                                               // evita warning
    // TODO: implementar delay real                           // comentario
}                                                           // fecha funcao
