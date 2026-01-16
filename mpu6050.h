#ifndef MPU6050_H                              // guarda de include
#define MPU6050_H                              // guarda de include

#include <stdint.h>                            // tipos uint8_t/uint16_t

#define MPU6050_I2C_ADDR_0x68  (0x68)          // endereco padrao quando AD0=0
#define MPU6050_I2C_ADDR_0x69  (0x69)          // endereco quando AD0=1

#define MPU6050_REG_WHO_AM_I   (0x75)          // registrador WHO_AM_I
#define MPU6050_REG_PWR_MGMT_1 (0x6B)          // gerenciamento de energia
#define MPU6050_REG_ACCEL_CFG  (0x1C)          // configuracao do acelerometro
#define MPU6050_REG_GYRO_CFG   (0x1B)          // configuracao do giroscopio
#define MPU6050_REG_ACCEL_XH   (0x3B)          // inicio do bloco de leitura (ACCEL_XOUT_H)

typedef struct                                  // estrutura com leituras brutas
{                                                // abre struct
    int16_t ax;                                  // aceleracao x bruta
    int16_t ay;                                  // aceleracao y bruta
    int16_t az;                                  // aceleracao z bruta
    int16_t gx;                                  // giro x bruto
    int16_t gy;                                  // giro y bruto
    int16_t gz;                                  // giro z bruto
} mpu6050_raw_t;                                 // nome do tipo

typedef enum                                     // escalas do acelerometro
{                                                // abre enum
    MPU6050_ACCEL_2G  = 0,                       // ±2g
    MPU6050_ACCEL_4G  = 1,                       // ±4g
    MPU6050_ACCEL_8G  = 2,                       // ±8g
    MPU6050_ACCEL_16G = 3                        // ±16g
} mpu6050_accel_scale_t;                         // nome do tipo

typedef enum                                     // escalas do giroscopio
{                                                // abre enum
    MPU6050_GYRO_250DPS  = 0,                    // ±250 °/s
    MPU6050_GYRO_500DPS  = 1,                    // ±500 °/s
    MPU6050_GYRO_1000DPS = 2,                    // ±1000 °/s
    MPU6050_GYRO_2000DPS = 3                     // ±2000 °/s
} mpu6050_gyro_scale_t;                          // nome do tipo

typedef int (*mpu_i2c_write_f)(uint8_t, uint8_t, const uint8_t*, uint16_t); // funcao de escrita I2C
typedef int (*mpu_i2c_read_f )(uint8_t, uint8_t,       uint8_t*, uint16_t); // funcao de leitura I2C

typedef struct                                  // interface para plugar seu I2C do MCU
{                                                // abre struct
    uint8_t addr;                                // endereco I2C do sensor
    mpu_i2c_write_f write;                        // ponteiro para funcao write
    mpu_i2c_read_f  read;                         // ponteiro para funcao read
} mpu6050_bus_t;                                 // nome do tipo

int mpu6050_init(mpu6050_bus_t *bus, mpu6050_accel_scale_t a, mpu6050_gyro_scale_t g); // init
int mpu6050_whoami(mpu6050_bus_t *bus, uint8_t *id);                                   // le WHO_AM_I
int mpu6050_read_raw(mpu6050_bus_t *bus, mpu6050_raw_t *out);                           // le bloco bruto

#endif                                           // fecha guarda
