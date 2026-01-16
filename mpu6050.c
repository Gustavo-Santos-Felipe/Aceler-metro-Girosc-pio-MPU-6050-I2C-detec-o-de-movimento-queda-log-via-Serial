#include "mpu6050.h"                              // inclui header do driver

static int write_reg(mpu6050_bus_t *bus, uint8_t reg, uint8_t val) // helper: escreve 1 byte
{                                                // abre funcao
    return bus->write(bus->addr, reg, &val, 1);   // chama write do usuario
}                                                // fecha funcao

int mpu6050_whoami(mpu6050_bus_t *bus, uint8_t *id) // le WHO_AM_I
{                                                // abre funcao
    if (bus == 0 || id == 0) return -1;           // valida ponteiros
    return bus->read(bus->addr, MPU6050_REG_WHO_AM_I, id, 1); // le 1 byte
}                                                // fecha funcao

int mpu6050_init(mpu6050_bus_t *bus, mpu6050_accel_scale_t a, mpu6050_gyro_scale_t g) // init basico
{                                                // abre funcao
    uint8_t id = 0;                               // variavel para WHO_AM_I
    if (bus == 0) return -1;                      // valida ponteiro
    if (bus->write == 0 || bus->read == 0) return -2; // valida funcoes

    if (mpu6050_whoami(bus, &id) != 0) return -3;  // tenta ler WHO_AM_I
    if (id != 0x68 && id != 0x69) return -4;       // valida id esperado (depende do silicio/modulo)

    if (write_reg(bus, MPU6050_REG_PWR_MGMT_1, 0x00) != 0) return -5; // acorda (clear sleep)
    if (write_reg(bus, MPU6050_REG_ACCEL_CFG, (uint8_t)(a << 3)) != 0) return -6; // seta escala accel
    if (write_reg(bus, MPU6050_REG_GYRO_CFG,  (uint8_t)(g << 3)) != 0) return -7; // seta escala gyro

    return 0;                                     // sucesso
}                                                // fecha funcao

int mpu6050_read_raw(mpu6050_bus_t *bus, mpu6050_raw_t *out) // le accel/gyro em burst
{                                                // abre funcao
    uint8_t buf[14];                              // accel(6) temp(2) gyro(6) = 14 bytes
    if (bus == 0 || out == 0) return -1;          // valida ponteiros

    if (bus->read(bus->addr, MPU6050_REG_ACCEL_XH, buf, 14) != 0) return -2; // le burst

    out->ax = (int16_t)((buf[0] << 8) | buf[1]);  // AX
    out->ay = (int16_t)((buf[2] << 8) | buf[3]);  // AY
    out->az = (int16_t)((buf[4] << 8) | buf[5]);  // AZ
    out->gx = (int16_t)((buf[8] << 8) | buf[9]);  // GX (pula temp em [6..7])
    out->gy = (int16_t)((buf[10] << 8) | buf[11]); // GY
    out->gz = (int16_t)((buf[12] << 8) | buf[13]); // GZ

    return 0;                                     // sucesso
}                                                // fecha funcao
