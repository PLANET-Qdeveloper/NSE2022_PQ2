#include "mbed.h"
#include "PQ_MPU9250.h"

MPU9250::MPU9250(I2C &i2c, AD0_t AD0)
{
    _i2c = &i2c;
    _addr = AD0;
    _i2c->frequency(400000);
}

void MPU9250::begin()
{
    cmd[0] = MPU9250_PWR_MGMT_1;
    cmd[1] = 0x00;
    _i2c->write(_addr, cmd, 2);
    cmd[0] = MPU9250_INT_PIN_CFG;
    cmd[1] = 0x02;
    _i2c->write(_addr, cmd, 2);
    set(_2G, _250DPS, _16B_8HZ);
}

bool MPU9250::test()
{
    cmd[0] = MPU9250_WHO_AM_I;
    _i2c->write(_addr, cmd, 1);
    _i2c->read(_addr, buff, 1);
    if (buff[0] == 0x71) {
        return true;
    } else {
        return false;
    }
}

bool MPU9250::test_AK8963()
{
    cmd[0] = MPU9250_WIA;
    _i2c->write(MPU9250_AK8963_ADDR, cmd, 1);
    _i2c->read(MPU9250_AK8963_ADDR, buff, 1);
    if (buff[0] == 0x48) {
        return true;
    } else {
        return false;
    }
}

void MPU9250::set(AccelConfig_t accel_config, GyroConfig_t gyro_config, MagConfig_t mag_config)
{
    set_accel(accel_config);
    set_gyro(gyro_config);
    set_mag(mag_config);
}

void MPU9250::set_accel(AccelConfig_t accel_config)
{
    cmd[0] = MPU9250_ACCEL_CONFIG;
    cmd[1] = accel_config;
    _i2c->write(_addr, cmd, 2);
    if (accel_config == _2G) {
        accel_LSB = MPU9250_ACCEL_LSB;
    } else if (accel_config == _4G) {
        accel_LSB = MPU9250_ACCEL_LSB * 2;
    } else if (accel_config == _8G) {
        accel_LSB = MPU9250_ACCEL_LSB * 4;
    } else if (accel_config == _16G) {
        accel_LSB = MPU9250_ACCEL_LSB * 8;
    }
}

void MPU9250::set_gyro(GyroConfig_t gyro_config)
{
    cmd[0] = MPU9250_GYRO_CONFIG;
    cmd[1] = gyro_config;
    _i2c->write(_addr, cmd, 2);
    if (gyro_config == _250DPS) {
        gyro_LSB = MPU9250_GYRO_LSB;
    } else if (gyro_config == _500DPS) {
        gyro_LSB = MPU9250_GYRO_LSB * 2;
    } else if (gyro_config == _1000DPS) {
        gyro_LSB = MPU9250_GYRO_LSB * 4;
    } else if (gyro_config == _2000DPS) {
        gyro_LSB = MPU9250_GYRO_LSB * 8;
    }
}

void MPU9250::set_mag(MagConfig_t mag_config)
{
    cmd[0] = MPU9250_CNTL1;
    cmd[1] = mag_config;
    _i2c->write(MPU9250_AK8963_ADDR, cmd, 2);
    mag_LSB = MPU9250_MAG_LSB;
}

void MPU9250::offset(float *accel, float *gyro, float *mag)
{
    offset_accel(accel);
    offset_gyro(gyro);
    offset_mag(mag);
}

void MPU9250::offset_accel(float *accel)
{
    for (int i = 0; i < 3; i++) {
        accel_offset[i] = accel[i];
    }
}

void MPU9250::offset_gyro(float *gyro)
{
    for (int i = 0; i < 3; i++) {
        gyro_offset[i] = gyro[i];
    }
}

void MPU9250::offset_mag(float *mag)
{
    for (int i = 0; i < 3; i++) {
        mag_offset[i] = mag[i];
    }
}

void MPU9250::read(float *accel, float *gyro, float *mag)
{
    read_accel(accel);
    read_gyro(gyro);
    read_mag(mag);
}

void MPU9250::read_accel(float *accel)
{
    cmd[0] = MPU9250_ACCEL_XOUT_H;
    _i2c->write(_addr, cmd, 1);
    _i2c->read(_addr, buff, 6);
    accel[0] = (short)(buff[0] << 8 | buff[1]) * accel_LSB - accel_offset[0];
    accel[1] = (short)(buff[2] << 8 | buff[3]) * accel_LSB - accel_offset[1];
    accel[2] = (short)(buff[4] << 8 | buff[5]) * accel_LSB - accel_offset[2];
}

void MPU9250::read_gyro(float *gyro)
{
    cmd[0] = MPU9250_GYRO_XOUT_H;
    _i2c->write(_addr, cmd, 1);
    _i2c->read(_addr, buff, 6);
    gyro[0] = (short)(buff[0] << 8 | buff[1]) * gyro_LSB - gyro_offset[0];
    gyro[1] = (short)(buff[2] << 8 | buff[3]) * gyro_LSB - gyro_offset[1];
    gyro[2] = (short)(buff[4] << 8 | buff[5]) * gyro_LSB - gyro_offset[2];
}

void MPU9250::read_mag(float *mag)
{
    cmd[0] = MPU9250_HXL;
    _i2c->write(MPU9250_AK8963_ADDR, cmd, 1);
    _i2c->read(MPU9250_AK8963_ADDR, buff, 7);
    mag[0] = (short)(buff[0] | buff[1] << 8) * mag_LSB - mag_offset[0];
    mag[1] = (short)(buff[2] | buff[3] << 8) * mag_LSB - mag_offset[1];
    mag[2] = (short)(buff[4] | buff[5] << 8) * mag_LSB - mag_offset[2];
}
