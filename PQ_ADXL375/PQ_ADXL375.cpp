#include "mbed.h"
#include "PQ_ADXL375.h"

ADXL375 :: ADXL375(I2C &i2c, ALT_ADDRESS_t ALT_ADDRESS)
{
    _addr = ALT_ADDRESS;
    _i2c = &i2c;
    _i2c->frequency(400000);
}

void ADXL375 :: begin()
{
    cmd[0] = ADXL375_BW_RATE;
    cmd[1] = 0x0F;
    _i2c->write(_addr, cmd, 2);
    
    cmd[0] = ADXL375_POWER_CTL;
    cmd[1] = 0x08;
    _i2c->write(_addr, cmd, 2);
}

bool ADXL375 :: test()
{
    cmd[0] = ADXL375_DEVID;
    _i2c->write(_addr, cmd, 1);
    _i2c->read(_addr, buff, 1);
    if (buff[0] == 0xE5) {
        return true;
    } else {
        return false;
    }
}

void ADXL375 :: offset(float *high_accel)
{
    for(int i = 0; i < 3; i++) {
        high_accel_offset[i] = high_accel[i];
    }
}

void ADXL375 :: read(float *high_accel)
{
    cmd[0] = ADXL375_DATAX0;
    _i2c->write(_addr, cmd, 1);
    _i2c->read(_addr, buff, 6);
    high_accel[0] = (short)(buff[1] << 8 | buff[0]) * ADXL375_LSB - high_accel_offset[0];
    high_accel[1] = (short)(buff[3] << 8 | buff[2]) * ADXL375_LSB - high_accel_offset[1];
    high_accel[2] = (short)(buff[5] << 8 | buff[4]) * ADXL375_LSB - high_accel_offset[2];
}