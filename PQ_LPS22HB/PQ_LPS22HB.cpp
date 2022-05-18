#include "mbed.h"
#include "PQ_LPS22HB.h"

LPS22HB :: LPS22HB(I2C &i2c, SA0_t SA0)
{
    _addr = SA0;
    _i2c = &i2c;
    _i2c -> frequency(400000);
}

void LPS22HB :: begin()
{
    cmd[0] = LPS22HB_CTRL_REG1;
    cmd[1] = 0x40;
    _i2c -> write(_addr, cmd, 2);
}

bool LPS22HB :: test()
{
    cmd[0] = LPS22HB_WHO_AM_I;
    _i2c -> write(_addr, cmd, 1);
    _i2c -> read(_addr, buff, 1);
    if(buff[0] == 0xB1) {
        return true;
    } else {
        return false;
    }
}

void LPS22HB :: read(float *press, float *temp)
{
    read_press(press);
    read_temp(temp);
}

void LPS22HB :: read_press(float *press)
{
    cmd[0] = LPS22HB_PRESS_XL;
    _i2c -> write(_addr, cmd, 1);
    _i2c -> read(_addr, buff, 3);
    *press = (int)(buff[0] | buff[1] << 8 | buff[2] << 16) / LPS22HB_PRESS_LSB;
}

void LPS22HB :: read_temp(float *temp)
{
    cmd[0] = LPS22HB_TEMP_L;
    _i2c -> write(_addr, cmd, 1);
    _i2c -> read(_addr, buff, 2);
    *temp = (short)(buff[0] | buff[1] << 8) * LPS22HB_TEMP_LSB;
}