#include "mbed.h"
#include "PQ_INA226.h"

INA226::INA226(I2C &i2c, A0_t A0, A1_t A1)
{
    _i2c = &i2c;
    _addr = (0b1000000 | A1 << 2 | A0) << 1;
    _i2c->frequency(400000);
}

void INA226::begin(){
    cmd[0] = INA226_CALIBRATION;
    cmd[1] = 0x0A;
    cmd[2] = 0x00;
    _i2c->write(_addr, cmd, 3);
}

bool INA226::test()
{
    cmd[0] = INA226_WHO_AM_I;
    _i2c->write(_addr, cmd, 1);
    _i2c->read(_addr, buff, 1);
    if(buff[0] == 0x22) {
        return true;
    } else {
        return false;
    }
}

void INA226::read(float *voltage, float *current, float *power)
{
    read_voltage(voltage);
    read_current(current);
    read_power(power);
}

void INA226::read_voltage(float *voltage){
    cmd[0] = INA226_BUS_VOLTAGE;
    _i2c->write(_addr, cmd, 1);
    _i2c->read(_addr, buff, 2);
    *voltage = (short)(buff[0] << 8 | buff[1]) * INA226_VOLTAGE_LSB;
}

void INA226::read_current(float *current){
    cmd[0] = INA226_CURRENT;
    _i2c->write(_addr, cmd, 1);
    _i2c->read(_addr, buff, 2);
    *current = (short)(buff[0] << 8 | buff[1]) * INA226_CURRENT_LSB;
}

void INA226::read_power(float *power)
{
    cmd[0] = INA226_POWER;
    _i2c->write(_addr, cmd, 1);
    _i2c->read(_addr, buff, 2);
    *power = (short)(buff[0] << 8 | buff[1]) * INA226_POWER_LSB;
}