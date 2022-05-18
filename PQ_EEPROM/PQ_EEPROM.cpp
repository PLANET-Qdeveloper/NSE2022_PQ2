#include "mbed.h"
#include "PQ_EEPROM.h"

EEPROM::EEPROM(I2C &i2c)
{
    _i2c = &i2c;
    _i2c -> frequency(400000);
}

void EEPROM::write(int addr, char *data, int size)
{
    char B0 = (addr >> 16) & 0x01;
    char A0 = (addr >> 17) & 0x01;
    char A1 = (addr >> 18) & 0x01;
    _addr = 0b10100000 | B0 << 3 | A1 << 2 | A0 << 1;
    
    if(size > 128){
        size = 128;
    }
    cmd[0] = (addr >> 8) & 0xFF;
    cmd[1] = addr & 0xFF;
    for(int i = 0; i < size; i++){
        cmd[2 + i] = data[i];
    }
    _i2c -> write(_addr, cmd, 2 + size);
    
    wait_ms(3);
}

void EEPROM::read(int addr, char *data, int size)
{   
    char B0 = (addr >> 16) & 0x01;
    char A0 = (addr >> 17) & 0x01;
    char A1 = (addr >> 18) & 0x01;
    _addr = 0b10100000 | B0 << 3 | A1 << 2 | A0 << 1;
    
    cmd[0] = (addr >> 8) & 0xFF;
    cmd[1] = addr & 0xFF;
    
    _i2c -> write(_addr, cmd, 2);
    _i2c -> read(_addr, buff, size);
    
    for(int i = 0; i < size; i++){
        data[i] = buff[i];
    }
}