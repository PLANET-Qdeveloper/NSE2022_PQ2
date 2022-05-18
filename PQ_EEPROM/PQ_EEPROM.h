#ifndef PQ_EEPROM_H
#define PQ_EEPROM_H

/**
 * 3軸高加速度センサADXL375のライブラリ
 * @code
#include "mbed.h"
#include "PQ_EEPROM.h"

Serial pc(USBTX, USBRX, 115200);

I2C i2c(p28, p27);

EEPROM eeprom(i2c);

int addr;
char data[512];

int main()
{
    addr = 0x0000;
    for(int i = 0; i < 128; i++) {
        data[i] = i;
    }
    eeprom.write(addr, data, 128);
    
    eeprom.read(addr, data, 128);
    for(int i = 0; i < 128; i++) {
        pc.printf("%02x ", data[i]);
    }
}
 * @endcode
 */
class EEPROM
{
private:
    I2C *_i2c;
    int _addr;
    char cmd[130];
    char buff[128];
public:
    /**
     * @param i2c I2Cのインスタンスへの参照
     */
    EEPROM(I2C &i2c);
    
    /**
     * 連続書き込み
     * @param addr 書き込み開始アドレス
     * @param data 書き込むデータを格納する配列
     * @param size 書き込むデータのサイズ
     */
    void write(int addr, char *data, int size);
    
    /**
     * 連続読み出し
     * @param addr 読み出し開始アドレス
     * @param data 読みだすデータを格納する配列
     * @param size 読みだすデータのサイズ
     */
    void read(int addr, char *data, int size);
};

#endif