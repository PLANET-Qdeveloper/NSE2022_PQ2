#ifndef PQ_ADXL375_H
#define PQ_ADXL375_H

#define ADXL375_ADDR_HIGH 0b0011101<<1
#define ADXL375_ADDR_LOW 0b1010011<<1

#define ADXL375_DEVID 0x00
#define ADXL375_BW_RATE 0x2C
#define ADXL375_POWER_CTL 0x2D
#define ADXL375_DATAX0 0x32
#define ADXL375_LSB 0.049

/**
 * 3軸高加速度センサADXL375のライブラリ
 * @code
#include "mbed.h"
#include "PQ_ADXL375.h"

Serial pc(USBTX, USBRX, 115200);
I2C i2c(p9, p10);

ADXL375 adxl(i2c, ADXL375::ALT_ADDRESS_HIGH);

float high_accel_offset[] = {0, 0, 0};
float high_accel[3];

int main() {
    adxl.begin();
    adxl.offset(high_accel);
    if(adxl.test()){
        adxl.read(high_accel);
        pc.printf("%f\t%f\t%f\r\n", high_accel[0], high_accel[1], high_accel[2]);
    }
    else{
        pc.printf("ERROR!!\r\n");
    }
}
 * @endcode
 */
class ADXL375
{
public:
    typedef enum {
        ALT_ADDRESS_HIGH = ADXL375_ADDR_HIGH, ALT_ADDRESS_LOW = ADXL375_ADDR_LOW
    } ALT_ADDRESS_t;
private:
    I2C *_i2c;
    int _addr;
    char cmd[2];
    char buff[6];
    float high_accel_offset[3];
public:
    /**
     * @param i2c I2Cのインスタンスへの参照
     * @param AD0 ALT_ADDRESSピンのH/Lレベル
     */
    ADXL375(I2C &i2c, ALT_ADDRESS_t ALT_ADDRESS);
    
    
    /**
     * センサ動作開始
     */
    void begin();
    
    /**
     * センサ通信テスト
     * @retval true 通信成功
     * @retval false 通信失敗
     */
    bool test();
    
    /**
     * 加速度のゼロ点補正
     * @param high_accel 加速度のオフセット配列
     */
    void offset(float *high_accel);
    
    /**
     * 加速度測定値の読み取り
     * @param high_accel 加速度を格納する配列
     */
    void read(float *high_accel);
};

#endif