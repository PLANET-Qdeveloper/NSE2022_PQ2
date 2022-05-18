#ifndef PQ_LPS22HB_H
#define PQ_LPS22HB_H

#define LPS22HB_ADDR_HIGH 0b1011101<<1
#define LPS22HB_ADDR_LOW 0b1011100<<1
#define LPS22HB_WHO_AM_I 0x0F
#define LPS22HB_CTRL_REG1 0x10
#define LPS22HB_PRESS_XL 0x28
#define LPS22HB_TEMP_L 0x2B
#define LPS22HB_PRESS_LSB 4096.0
#define LPS22HB_TEMP_LSB 0.01

/** LPS22HBのライブラリ
 * @code
#include "mbed.h"
#include "PQ_LPS22HB.h"

Serial pc(USBTX, USBRX, 115200);

I2C i2c(p9, p10);

LPS22HB lps(i2c, LPS22HB::SA0_HIGH);

float press, temp;

int main() {
    lps.begin();
    while(1) {
        if(lps.test()){
            lps.read(&press, &temp);
            pc.printf("%f\t%f\r\n", press, temp);
        }
        else {
            pc.printf("ERROR!!\r\n");
        }
    }
}
 * @endcode
 */
class LPS22HB
{
public:
    typedef enum {
        SA0_HIGH = LPS22HB_ADDR_HIGH,
        SA0_LOW = LPS22HB_ADDR_LOW
    } SA0_t;

private:
    I2C *_i2c;
    int _addr;
    char cmd[2];
    char buff[6];

public:
    /**
     * @param i2c I2Cのインスタンスへの参照
     * @param SA0 SA0ピンのH/Lレベル
     */
    LPS22HB(I2C &i2c, SA0_t SA0);

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
     * 測定値の読み取り
     * @param press 気圧を格納する変数
     * @param temp 温度を格納する変数
     */
    void read(float *press, float *temp);

    /**
     * 気圧測定値の読み取り
     * @param press 気圧を格納する変数
     */
    void read_press(float *press);

    /**
     * 温度測定値の読み取り
     * @param temp 温度を格納する変数
     */
    void read_temp(float *temp);
};

#endif