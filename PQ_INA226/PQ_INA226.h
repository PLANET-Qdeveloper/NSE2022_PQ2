#ifndef PQ_INA226_H
#define PQ_INA226_H

#define INA226_BUS_VOLTAGE 0x02
#define INA226_POWER 0x03
#define INA226_CURRENT 0x04
#define INA226_CALIBRATION 0x05
#define INA226_WHO_AM_I 0xFF
#define INA226_VOLTAGE_LSB 1.25
#define INA226_CURRENT_LSB 1.25
#define INA226_POWER_LSB 25

/** 電圧電流センサINA226のライブラリ（シャント抵抗0.002Ω）
 * @code
#include "mbed.h"
#include "PQ_INA226.h"

Serial pc(USBTX, USBRX, 115200);
I2C i2c(p9, p10);

INA226 ina(i2c, INA226::A0_GND, INA226::A1_GND);

float voltage, current, power;

int main()
{
    ina.begin();
    while(true) {
        if(ina.test()) {
            ina.read(&voltage, &current, &power);
            pc.printf("%f\t%f\t%f\r\n", voltage, current, power);
        } else {
            pc.printf("ERROR!!\r\n");
        }
    }
}
 * @endcode
 */
class INA226
{
public:
    typedef enum A0 {
        A0_GND = 0b00,
        A0_VS  = 0b01,
        A0_SDA = 0b10,
        A0_SCL = 0b11
    } A0_t;

    typedef enum A1 {
        A1_GND = 0b00,
        A1_VS  = 0b01,
        A1_SDA = 0b10,
        A1_SCL = 0b11
    } A1_t;
    
private:
    I2C *_i2c;
    int _addr;
    char cmd[3];
    char buff[2];
    
public:
    /**
     * @param i2c I2Cのインスタンスへの参照
     * @param A0 A0のジャンパ
     * @param A1 A1のジャンパ
     */
    INA226(I2C &i2c, A0_t A0, A1_t A1);

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
     * 測定値読み取り
     * @param voltage 電圧を格納する変数へのポインタ
     * @param current 電流を格納する変数へのポインタ
     * @param power 電力を格納する変数へのポインタ
     */
    void read(float *voltage, float *current, float *power);

    /**
     * 電圧測定値読み取り
     * @param voltage 電圧を格納する変数へのポインタ
     */
    void read_voltage(float *voltage);

    /**
     * 電流測定値読み取り
     * @param current 電流を格納する変数へのポインタ
     */
    void read_current(float *current);

    /**
     * 電力測定値読み取り
     * @param power 電力を格納する変数へのポインタ
     */
    void read_power(float *power);
};

#endif