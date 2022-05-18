#ifndef PQ_MPU9250_H
#define PQ_MPU9250_H

#define MPU9250_ADDR_HIGH 0b1101001<<1
#define MPU9250_ADDR_LOW 0b1101000<<1
#define MPU9250_AK8963_ADDR 0b0001100<<1
#define MPU9250_WIA 0x00
#define MPU9250_ST1 0x02
#define MPU9250_HXL 0x03
#define MPU9250_CNTL1 0x0A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_ACCEL_LSB 0.0000610
#define MPU9250_GYRO_LSB 0.00763
#define MPU9250_MAG_LSB 0.150

/**
 * 9軸センサMPU9250のライブラリ
 * @code
#include "mbed.h"
#include "PQ_MPU9250.h"

Serial pc(USBTX, USBRX, 115200);
I2C i2c(p28, p27);

MPU9250 mpu(i2c, MPU9250::AD0_HIGH);

float accel_offset[] = {0, 0, 0};
float gyro_offset[] = {0, 0, 0};
float mag_offset[] = {0, 0, 0};
float accel[3];
float gyro[3];
float mag[3];

int main()
{
    mpu.begin();
    mpu.set(MPU9250::_2G, MPU9250::_250DPS, MPU9250::_16B_8HZ);
    mpu.offset(accel_offset, gyro_offset, mag_offset);
    while(1) {
        if(mpu.test()) {
            mpu.read_accel(accel);
            mpu.read_gyro(gyro);
        } else {
            pc.printf("ERROR!!\r\n");
        }
        if(mpu.test_AK8963()) {
            mpu.read_mag(mag);
        } else {
            pc.printf("ERROR!!\r\n");
        }
        pc.printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);
    }
}
 * @endcode
 */
class MPU9250
{
public:
    typedef enum {
        AD0_HIGH = MPU9250_ADDR_HIGH,
        AD0_LOW  = MPU9250_ADDR_LOW
    } AD0_t;

    typedef enum {
        _2G  = 0b00000000,
        _4G  = 0b00001000,
        _8G  = 0b00010000,
        _16G = 0b00011000
    } AccelConfig_t;

    typedef enum {
        _250DPS  = 0b00000000,
        _500DPS  = 0b00001000,
        _1000DPS = 0b00010000,
        _2000DPS = 0b00011000
    } GyroConfig_t;

    typedef enum {
        _16B_8HZ   = 0b00010010,
        _16B_100HZ = 0b00010110,
        _14B_8HZ   = 0b00000010,
        _14B_100HZ = 0b00000100
    } MagConfig_t;

private:
    I2C *_i2c;
    int _addr;
    char cmd[2];
    char buff[8];
    float accel_LSB;
    float gyro_LSB;
    float mag_LSB;
    float accel_offset[3];
    float gyro_offset[3];
    float mag_offset[3];
public:

    /**
     * @param i2c I2Cのインスタンスへの参照
     * @param AD0 AD0ピンのH/Lレベル
     */
    MPU9250(I2C &i2c, AD0_t AD0);

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
     * 磁気センサAK8963通信テスト
     * @retval true 通信成功
     * @retval false 通信失敗
     */
    bool test_AK8963();

    /**
     * センサ設定
     * @param accel_config 加速度の測定レンジ
     * @param gyro_config 角速度の測定レンジ
     * @param mag_config 磁気センサの分解能/データレート
     */
    void set(AccelConfig_t accel_config, GyroConfig_t gyro_config, MagConfig_t mag_config);

    /**
     * 加速度センサ設定
     * @param accel_config 角速度センサの測定レンジ
     */
    void set_accel(AccelConfig_t accel_config);

    /**
     * 角速度センサ設定
     * @param gyro_config 角速度の測定レンジ
     */
    void set_gyro(GyroConfig_t gyro_config);

    /**
     * 磁気センサ設定
     * @param mag_config 磁気センサの分解能/データレート
     */
    void set_mag(MagConfig_t mag_config);

    /**
     * ゼロ点補正
     * @param accel 加速度のオフセット配列
     * @param gyro 角速度のオフセット配列
     * @param mag 磁気のオフセット配列
     */
    void offset(float *accel, float *gyro, float *mag);

    /**
     * 加速度のゼロ点補正
     * @param accel 加速度のオフセット配列
     */
    void offset_accel(float *accel);

    /**
     * 角速度のゼロ点補正
     * @param gyro 角速度のオフセット配列
     */
    void offset_gyro(float *gyro);

    /**
     * 磁気のゼロ点補正
     * @param mag 磁気のオフセット配列
     */
    void offset_mag(float *mag);

    /**
     * 測定値の読み取り
     * @param accel 加速度を格納する配列
     * @param gyro 角速度を格納する配列
     * @param mag 磁気を格納する配列
     */
    void read(float *accel, float *gyro, float *mag);

    /**
     * 加速度測定値の読み取り
     * @param accel 加速度を格納する配列
     */
    void read_accel(float *accel);

    /**
     * 角速度測定値の読み取り
     * @param gyro 角速度を格納する配列
     */
    void read_gyro(float *gyro);

    /**
     * 磁気測定値の読み取り
     * @param mag 磁気を格納する配列
     */
    void read_mag(float *mag);
};

#endif