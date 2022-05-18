#ifndef PQ_GPS_H
#define PQ_GPS_H

/**
 * GPSのライブラリ
 * @code
#include "mbed.h"
#include "PQ_GPS.h"

Serial pc(USBTX, USBRX, 115200);
Serial gps_serial(p9, p10, 115200);

GPS gps(gps_serial);

int main()
{
    while(1) {
        pc.printf("time:%d:%d:%.3f, lat:%.6f, lon:%.6f, fix:%d, sat:%d, hdop:%.2f, alt:%.1f, geoid:%.1f\r\n", gps.get_hour(), gps.get_min(), gps.get_sec(), gps.get_lat(), gps.get_lon(), gps.get_fix(), gps.get_sat(), gps.get_hdop(), gps.get_alt(), gps.get_geoid());
    }
}
 * @endcode
 */
class GPS{
private:
    Serial *_serial;
    char rx_buf[256];
    int index;
    int flag;
    int hour;
    int min;
    float sec;
    float lat;
    char ns;
    float lon;
    char ew;
    int fix;
    int sat;
    float hdop;
    float alt;
    float geoid;
    
public:
    /**
     * @param gps Serialのインスタンスへの参照
     */
    GPS(Serial &gps);
    
private:
    void receive();

public:
    /**
     * UTC時刻の取得
     * @retval 時間
     */
    int get_hour();
    
    /**
     * UTC時刻の取得
     * @retval 分
     */
    int get_min();
    
    /**
     * UTC時刻の取得
     * @retval 秒
     */
    float get_sec();
    
    /**
     * 緯度の取得
     * @retval 緯度
     */
    float get_lat();
    
    /**
     * 経度の取得
     * @retval 経度
     */
    float get_lon();
    
    /**
     * 位置特定品質の取得
     * @retval 位置特定品質
     */
    int get_fix();
    
    /**
     * 使用衛星数の取得
     * @retval 使用衛星数
     */
    int get_sat();
    
    /**
     * 水平精度低下率の取得
     * @retval 水平精度低下率
     */
    float get_hdop();
    
    /**
     * 海抜高度の取得
     * @retval 海抜高度
     */
    float get_alt();
    
    /**
     * ジオイド高の取得
     * @retval ジオイド高
     */
    float get_geoid();
};

#endif