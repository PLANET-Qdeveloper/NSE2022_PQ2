#ifndef PQ_ES920_H
#define PQ_ES920_H

/**
 * 3軸高加速度センサADXL375のライブラリ
 * @code
#include "mbed.h"
#include "PQ_ES920.h"

Serial pc(USBTX, USBRX, 115200);
Serial es_serial(p9, p10, 115200);

ES920 es(es_serial);

char data[52];

void command_handler(char *buff)
{
    for(int i = 0; i < 50; i++) {
        pc.printf("%02x ", data[i]);
    }
}

int main()
{
    es.attach(&command_handler);
    for(int i = 0; i < 50; i++) {
        data[i] = i;
    }
    while(1) {
        pc.printf("Send.\r\n");
        es.send(data, 50);
        wait(3);
    }
}
 * @endcode
 */
class ES920 {
private:
    Serial *_serial;
    char tx_buf[51];
    char rx_buf[51];
    char rx_size;
    int index;
    int flag;
    bool response;
    
    void (*func)(char*);
    
public:
    /**
     * @param serial Serialのインスタンスへの参照
     */
    ES920(Serial &serial);
    
    /**
     * データ送信
     * @param data 送信データを格納する配列
     * @param size 送信データのサイズ
     */
    void send(char *data, int size);
    
    /**
     * 受信割り込み設定
     * @param func_ptr 受信したときに実行する関数へのポインタ
     */
    void attach(void(*func_ptr)(char*));

private:
    void receive();
};

#endif