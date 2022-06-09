#include "mbed.h"
#include "PQ_RM92A.h"

RM92::RM92(Serial &serial) {
    _serial = &serial;
    memset(tx_buf, '\0', 8);    
    memset(rx_buf, '\0', 51);
    index = 0;
    response = false;
    //Timer _rm92a_timer;
    //_rm92a_timer.start();
    wait(0.1);
    _serial -> attach(callback(this, &RM92::receive), Serial::RxIrq);   // 受信割り込みを登録
}

void RM92::send_cmd(int dst, char *cmd) {
    tx_buf[0] = '@';
    tx_buf[1] = '@';
    tx_buf[3] = 1;
    tx_buf[4] = dst << 8 & 0xff;
    tx_buf[5] = dst << 0 & 0xff;
    tx_buf[6] = cmd[0];
    tx_buf[7] = 0xff;
    for (int i = 0; i < 8; i++) {
        _serial->putc(tx_buf[i]);
    }
    response = true;
}

// データ処理関数を設定
void RM92::attach(void(*func_ptr)(char*)) {
    func = func_ptr;
}

void RM92::receive() {
    /*
    first_flagが1になる条件は
        1. 初めてreceive()が呼ばれた時
        2. recieve()が呼ばれる間隔が一定時間以上になった時
        3. 1パケットのデータがすべて取得できた時
    */

    //static int call_time[2] = {0, 0};

    c = _serial->getc();   // 1byte取得これは毎回必ず呼ばれる

/*
    call_time[1] = _rm92a_timer.read_us(); // 現在時刻を取得
    if(abs(call_time[1] - call_time[0]) > RM92A_DATA_INTERVAL){ // 前の呼び出しとのΔtから新しいデータかどうか判断
        first_flag = 1;
    }
    call_time[0] = call_time[1] // 時刻の更新
*/  
    if(first_flag){
        rx_size = (unsigned)c;
        first_flag = 0;
        index = 0;
        memset(rx_buf, '\0', 51);
        return;
    }else{
        rx_buf[index] = c;
        if(index == rx_size){ // 1パケット取得完了
            first_flag = 1;
            if(!response){
                if(func != NULL){
                    (*func)(rx_buf);
                }
            }else{
                response = false;
            }
        }
        index++;
        return;
    }
}