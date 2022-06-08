#include "mbed.h"
#include "PQ_RM92A.h"

RM92::RM92(Serial &serial) {
    _serial = &serial;
    _serial -> attach(callback(this, &RM92::receive), Serial::RxIrq);
    memset(tx_buf, '\0', 51);    
    memset(rx_buf, '\0', 51);
    index = 0;
    flag = 0;
    response = false;
}

void RM92::send(char *data, int size) {
    if(size > 50){
        size = 50;
    }

    tx_buf[0] = size;
    for(int i = 0; i < size; i++) {
        tx_buf[1 + i] = data[i];
    }
    for (int i = 0; i < 1 + size; i++) {
        _serial->putc(tx_buf[i]);
    }
    
    flag = 0;
    response = true;
}

void RM92::attach(void(*func_ptr)(char*)) {
    func = func_ptr;
}

void RM92::receive() {
    if(flag == 0) {
        rx_size = _serial->getc();
        memset(rx_buf, '\0', 51);
        index = 0;
        flag = 1;
    }
    if(flag == 1) {
        rx_buf[index] = _serial->getc();
        if(index == rx_size - 1) {
            if(!response) {
                if(func != NULL) {
                    (*func)(rx_buf);
                }
            } else {
                response = false;
            }
            flag = 0;
        } else {
            index ++;
        }
    }
}