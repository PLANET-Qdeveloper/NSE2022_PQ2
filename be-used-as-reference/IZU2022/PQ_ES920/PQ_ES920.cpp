#include "mbed.h"
#include "PQ_ES920.h"

ES920::ES920(Serial &serial)
{
    _serial = &serial;
    _serial->attach(callback(this, &ES920::receive), Serial::RxIrq);
    memset(tx_buf, '\0', 51);    
    memset(rx_buf, '\0', 51);
    index = 0;
    flag = 0;
    response = false;
}

void ES920::send(char *data, int size)
{
    if(size > 50) {
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

void ES920::attach(void(*func_ptr)(char*))
{
    func = func_ptr;
}

void ES920::receive()
{
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