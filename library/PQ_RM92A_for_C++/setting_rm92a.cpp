#include "mbed.h"

Serial pc(USBTX, USBRX, 115200);
Serial rm_serial(D1, D0, 115200);

char c, r;

void receive(){
    r = rm_serial.getc();
    printf("%c", r);    
    return;
}

int main(){
    rm_serial.attach(receive, Serial::RxIrq);
    while(1){
        c = pc.getc();
        rm_serial.putc(c);
        wait(0.1);
    }
}