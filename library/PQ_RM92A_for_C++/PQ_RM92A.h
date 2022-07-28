#ifndef PQ_RM92_H
#define PQ_RM92_H

class RM92 {
    private:
        Serial *_serial;
        char c;
        char tx_buf[8];
        char rx_buf[51];
        char rx_size;
        int index;
        int first_flag;
        bool response;
        
        void (*func)(char*);

    public:
        RM92(Serial &serial);

        void send_cmd(int dst, char *cmd);

        void attach(void(*func_ptr)(char*));

    private:
        void receive();
};

#endif