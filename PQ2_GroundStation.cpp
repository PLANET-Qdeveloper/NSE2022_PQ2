#include "mbed.h"
#include "PQ_RM92A.h"

Serial pc(USBTX, USBRX, 115200);
Serial rm_serial(D1, D0, 115200);

RM92A rm(rm_serial);

void downlink_handler(char* data);

char mission_timer_reset;
short mission_time_bits;
int mission_time;
int flight_time;
char phase;
char flags;

float lat10000;
float lon10000;
float press100;
float temperature;

int main(){
    rm.attach(downlink_handler);
    pc.printf("PLANET-Q GROUND STATION\r\n");

    while(1){
        if(pc.readable()){
            char cmd[1];
            cmd[0] = pc.getc();
            rm.send(0x1234, cmd, 1);
            pc.printf("send:\t%x\r\n", cmd[0]);
        }
    }
}

void downlink_handler(char* data){
    /*
    send_data = [0]*50
    send_data[0] = mission_time_reset
    send_data[1] = mission_time_bits_A
    send_data[2] = mission_time_bits_B
    send_data[3] = flight_time_bits_A
    send_data[4] = flight_time_bits_B
    send_data[5] = phase
    send_data[6] = flags
    send_data[] = press_bits_A
    send_data[] = press_bits_B
    send_data[] = press_bits_C
    send_data[] = temp_bits
    send_data[] = lat_bits_A
    send_data[] = lat_bits_B
    send_data[] = lat_bits_C
    send_data[] = lon_bits_A
    send_data[] = lon_bits_B
    send_data[] = lon_bits_C
    #send_data[] = 
    */
    
    mission_timer_reset = data[0];
    mission_time_bits = short(data[1] << 8) | (data[2] << 0);
    flight_time = short(data[3] << 8) | (data[4] << 0);

    phase = data[5];    // 0:SAFETY, 1:READY, ...
    flags = data[6];    // f_pin, launched, burned, apogee, separate, landed

    lat10000 = int(data[7] << 16) | (data[8] << 8) | (data[9] << 0);
    lon10000 = int(data[10] << 16) | (data[11] << 8) | (data[12] << 0);

    press100 = int(data[13] << 16) | (data[14] << 8) | (data[15] << 0);
    temperature = int(data[16]);

    mission_time = mission_time_bits + 60*60*mission_timer_reset
    
    pc.printf(" PLANET-Q GROUND STATION\r\n");
    pc.printf(" commands:\r\n"); 
    pc.printf(" 0   : SAFETY\r\n");
    pc.printf(" 1   : READY\r\n");
    pc.printf(" 2   : FLIGHT\r\n");
    pc.printf(" 3   : SEP\r\n");
    pc.printf(" 4   : EMERGENCY\r\n");
    pc.printf(" 5   : RECOVERY\r\n");
    pc.printf(" DEL : SYSTEM RESET\r\n");
    pc.printf("\r\n");
    pc.printf(" mission time:\t%d", mission_time);
    pc.printf(" flight time:\t%d[ms]\r\n", flight_time);
    pc.printf(" phase:\t%d\r\n", phase);
    pc.printf("\r\n");
    pc.printf(" lat:\t\t%.6f\r\n", lat10000/10000);
    pc.printf(" lon:\t\t%.6f\r\n", lon10000/10000);
    pc.printf("\r\n");
    pc.printf(" press:\t\t%.2f[hPa]\r\n", press100/100);
    pc.printf(" temp:\t\t%.2f[C]\r\n", temperature);
    pc.printf("\r\n");
}
