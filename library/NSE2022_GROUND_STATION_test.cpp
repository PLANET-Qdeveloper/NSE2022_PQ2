#include "mbed.h"

#include "PQ_RM92A.h"

Serial pc(USBTX, USBRX, 115200);
Serial rm_serial(D1, D0, 115200);

RM92A rm(rm_serial);

void downlink_handler(char* data);

char *phase_names[] =  {"SAFETY", "READY", "FLIGHT", "SEP", "EMERGENCY", "RECOVERY"};

char mission_timer_reset;
int mission_time;
int flight_time;
char phase;
float lat;
float lon;
float alt;
float press;
float temperature;

short mission_time_bits;
short flight_time_bits;
short voltage_in_bits;
short current_in_bits;
short voltage_ex_bits;
short current_ex_bits;
short press_bits;
short temp_bits;
short accel_bits[3];
short gyro_bits[3];
short mag_bits[3];

int main(){
    rm.attach(downlink_handler);
    pc.printf("PLANET-Q GROUND STATION\r\n");

    while(1){
        if(pc.readable()){
            char cmd[1];
            cmd[0] = pc.getc() + 0x80
            rm.send(0x1234, cmd, 1);
            pc.printf("send:\t%02x\r\n", cmd[0]);
        }
    }
}

void downlink_handler(char *data){

    /*
    send_data = [0]*50
    send_data[] = mission_time_bits_A
    send_data[] = mission_time_bits_B
    send_data[] = mission_timer_reset
    send_data[] = flight_time_bits_A
    send_data[] = flight_time_bits_B
    send_data[] = phase
    send_data[] = launched
    send_data[] = landed
    #send_data[] = relay.value()
    send_data[] = sep_pin.value()
    send_data[] = apogee
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
    send_data[] = alt_bits_A
    send_data[] = alt_bits_B
    #send_data[] = 
    */

   mission_time_bits_
    mission_timer_reset = rcv_data[2]

    pc.printf(" PLANET-Q GROUND STATION\r\n");
    pc.printf(" commands:\r\n"); 
    pc.printf(" 0   : SAFETY\r\n");
    pc.printf(" 1   : WAIT\r\n");
    pc.printf(" 2   : FLIGHT\r\n");
    pc.printf(" 3   : SEP\r\n");
    pc.printf(" 4   : EMERGENCY\r\n");
    pc.printf(" 5   : RECOVERY\r\n");
    pc.printf(" DEL : SYSTEM RESET\r\n");
    pc.printf("\r\n");
    pc.printf(" mission time:\t%02d:%02d:%02d\r\n", mission_hour, mission_min, mission_sec);
    pc.printf(" flight time:\t%d[ms]\r\n", flight_time);
    pc.printf(" phase:\t%s\r\n", phase_names[phase]);
    pc.printf(" lat:\t\t%.6f\r\n", lat);
    pc.printf(" lon:\t\t%.6f\r\n", lon);
    pc.printf(" press:\t\t%.2f[hPa]\r\n", press);
    pc.printf(" temp:\t\t%.2f[C]\r\n", temprature);
    }

