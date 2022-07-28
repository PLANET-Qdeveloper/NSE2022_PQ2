#include "mbed.h"
#include "PQ_RM92A.h"

Serial pc(USBTX, USBRX, 115200);
Serial rm_serial(D1, D0, 115200);

RM92 rm(rm_serial);

void downlink_handler(char* data);

char mission_timer_reset;
short mission_time_bits;
int mission_time;
int flight_time;
char phase;
char flags;
char flags2;

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
            rm.send_cmd(0x0000, cmd);
            pc.printf("send:\t%x\r\n", cmd[0]);
        }
    }
}

void downlink_handler(char* data){  
    /*
    press_int = int(pressure*100)  # 下2桁までを繰り上げして型変換
    press_bits_A = press_int >> 16 & 0xff
    press_bits_B = press_int >> 8  & 0xff
    press_bits_C = press_int >> 0  & 0xff

    temp_int = int(temperature) # 小数点以下は切り捨て
    temp_bits = temp_int >> 0 & 0xff

    lat_int = int(lat*10000)    # 下4桁までを繰り上げ
    lat_bits_A = lat_int >> 16 & 0xff 
    lat_bits_B = lat_int >> 8  & 0xff
    lat_bits_C = lat_int >> 0  & 0xff
    lon_int = int(lon*10000)    # 下4桁までを繰り上げ
    lon_bits_A = lon_int >> 16 & 0xff 
    lon_bits_B = lon_int >> 8  & 0xff
    lon_bits_C = lon_int >> 0  & 0xff
    send_data = bytearray(16)
    send_data[0] = 0x44   # Header
    send_data[1] = mission_timer_reset
    send_data[2] = mission_time_int
    send_data[3] = flight_time_int
    send_data[4] = phase
    send_data[5] = flags
    send_data[6] = press_bits_A
    send_data[7] = press_bits_B
    send_data[8] = press_bits_C
    send_data[9] = temp_bits 
    send_data[10] = lat_bits_A
    send_data[11] = lat_bits_B
    send_data[12] = lat_bits_C
    send_data[13] = lon_bits_A
    send_data[14] = lon_bits_B
    send_data[15] = lon_bits_C
    */
    mission_timer_reset = data[7];
    mission_time_bits = data[8];
    mission_time = mission_time_bits + 180*mission_timer_reset;
    flight_time = data[9];
    phase = data[10];
    flags = data[11];
    char flight_pin = data[11] >> 7 & 1;
    char burning     = data[11] >> 6 & 1;
    char apogee     = data[11] >> 5 & 1;
    char separated     = data[11] >> 4 & 1;
    char sep_pin    = data[11] >> 3 & 1;
    char landed     = data[11] >> 2 & 1;
    
    press100 = int((data[12] << 16) | (data[13] << 8) | (data[14] << 0));
    temperature = int(data[15]);
    lat10000 = int(data[16] << 16) | (data[17] << 8) | (data[18] << 0);
    lon10000 = int(data[19] << 16) | (data[20] << 8) | (data[21] << 0);
    
    pc.printf(" ========== PLANET-Q GROUND STATION ==========\r\n");
    pc.printf("\r\n");
    pc.printf(" The commands are as follows...\r\n"); 
    pc.printf(" 0   : SAFETY\r\n");
    pc.printf(" 1   : READY\r\n");
    pc.printf(" 2   : FLIGHT\r\n");
    pc.printf(" 3   : SEP\r\n");
    pc.printf(" 4   : EMERGENCY\r\n");
    pc.printf(" 5   : RECOVERY\r\n");
    pc.printf(" DEL : SYSTEM RESET\r\n");
    pc.printf("\r\n");
    pc.printf(" MISSION TIME:\t%d[s]", mission_time);
    pc.printf(" FLIGHT TIME:\t%d[s]\r\n", flight_time);
    pc.printf(" PHASE:\t%d\r\n", phase);
    pc.printf(" flags:\tF_PIN\tBURNING\tAPOGEE\tSEPARATED\tSEP_PIN\tLANDED\r\n");
    pc.printf("       \t%d\t%d\t%d\t%d\t\t%d\t%d\r\n", flight_pin, burning, apogee, separated, sep_pin, landed);
    pc.printf("\r\n");
    /*
    pc.printf(" LAT:\t\t%.6f\r\n", lat10000/10000);
    pc.printf(" LON:\t\t%.6f\r\n", lon10000/10000);
    pc.printf("\r\n");
    pc.printf(" PRESSURE:\t%.2f[hPa]\r\n", press100/100);
    pc.printf(" TEMPERATURE:\t%.2f[C]\r\n", temperature);
    pc.printf("\r\n");
    */
}
