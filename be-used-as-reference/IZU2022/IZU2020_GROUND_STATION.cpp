#include "mbed.h"

#include "PQES920LR.h"

#define TIME_LSB 54.931640625
#define VOLTAGE_LSB 1.25
#define CURRENT_LSB 1.0986328125
#define PRESS_LSB 0.0384521484375
#define TEMP_LSB 0.002593994140625
#define ACCEL_LSB 0.00048828125
#define GYRO_LSB 0.06103515625
#define MAG_LSB 0.146484375

Serial pc(USBTX, USBRX, 115200);
Serial es_serial(D1, D0, 115200);

ES920LR es(es_serial);

void prompt();

void downlink_handler(char* data);

char *phase_names[] =  {"SAFETY", "READY", "FLIGHT", "SEP", "EMERGENCY", "RECOVERY"};

char mission_timer_reset;
int mission_time;
int flight_time;
char f_sd;
char f_gps;
char f_adxl;
char f_ina_in;
char f_ina_ex;
char f_lps;
char f_mpu;
char f_mpu_ak;
char phase;
float lat;
float lon;
float alt;
float voltage_in;
float current_in;
float voltage_ex;
float current_ex;
float press;
float temp;
float accel[3];
float gyro[3];
float mag[3];

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

int mission_time_s;
int mission_hour, mission_min, mission_sec;
float voltage_in_V, voltage_ex_V;

int main()
{
    es.attach(downlink_handler);
    pc.printf("PLANET-Q GROUND STATION\r\n");

    while(1) {
        if(pc.readable()) {
            char command[1];
            command[0] = pc.getc() + 0x80;
            es.send(command, 1);
            pc.printf("send:\t%02x\r\n", command[0]);
        }
    }
}

void downlink_handler(char *data)
{
    mission_timer_reset = data[1];
    mission_time_bits = (short)(data[2] | data[3] << 8);
    flight_time_bits = (short)(data[4] | data[5] << 8);
    f_sd = data[6]     >> 7 & 1;
    f_gps = data[6]    >> 6 & 1;
    f_adxl = data[6]   >> 5 & 1;
    f_ina_in = data[6] >> 4 & 1;
    f_ina_ex = data[6] >> 3 & 1;
    f_lps = data[6]    >> 2 & 1;
    f_mpu = data[6]    >> 1 & 1;
    f_mpu_ak = data[6]      & 1;
    phase = data[7];
    lat = *(float*)&data[8];
    lon = *(float*)&data[12];
    alt = *(float*)&data[16];
    voltage_in_bits = (short)(data[20] | data[21] << 8);
    current_in_bits = (short)(data[22] | data[23] << 8);
    voltage_ex_bits = (short)(data[24] | data[25] << 8);
    current_ex_bits = (short)(data[26] | data[27] << 8);
    press_bits = (short)(data[28] | data[29] << 8);
    temp_bits = (short)(data[30] | data[31] << 8);
    accel_bits[0] = (short)(data[32] | data[33] << 8);
    accel_bits[1] = (short)(data[34] | data[35] << 8);
    accel_bits[2] = (short)(data[36] | data[37] << 8);
    gyro_bits[0] = (short)(data[38] | data[39] << 8);
    gyro_bits[1] = (short)(data[40] | data[41] << 8);
    gyro_bits[2] = (short)(data[42] | data[43] << 8);
    mag_bits[0] = (short)(data[44] | data[45] << 8);
    mag_bits[1] = (short)(data[46] | data[47] << 8);
    mag_bits[2] = (short)(data[48] | data[49] << 8);

    mission_time = mission_time_bits * TIME_LSB;
    flight_time = flight_time_bits * TIME_LSB;
    voltage_in = voltage_in_bits * VOLTAGE_LSB;
    current_in = current_in_bits * CURRENT_LSB;
    voltage_ex = voltage_ex_bits * VOLTAGE_LSB;
    current_ex = current_ex_bits * CURRENT_LSB;
    press = press_bits * PRESS_LSB;
    temp = temp_bits * TEMP_LSB;
    for(int i = 0; i < 3; i++) {
        accel[i] = accel_bits[i] * ACCEL_LSB;
    }
    for(int i = 0; i < 3; i++) {
        gyro[i] = gyro_bits[i] * GYRO_LSB;
    }
    for(int i = 0; i < 3; i++) {
        mag[i] = mag_bits[i] * MAG_LSB;
    }
    
    mission_time_s = (mission_time / 1000) + (mission_timer_reset * 30*60);
    mission_hour = mission_time_s / 3600;
    mission_time_s %= 3600;
    mission_min = mission_time_s / 60;
    mission_time_s %= 60;
    mission_sec = mission_time_s;
    
    voltage_in_V = voltage_in / 1000.0f;
    voltage_ex_V = voltage_ex / 1000.0f;
    
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
    pc.printf(" flags:\tSD\tGPS\tINA_in\tINA_ex\tADXL\tLPS\tMPU\tMPU_AK\r\n");
    pc.printf("       \t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", f_sd, f_gps, f_adxl, f_ina_in, f_ina_ex, f_lps, f_mpu, f_mpu_ak);
    pc.printf("\r\n");
    pc.printf(" lat:\t\t%.6f\r\n", lat);
    pc.printf(" lon:\t\t%.6f\r\n", lon);
    pc.printf(" alt:\t\t%.1f[m]\r\n", alt);
    pc.printf("\r\n");
    pc.printf(" internal:\tvoltage\t\tcurrent\r\n");
    pc.printf("          \t%.2f[V]\t\t%.0f[mA]\r\n", voltage_in_V, current_in);
    pc.printf(" external:\tvoltage\t\tcurrent\r\n");
    pc.printf("          \t%.2f[V]\t\t%.0f[mA]\r\n", voltage_ex_V, current_ex);
    pc.printf("\r\n");
    pc.printf(" press:\t\t%.2f[hPa]\r\n", press);
    pc.printf(" temp:\t\t%.2f[C]\r\n", temp);
    pc.printf("\r\n");
    pc.printf(" accel:\t\tx\t\ty\t\tz\r\n");
    pc.printf("       \t\t%.2f\t\t%.2f\t\t%.2f\t[G]\r\n", accel[0], accel[1], accel[2]);
    pc.printf(" gyro:\t\tx\t\ty\t\tz\r\n");
    pc.printf("       \t\t%.2f\t\t%.2f\t\t%.2f\t[rad/s]\r\n", gyro[0], gyro[1], gyro[2]);
    pc.printf(" mag:\t\tx\t\ty\t\tz\r\n");
    pc.printf("       \t\t%.2f\t\t%.2f\t\t%.2f\t[uT]\r\n", mag[0], mag[1], mag[2]);
}