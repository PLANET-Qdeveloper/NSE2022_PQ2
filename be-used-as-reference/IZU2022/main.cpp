/**************************************************
* 2021年度伊豆大島宇宙イベント電装系メインプログラム *
***************************************************/
#include <mbed.h>
#include <stdio.h>

#include "SDFileSystem.h"
#include "PQ_GPS.h"
#include "PQ_MPU9250.h"
#include "PQ_LPS22HB.h"
#include "PQ_INA226.h"
#include "PQ_ES920.h"
#include "PQ_EEPROM.h"
#include "PQ_ADXL375.h"

/*******************************************************************************
 定数宣言
*******************************************************************************/
#define BURN_TIME       2400.0f       // エンジン燃焼時間[ms]
#define T_APOGEE        1000000.0f    // FLIGHTからSEPまでの制限時間[ms]
#define T_HEATING       5170.0f       // ニクロム線加熱時間[ms]
#define T_RELAY_OFF     50000.0f     // FLIGHTからRECOVERYまでの時間[ms]
#define coef            0.1f

// データ圧縮率
#define TIME_LSB    54.931
#define VOLTAGE_LSB 1.25    // 1.25mV
#define CURRENT_LSB 1.25    // 1.25mA
#define PRESS_LSB   4096
#define TEMP_LSB    0.01
#define ADXL375_LSB 0.049           // 49mG
#define ACCEL_LSB   0.000488   // 1LSB(+-16G)
#define GYRO_LSB    0.061035
//#define MPU9250_MAG_LSB     0.146484375

/*******************************************************************************
 関数のプロトタイプ宣言. main()より後で使うので予め宣言する.
*******************************************************************************/
void init();
void read();
void smoothing();
void peak_detection();
void record();
void downlink();
void command_handler(char* command);
void print();
void buzzer();

/*******************************************************************************
 プロトタイプ宣言
*******************************************************************************/
Serial pc(USBTX, USBRX, 115200);    // パソコンの画面に映すためのもの
Serial es_serial(p9, p10, 115200);
Serial gps_serial(p13, p14, 115200);

I2C i2c(p28, p27);  //SDA, SCL

ES920 es(es_serial);
GPS gps(gps_serial);

SDFileSystem sd(p5, p6, p7, p8, "sd");  //MOSI,MISO,SCK,CS
EEPROM eeprom(i2c); // 24FC1025*4個（0x000000~0x07FFFF）
ADXL375 adxl(i2c, ADXL375::ALT_ADDRESS_HIGH); // 3軸高加速度
LPS22HB lps(i2c, LPS22HB::SA0_HIGH); // 気圧，温度
MPU9250 mpu(i2c, MPU9250::AD0_HIGH); // 3軸加速度，3軸角速度，3軸地磁気
INA226 ina_in(i2c, INA226::A0_VS, INA226::A1_VS);   // 電圧，電流（バッテリー）
INA226 ina_ex(i2c, INA226::A0_GND, INA226::A1_GND); // 電圧，電流（外部電源）

Timer mission_timer; // 電源投入からの時間[s]
Timer sd_timer;
Timer flight_timer;  // 離床検知からの時間[s]
Timer sep1_timer;     // 1段目の分離用のタイマー
Timer sep2_timer;    // 2段目の分離用のタイマー

Ticker downlink_ticker;
Ticker record_ticker;
Ticker smoothing_ticker;
Ticker peak_detection_ticker;
Ticker print_ticker;
Ticker buzzer_ticker;

DigitalIn flight_pin(p20);  // 離床検知用フライトピン
DigitalOut sep1(p24);       // 1段目のニクロム線制御
DigitalOut sep2(p25);       // 2段目のニクロム線制御
DigitalOut relay(p26);      // リレーのオンオフ
DigitalOut buzzer_pin(p21);

/*******************************************************************************
 変数宣言
*******************************************************************************/
bool initialized = false;
bool launched = false;          // フライトピンが抜けたか？
bool burning = false;           // エンジンが燃焼中か？
bool apogee = false;            // 気圧による頂点検知したか？
bool first_separated  = false;  // 1段目が作動したか？
bool second_separated = false;  // 2段目が作動したか？
bool landed = false;            // 着地したか？
bool SEP2_NG = false;           // 2段目作動させるか？

enum {
    SAFETY,      // 動作無し（コマンドでREADYへ移行可）
    READY,       // 離床検知待機（コマンドでSAFETY，FLIGHTへ移行可）
    FLIGHT,      // 離床検知（フライトピン）から分離まで（コマンドでSEP，EMERGENCYへ移行可）
    SEP1,        // 1段目作動フェーズ
    SEP2,        // 2段目作動フェーズ
    EMERGENCY,   // 離床失敗した場合の緊急停止（パラシュート分離を防ぐ. コマンドによる）
    RECOVERY     // 分離終了から回収まで
} phase;

char *phase_names[] =  {"SAFETY", "READY", "FLIGHT", "SEP1", "SEP2", "EMERGENCY", "RECOVERY"};

char file_name[64];
FILE *fp;

int addr;   // EEPROMの書き込みアドレス

char mission_timer_reset;  // mission_timerをリセットした回数
int mission_time;         //電源投入からの経過時間
int flight_time;          //離床検知からの経過時間

int not_recovery = 1;

//動作確認用のフラグ：1=有効, 0=無効
char f_sd;      // SDカード
char f_gps;     // GPS
char f_adxl;    // ADXL375
char f_ina_in;  // INA226（バッテリー）
char f_ina_ex;  // INA226（外部電源）
bool f_lps;     // LPS22HB
char f_mpu;     // MPU9250

float lat;          // 緯度[deg]
float lon;          // 経度[deg]
int sat;            // 捕捉衛星数
int fix;            // 位置特定品質(0だと位置特定できない)
float hdop;         // 水平精度低下率
float alt;          // 海抜高度[m]
float geoid;        // ジオイド[m]
float voltage_in;   // バッテリー電圧[mV]
float current_in;   // バッテリー電流[mA]
float voltage_ex;   // 外部電源電圧[mV]
float current_ex;   // 外部電源電流[mA]
float high_accel[3];// 3軸高加速度[G]
float accel[3];     // 3軸加速度[G]
float gyro[3];      // 3軸角速度[rad/s]
float press;
float temperature;
int press_count;
float press_buf[10];
int detection_count = 0;
float press_LPF;
float press_prev_LPF;
float ground_press;

int main() {
    init();
    wait(1);
    while(not_recovery) {
        read();
        switch(phase) {
            case SAFETY:
                if((press_count == 9)&&(!initialized)){
                    smoothing_ticker.attach(&smoothing, 0.01f);
                    initialized = true;
                }
                break;
            case READY:
                if(flight_pin.read() == 1) phase = FLIGHT;
                break;
            case FLIGHT:
                if(!launched) {
                    flight_timer.start();
                    launched = true;
                    burning = true;
                }
                if(flight_timer.read_ms() > BURN_TIME) {
                    if(burning) {
                        burning = false;
                        peak_detection_ticker.attach(&peak_detection, 0.1f);
                    }
                }
                if(!burning && (apogee || (flight_timer.read_ms() > T_APOGEE))) phase = SEP1;
                break;
            case SEP1:
                peak_detection_ticker.detach();
                sep1 = 1;
                if(!first_separated){
                    sep1_timer.start();
                    first_separated = true;
                }
                //現状では加熱時間が経過したと同時に二段目が動作
                if(sep1_timer.read_ms() > T_HEATING){
                    sep1 = 0;
                    sep1_timer.stop();
                    phase = SEP2;
                }
                break;
            case SEP2:
                if(!second_separated && !SEP2_NG){
                    sep2 = 1;
                    sep2_timer.start();
                    second_separated = true;
                }
                if(SEP2_NG){
                    sep2 = 0;
                    phase = RECOVERY;
                }
                if(sep2_timer.read_ms() > T_HEATING){
                    sep2 = 0;
                    sep2_timer.stop();
                    phase = RECOVERY;
                }
                
                break;
            case RECOVERY:
                buzzer_ticker.attach(&buzzer, 10.0f);
                if(!landed){
                    if((press_LPF > ground_press) || (flight_timer.read_ms() > T_RELAY_OFF)){
                        relay = 0;
                        landed = true;
                        flight_timer.stop();
                        not_recovery = 0;                        
                    }
                }
                break;
            case EMERGENCY:
                mission_timer.stop();
                sep1 = 0;
                sep2 = 0;
                break;
        }
    }
}

void init(){
    mission_timer.start();
    relay = 1;
    wait(0.5);

    es.attach(command_handler);    //ES920LRと接続開始
    downlink_ticker.attach(&downlink, 1.0f);
    record_ticker.attach(&record, 0.01f);
    print_ticker.attach(&print, 0.5f);

    char file_name_format[] = "/sd/IZU2022_AVIONICS_%d.dat";
    int file_number = 1;
    while(1) {
        sprintf(file_name, file_name_format, file_number);
        fp = fopen(file_name, "r");
        if(fp != NULL) {
            fclose(fp);
            file_number++;
        } else {
            sprintf(file_name, file_name_format, file_number);
            break;
        }
    }
    fp = fopen(file_name, "w");
    sd_timer.start();

    if(fp) {
        fprintf(fp, "mission_time,");
        fprintf(fp, "flight_time,");
        fprintf(fp, "phase,");
        fprintf(fp, "relay,");
        fprintf(fp, "sep1,");
        fprintf(fp, "sep2,");
        fprintf(fp, "apogee,");
        fprintf(fp, "first_separated,");
        fprintf(fp, "second_separated,");
        fprintf(fp, "landed,");
        fprintf(fp, "f_sd,f_gps,f_adxl,f_ina_in,f_ina_ex,f_lps,f_mpu,");
        fprintf(fp, "voltage_in,current_in,voltage_ex,current_ex,");
        fprintf(fp, "lat,lon,sat,fix,hdop,alt,geoid,");
        fprintf(fp, "press,press_LPF,temperature,");
        fprintf(fp, "high_accel_x,high_accel_y,high_accel_z,");
        fprintf(fp, "accel_x,accel_y,accel_z,");
        fprintf(fp, "gyro_x,gyro_y,gyro_z,");
        fprintf(fp, "\r\n");
    }
    
    ina_in.begin();
    ina_ex.begin();
    adxl.begin();
    mpu.begin();
    lps.begin();

    wait(0.5);
    f_lps = lps.test();
    if(f_lps) {
        lps.read_press(&press);
        press_LPF = press;
        press_prev_LPF = press;
    }
}

void read(){
    //タイマーがオーバーフローしないために30分おきにリセット
    if(mission_timer.read() >= 30*60) {
        mission_timer.reset();
        mission_timer_reset ++;
    }
    mission_time = mission_timer.read() + mission_timer_reset*30*60;
    flight_time  = flight_timer.read();

    f_sd = (bool)fp;
    f_gps = (bool)fix;
    
    lat   = gps.get_lat();
    lon   = gps.get_lon();
    sat   = gps.get_sat();
    fix   = gps.get_fix();
    hdop  = gps.get_hdop();
    alt   = gps.get_alt();
    geoid = gps.get_geoid();

    f_ina_in = ina_in.test();
    if(f_ina_in) {
        ina_in.read_voltage(&voltage_in);
        ina_in.read_current(&current_in);
    }

    f_ina_ex = ina_ex.test();
    if(f_ina_ex) {
        ina_ex.read_voltage(&voltage_ex);
        ina_ex.read_current(&current_ex);
    }

    f_mpu = mpu.test();
    if(f_mpu) {
        mpu.read_accel(accel);
        mpu.read_gyro(gyro);
    }

    f_adxl = adxl.test();
        if(f_adxl) {
            adxl.read(high_accel);
        }
    
    f_lps = lps.test();
    if(f_lps) {
        lps.read_press(&press);
        lps.read_temp(&temperature);
        press_buf[press_count] = press;
        press_count++;
        if(press_count > 9) {
            press_count = 0;
        }
    }
}

void smoothing(){
    // 配列のコピー
    float buf[10];
    for(int i = 0; i < 10; i++) {
        buf[i] = press_buf[i];
    }

    // バブルソート
    for(int i = 0; i < 9; i++) {    //9???
        for(int j = 9; j > i; j--) {
            if(buf[j] < buf[j - 1]) {
                float temp = buf[j];
                buf[j] = buf[j - 1];
                buf[j - 1] = temp;
            }
        }
    }
    float press_median = (buf[4] + buf[5]) / 2.0f;
    press_prev_LPF = press_LPF;
    press_LPF = press_median * coef + press_prev_LPF * (1 - coef);
}

void peak_detection(){
    if(press_prev_LPF < press_LPF){
        detection_count++;
    }else{
        detection_count = 0;
    }
    if(detection_count == 5) apogee = true;
}

void print(){
    pc.printf("%d\r\n", phase);
};

void record(){
    // EEPROM
    if((phase >= FLIGHT) && !landed) {
        //書き込みページサイズは1バイト(0-127)なのでchar型.
        // int:4バイト, float:4バイト, bool:1バイト
        char data[128]; 
        data[0] = ((char*)&mission_time)[0];
        data[1] = ((char*)&mission_time)[1];
        data[2] = ((char*)&mission_time)[2];
        data[3] = ((char*)&mission_time)[3];
        data[4] = ((char*)&flight_time)[0];
        data[5] = ((char*)&flight_time)[1];
        data[6] = ((char*)&flight_time)[2];
        data[7] = ((char*)&flight_time)[3];
        data[8] = phase;
        data[9] = relay.read();
        data[10] = sep1.read();
        data[11] = sep2.read();
        data[12] = apogee;
        data[13] = first_separated;
        data[14] = second_separated;
        data[15] = landed;
        data[16] = 0;
        data[17] = 0;
        data[18] = f_sd;
        data[19] = f_gps;
        data[20] = f_adxl;
        data[21] = f_ina_in;
        data[22] = f_ina_ex;
        data[23] = f_lps;
        data[24] = f_mpu;
        data[25] = ((char*)&lat)[0];    //GPS↓
        data[26] = ((char*)&lat)[1];
        data[27] = ((char*)&lat)[2];
        data[28] = ((char*)&lat)[3];
        data[29] = ((char*)&lon)[0];
        data[30] = ((char*)&lon)[1];
        data[31] = ((char*)&lon)[2];
        data[32] = ((char*)&lon)[3];
        data[33] = ((char*)&sat)[0];
        data[34] = ((char*)&sat)[1];
        data[35] = ((char*)&sat)[2];
        data[36] = ((char*)&sat)[3];
        data[37] = ((char*)&fix)[0];
        data[38] = ((char*)&fix)[1];
        data[39] = ((char*)&fix)[2];
        data[40] = ((char*)&fix)[3];
        data[41] = ((char*)&hdop)[0];
        data[42] = ((char*)&hdop)[1];
        data[43] = ((char*)&hdop)[2];
        data[44] = ((char*)&hdop)[3];
        data[45] = ((char*)&alt)[0];
        data[46] = ((char*)&alt)[1];
        data[47] = ((char*)&alt)[2];
        data[48] = ((char*)&alt)[3];
        data[49] = ((char*)&geoid)[0];
        data[50] = ((char*)&geoid)[1];
        data[51] = ((char*)&geoid)[2];
        data[52] = ((char*)&geoid)[3];
        data[53] = ((char*)&high_accel[0])[0];  //高加速度センサ↓
        data[54] = ((char*)&high_accel[0])[1];
        data[55] = ((char*)&high_accel[0])[2];
        data[56] = ((char*)&high_accel[0])[3];
        data[57] = ((char*)&high_accel[1])[0];
        data[58] = ((char*)&high_accel[1])[1];
        data[59] = ((char*)&high_accel[1])[2];
        data[60] = ((char*)&high_accel[1])[3];
        data[61] = ((char*)&high_accel[2])[0];
        data[62] = ((char*)&high_accel[2])[1];
        data[63] = ((char*)&high_accel[2])[2];
        data[64] = ((char*)&high_accel[2])[3];
        data[65] = ((char*)&voltage_in)[0];      //電圧電流センサ↓
        data[66] = ((char*)&voltage_in)[1];
        data[67] = ((char*)&voltage_in)[2];
        data[68] = ((char*)&voltage_in)[3];
        data[69] = ((char*)&current_in)[0];
        data[70] = ((char*)&current_in)[1];
        data[71] = ((char*)&current_in)[2];
        data[72] = ((char*)&current_in)[3];
        data[73] = ((char*)&voltage_ex)[0];
        data[74] = ((char*)&voltage_ex)[1];
        data[75] = ((char*)&voltage_ex)[2];
        data[76] = ((char*)&voltage_ex)[3];
        data[77] = ((char*)&current_ex)[0];
        data[78] = ((char*)&current_ex)[1];
        data[79] = ((char*)&current_ex)[2];
        data[80] = ((char*)&current_ex)[3];
        data[81] = ((char*)&press)[0];          //気圧センサ↓
        data[82] = ((char*)&press)[1];
        data[83] = ((char*)&press)[2];
        data[84] = ((char*)&press)[3];
        data[85] = ((char*)&temperature)[0];
        data[86] = ((char*)&temperature)[1];
        data[87] = ((char*)&temperature)[2];
        data[88] = ((char*)&temperature)[3];
        data[89] = ((char*)&accel[0])[0];       //9軸加速度センサ↓
        data[90] = ((char*)&accel[0])[1];
        data[91] = ((char*)&accel[0])[2];
        data[92] = ((char*)&accel[0])[3];
        data[93] = ((char*)&accel[1])[0];
        data[94] = ((char*)&accel[1])[1];
        data[95] = ((char*)&accel[1])[2];
        data[96] = ((char*)&accel[1])[3];
        data[97] = ((char*)&accel[2])[0];
        data[98] = ((char*)&accel[2])[1];
        data[99] = ((char*)&accel[2])[2];
        data[100] = ((char*)&accel[2])[3];
        data[101] = ((char*)&gyro[0])[0];
        data[102] = ((char*)&gyro[0])[1];
        data[103] = ((char*)&gyro[0])[2];
        data[104] = ((char*)&gyro[0])[3];
        data[105] = ((char*)&gyro[1])[0];
        data[106] = ((char*)&gyro[1])[1];
        data[107] = ((char*)&gyro[1])[2];
        data[108] = ((char*)&gyro[1])[3];
        data[109] = ((char*)&gyro[2])[0];
        data[110] = ((char*)&gyro[2])[1];
        data[111] = ((char*)&gyro[2])[2];
        data[112] = ((char*)&gyro[2])[3];
        data[113] = 0;
        data[114] = 0;
        data[115] = 0;
        data[116] = 0;
        data[117] = 0;
        data[118] = 0;
        data[119] = 0;
        data[120] = 0;
        data[121] = 0;
        data[122] = 0;
        data[123] = 0;
        data[124] = 0;
        data[125] = 0;
        data[126] = 0;
        data[127] = 0;

        eeprom.write(addr, data, 128);
        addr += 0x80;
    }
    
    if(fp) {
        fprintf(fp, "%d,%d,%s,", mission_time, flight_time, phase_names[phase]);
        fprintf(fp, "%d,%d,%d,%d,%d,%d,%d,", relay.read(), sep1.read(), sep2.read(), apogee, first_separated, second_separated, landed);
        fprintf(fp, "%d,%d,%d,%d,%d,%d,%d,", f_sd, f_gps, f_adxl, f_ina_in, f_ina_ex, f_lps, f_mpu);
        fprintf(fp, "%.3f,%.3f,%.3f,%.3f,", voltage_in, current_in, voltage_ex, current_ex);
        fprintf(fp, "%.6f,%.6f,%d,%d,%f,%f,%f,", lat, lon, sat, fix, hdop, alt, geoid);
        fprintf(fp, "%f,%f,%.2f,", press, press_LPF, temperature);
        fprintf(fp, "%f,%f,%f,", high_accel[0], high_accel[1], high_accel[2]);
        fprintf(fp, "%f,%f,%f,", accel[0], accel[1], accel[2]);
        fprintf(fp, "%f,%f,%f", gyro[0], gyro[1], gyro[2]);
        fprintf(fp, "\r\n");
    }
    
    if(sd_timer.read() >= 60*30){  //30分ごとにfcloseする
        sd_timer.reset();
        if(fp){
            fclose(fp);
            fp = fopen(file_name, "a");
        }
    }
}

void downlink(){
    short mission_time_bits = (short)mission_time;   //int -> short型：2バイト
    short flight_time_bits = (short)flight_time;

    char flags1 = 0;
    flags1 |= (char)apogee      << 7;
    flags1 |= (char)landed      << 6;
    flags1 |= flight_pin.read() << 5;
    flags1 |= relay.read()      << 4;
    flags1 |= sep1.read()       << 3;
    flags1 |= sep2.read()       << 2;
    flags1 |= (char)SEP2_NG     << 1;
    flags1 |= (char)SEP2_NG     << 0;

    char flags2 = 0;
    flags2 |= f_sd      << 7;
    flags2 |= f_gps     << 6;
    flags2 |= f_ina_in  << 5;
    flags2 |= f_ina_ex  << 4;
    flags2 |= f_mpu     << 3;
    flags2 |= f_adxl    << 2;
    flags2 |= f_lps     << 1;
    flags2 |= f_lps     << 0;

    short voltage_in_bits = (short)(voltage_in / VOLTAGE_LSB);  // センサから出力される生データに戻す
    short current_in_bits = (short)(current_in / CURRENT_LSB);
    short voltage_ex_bits = (short)(voltage_ex / VOLTAGE_LSB);
    short current_ex_bits = (short)(current_ex / CURRENT_LSB);

    short press_bits = (short)(press_LPF * PRESS_LSB);
    short temp_bits = (short)(temperature / TEMP_LSB);

    short high_accel_bits[3];
    for(int i = 0; i < 3; i++) {
        high_accel_bits[i] = (short)(high_accel[i] / ADXL375_LSB);
    }

    short accel_bits[3];
    for(int i = 0; i < 3; i++) {
        accel_bits[i] = (short)(accel[i] / ACCEL_LSB);  //*8いる？？
    }

    short gyro_bits[3];
    for(int i = 0; i < 3; i++) {
        gyro_bits[i] = (short)(gyro[i] / GYRO_LSB);
    }

    char data[50];  //サイズ50の配列を用意. 1箱：1バイト.ポインタ型変数：4バイト. char型:1バイト
    data[0]  = ((char*)&mission_time_bits)[0];
    data[1]  = ((char*)&mission_time_bits)[1];
    data[2]  = ((char*)&flight_time_bits)[0];
    data[3]  = ((char*)&flight_time_bits)[1];
    data[4]  = phase;
    data[5]  = flags1;
    data[6]  = flags2;
    data[7]  = mission_timer_reset;
    data[8]  = ((char*)&lat)[0];
    data[9]  = ((char*)&lat)[1];    
    data[10] = ((char*)&lat)[2];
    data[11] = ((char*)&lat)[3];
    data[12] = ((char*)&lon)[0];
    data[13] = ((char*)&lon)[1];   
    data[14] = ((char*)&lon)[2];
    data[15] = ((char*)&lon)[3];
    data[16] = ((char*)&alt)[0];
    data[17] = ((char*)&alt)[1];
    data[18] = ((char*)&alt)[2];
    data[19] = ((char*)&alt)[3];
    data[20] = ((char*)&voltage_in_bits)[0];
    data[21] = ((char*)&voltage_in_bits)[1];
    data[22] = ((char*)&current_in_bits)[0];
    data[23] = ((char*)&current_in_bits)[1];
    data[24] = ((char*)&voltage_ex_bits)[0];
    data[25] = ((char*)&voltage_ex_bits)[1];
    data[26] = ((char*)&current_ex_bits)[0];
    data[27] = ((char*)&current_ex_bits)[1];
    data[28] = ((char*)&press_bits)[0];
    data[29] = ((char*)&press_bits)[1];
    data[30] = ((char*)&temp_bits)[0];
    data[31] = ((char*)&temp_bits)[1];
    data[32] = ((char*)&high_accel_bits[0])[0];
    data[33] = ((char*)&high_accel_bits[0])[1];
    data[34] = ((char*)&high_accel_bits[1])[0];
    data[35] = ((char*)&high_accel_bits[1])[1];
    data[36] = ((char*)&high_accel_bits[2])[0];
    data[37] = ((char*)&high_accel_bits[2])[1];
    data[38] = ((char*)&accel_bits[0])[0];
    data[39] = ((char*)&accel_bits[0])[1];
    data[40] = ((char*)&accel_bits[1])[0];
    data[41] = ((char*)&accel_bits[1])[1];
    data[42] = ((char*)&accel_bits[2])[0];
    data[43] = ((char*)&accel_bits[2])[1];
    data[44] = ((char*)&gyro_bits[0])[0];
    data[45] = ((char*)&gyro_bits[0])[1];
    data[46] = ((char*)&gyro_bits[1])[0];
    data[47] = ((char*)&gyro_bits[1])[1];
    data[48] = ((char*)&gyro_bits[2])[0];
    data[49] = ((char*)&gyro_bits[2])[1];

    es.send(data, 50);
}

void command_handler(char* command){
    switch(command[0]) {
        case 0xB0:  //"0" READY --> SAFETY
            if(phase == READY) phase = SAFETY;
            break;
        case 0xB1:  //"1" SAFETY --> READY
            if(phase == SAFETY) phase = READY;
            break;
        case 0xB2:  //"2" READY --> FLIGHT
            if(phase == READY) phase = FLIGHT;
            break;
        case 0xB3:  //"3" FLIGHT --> SEP1
            if(!burning && phase == FLIGHT) phase = SEP1;
            break;
        case 0xB4:  //"4" READY or FLIGHT, SEP1, SEP2 --> EMERGENCY
            if(phase >= READY && phase <= SEP2) phase = EMERGENCY;
            break;
        case 0xB5:  //"5" Switch SEP2_NG
            if(phase >= READY && phase <= SEP1) SEP2_NG = true;
            break;
        case 0xB6:  //"6"
            break;
        case 0xFF:  //"DEL" Mbed RESET
            NVIC_SystemReset();
            break;
    }
}

void buzzer(){
    buzzer_pin = 1;
    wait(3);
    buzzer_pin = 0;
}