#include "mbed.h"
#include "PQ_GPS.h"

GPS::GPS(Serial &gps_serial)
{
    _serial = &gps_serial;
    _serial->attach(callback(this, &GPS::receive), Serial::RxIrq);
}

void GPS::receive()
{
    char c = _serial->getc();
    if(c == '$') {
        index = 0;
        flag = 1;
    }
    if(flag == 1) {
        rx_buf[index] = c;
        if(c == '\n') {
            rx_buf[index + 1] = '\0';
            flag = 2;
        } else {
            index++;
        }
    }
    if(flag == 2) {
        sscanf(rx_buf, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,M,%f,M", &sec, &lat, &ns, &lon, &ew, &fix, &sat, &hdop, &alt, &geoid);
        
        hour = (int)(sec / 10000);
        min = (int)(sec - hour * 10000) / 100;
        sec = sec - hour * 10000 - min * 100;
        
        int d;
        float m;

        d = (int)lat / 100;
        m = lat - (float)d * 100;
        lat = (float)d + m / 60;
        if(ns == 'S') {
            lat = -lat;
        }

        d = (int)lon / 100;
        m = lon - (float)d * 100;
        lon = (float)d + m / 60;
        if(ns == 'W') {
            lon = -lon;
        }
        
        flag = 0;
    }
}

int GPS::get_hour(){
    return hour;
}

int GPS::get_min(){
    return min;
}

float GPS::get_sec(){
    return sec;
}

float GPS::get_lat(){
    return lat;
}

float GPS::get_lon(){
    return lon;
}

int GPS::get_fix(){
    return fix;
}

int GPS::get_sat(){
    return sat;
}

float GPS::get_hdop(){
    return hdop;
}

float GPS::get_alt(){
    return alt;
}

float GPS::get_geoid(){
    return geoid;
}