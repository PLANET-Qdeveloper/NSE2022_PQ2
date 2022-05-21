from pyrsistent import v
from machine import Pin, I2C, SPI, Timer
from microGPS import MicroGPS
import os, sdcard, utime

import PQ_RM92A
import PQ_LPS22HB

phase = ['SAFETY', 'READY', 'FLIGHT', 'SEP', 'RECOVERY']

gps_uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))
rm_uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))


# GPS
#______________________________________________________________________
TIMEZONE = 9
gps = MicroGPS(TIMEZONE)
#______________________________________________________________________


# I2C
#______________________________________________________________________
i2c = I2C(0, scl=Pin(9), sda=Pin(8))
#______________________________________________________________________

# SD card
#______________________________________________________________________
cs = Pin(9, Pin.OUT)
spi = SPI(1, baudrate=1000000, sck=Pin(10), mosi=Pin(11), miso=Pin(8))
sd = sdcard.SDCard(spi, cs)
os.mount(sd, '/sd')
#______________________________________________________________________


#ピン設定-------------------------------------------------------------
flight_pin = 27
sep = 18
#---------------------------------------------------------------------

def read():
    mission_time = ticks_ms()

    # GPS-------------------------------------------------------------
    # タプルで返されるから一行の文字列に変換. (37, 51.65, 'S') → 3751.65S
    lat     = ''.join(gps.latitude)    
    lon     = ''.join(gps.longitude)
    alt     = gps.altitude
    sat     = gps.sat
    fix     = gps.hdop
    geoid   = gps.geoid_height
    timestamp = '.'.join(gps.timestamp)
    #-----------------------------------------------------------------


    # LPS22HB---------------------------------------------------------








def record():
    file = open('/sd/HybridRocketProject_PQ2_'+file_number+'.csv', 'w')
    if(file):
        file.write(str(mission_time), str(flight_time), phase,)
        file.write(flight_pin.value(), sep.value(), apogee.value(), landed.value())
        file.write(str(lat), str(lon), str(sat), str(fix), str(hdop), str(alt), str(geoid))
        file.write(str(press), str(press_LPF), str(temperature))
        file.write('\r\n')


# 10ms周期で定期的に実行
read_ticker = Timer(period=10, mode=Timer.PERIODIC, callback=read)
record_ticker = Timer(period=10, callback=record)

