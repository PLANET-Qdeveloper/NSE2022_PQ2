from machine import Pin, UART, I2C, reset, Timer, lightsleep
from utime import ticks_ms

import PQ_LPS22HB
import PQ_RM92A
import PQ_GPS
import sd


# インスタンス生成
rm_uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
gps_uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
i2c = I2C(0, scl=Pin(21), sda=Pin(20))

rm = PQ_RM92A.RM92A(rm_uart)
gps = PQ_GPS.GPS(gps_uart) 
lps = PQ_LPS22HB.LPS22HB(i2c, 'LOW')


# bool型変数
burning = False
detect_peak = False
landed = False
apogee = False
flight_pin = Pin(0, Pin.IN)
sep_pin = Pin(0, Pin.OUT)

# int型変数
phase = 0
mission_time_int = 0
mission_timer_reset = 0
detection_count = 0
i = 0

# float型変数
init_mission_time = ticks_ms() 
mission_time = 0
flight_time = 0
ground_press = 0
press_buf = [0]*10
lat = 0.0  # 緯度[°]
lon = 0.0  # 経度[°]
press_prev_LPF = 0
press_LPF = 0
temperature = 0

# 定数
T_BURN = 3.3
T_SEP = 12.2
T_HEATING = 5.0

# Timerオブジェクト(周期処理用)
peak_detection_timer = Timer()
read_timer = Timer()
downlink_timer = Timer()


# 受け取ったコマンドを処理するためのhandler関数
def command_handler(command):
    if command == 0xB0:     # READY->SAFETY
        if phase == 1: phase = 0
    elif command == 0xB1:   # SAFETY->READY
        if phase == 0: phase = 1
    elif command == 0xB2:   # READY->FLIGHT
        if phase == 1: phase = 2
    elif command == 0xB3:   # SEP
        if (burning == False & phase == 2): phase = 3
    elif command == 0xB4:   # EMERGENCY
        if (phase >= 1 & phase <= 3): phase = 5
    elif command == 0xB5:   # RESET
        reset()
    return 0

def read():
    global mission_time
    global mission_timer_reset
    global flight_pin
    global press_LPF, press_prev_LPF, temperature
    global lat, lon, alt

    mission_time = ticks_ms() - init_mission_time
    if mission_time > 3600:
        mission_time = 0
        mission_time_reset += 1

    if not flight_pin:
        ground_press = lps.getPressure()
        press_prev_LPF = ground_press
    
    press_LPF, press_prev_LPF = smoothing(lps.getPressure(), press_prev_LPF)
    temperature = lps.getTemperature()

    lat = gps.getLat()
    lon = gps.getLon()
    alt = gps.getAlt()
    
    return

# 受け取った気圧データを平滑化する関数.毎回呼ばれる.
def smoothing(raw_press, press_LPF):
    global press_buf, index
    PRESS_PREV_LPF = press_LPF
    press_buf[index] = raw_press
    index+=1
    if index == 9: index = 0

    for i in range(10):
        for j in range(10-i-1):
            if press_buf[j] > press_buf[j+1]:   # 左のほうが大きい場合
                press_buf[j], press_buf[j+1] = press_buf[j+1], press_buf[j] # 前後入れ替え
    press_median = (press_buf[4]+press_buf[5])/2
    
    PRESS_LPF = press_median*0.01 + PRESS_PREV_LPF*(1-0.01)
    return PRESS_LPF, PRESS_PREV_LPF

# 頂点検知関数(有効：FLIGHTモード)
def peak_detection():
    global apogee
    global press_LPF, press_prev_LPF
    global detection_count
    
    if press_prev_LPF < press_LPF: 
        detection_count += 1
    else: 
        detection_count = 0
    if detection_count == 5: apogee = True
    return

# ダウンリンク関数.1Hzで呼び出される.
def downlink():
    mission_time_int = int(mission_time)    # 小数点以下は切り捨て
    mission_time_bits_A = mission_time_int >> 8 & 0xff
    mission_time_bits_B = mission_time_int >> 0 & 0xff

    flight_time_int = int(flight_time)  # 小数点以下は切り捨て
    flight_time_bits_A = flight_time_int >> 8 & 0xff
    flight_time_bits_B = flight_time_int >> 0 & 0xff

    flags |= flight_pin << 7 
    flags |= burning << 6
    flags |= detect_peak << 5 
    flags |= apogee << 4
    flags |= sep_pin << 3
    flags |= landed << 2
    flags |= 0 << 1
    flags |= 0 << 0 
    
    flags2 |= f_lps << 7
    flags2 |= f_gps << 6
    flags2 |= f_sd << 5
    flags2 |= 0 << 4
    flags2 |= 0 << 3
    flags2 |= 0 << 2
    flags2 |= 0 << 1
    flags2 |= 0 << 0 

    press_int = int(press_LPF*100)  # 下2桁までを繰り上げして型変換
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

    send_data = bytearray(18)
    send_data[0] = mission_time_bits_A
    send_data[1] = mission_time_bits_B
    send_data[2] = mission_timer_reset
    send_data[3] = flight_time_bits_A
    send_data[4] = flight_time_bits_B
    send_data[5] = phase
    send_data[6] = flags
    send_data[7] = flags2
    send_data[8] = press_bits_A
    send_data[9] = press_bits_B
    send_data[10] = press_bits_C
    send_data[11] = temp_bits
    send_data[12] = lat_bits_A
    send_data[13] = lat_bits_B
    send_data[14] = lat_bits_C
    send_data[15] = lon_bits_A
    send_data[16] = lon_bits_B
    send_data[17] = lon_bits_C

    rm.send(0xFFFF, send_data, 18)
    return
downlink_timer.init(period=1000, callback=downlink)

'''
def get_gps():
    lat = gps.getLat()
    lon = gps.getLon()
    return lat, lon
'''

def main():
    while True:
        read()
        if phase == 0:  # SAFETYモード
            lightsleep(10)  #念のため？
            break
        elif phase == 1:    # READYモード
            if flight_pin.value() == 1:
                burning = True
                flight_time_start = ticks_ms()
                phase = 2
            break
        elif phase == 2:    # FLIGHTモード
            if (ticks_ms() - flight_time_start) > T_BURN:
                if burning == True:
                    burning = False
                    peak_detection_timer.init(period=100, callback=peak_detection)
            if (not burning) and (apogee or ((ticks_ms() - flight_time_start) > T_SEP)):
                phase = 3
            break
        elif phase == 3:
            peak_detection_timer.deinit()
            sep_pin.value(1)
            if not separated:
                sep_time_start = ticks_ms()
                separated = True
            if (ticks_ms() - sep_time) > T_HEATING:
                sep_pin.value(0)
                phase = 4
            break
        elif phase == 4:   # RECOVERY
            if not landed:
                if (press_LPF > ground_press) or (ticks_ms() > flight_time_start):
                    #relay = 0
                    landed = True
            break
        else:   # EMERGENCY
            sep_pin.value(0)
            break

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("中断しました")