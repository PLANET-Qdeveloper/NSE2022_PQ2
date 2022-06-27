from machine import Pin, UART, I2C, reset, Timer
from utime import ticks_ms, sleep_ms

import PQ_LPS22HB
import PQ_RM92A
import PQ_GPS


# インスタンス生成
rm_uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
gps_uart = UART(1, baudrate=115200, tx=Pin(), rx=Pin())

rm = PQ_RM92A.RM92A(rm_uart)
gps = PQ_GPS.GPS(gps_uart)  # rx, tx, baudrate

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100)

lps = PQ_LPS22HB.LPS22HB(i2c)


# bool型変数
burning = False
detect_peak = False
launched = False
landed = False
apogee = False
flight_pin = Pin(0, Pin.IN)
sep_pin = Pin(0, Pin.OUT)

# Timerオブジェクト(周期処理)
peak_detection_timer = Timer()
read_timer = Timer()
downlink_timer = Timer()


# int型変数
phase = 0
mission_time_int = 0
mission_timer_reset = 0
detection_count = 0

# float型変数
init_mission_time = ticks_ms()
mission_time = 0
flight_time = 0
ground_press = 0
press_buf = [10]
lat = 0.0  # 緯度[°]
lon = 0.0  # 経度[°]
press_prev_LPF = 0
press_LPF = 0
temperature = 0
press_prev_LPF = 0

T_BURN = 3.3  # 燃焼時間
T_SEP = 12.2  # 分離時間
T_HEATING = 5.0

coef = 0.01


# 受け取ったコマンドを処理するためのhandler関数


def command_handler():
    # readlineをココで書く？
    command = rm_uart.read(1)
    if command == 0xB0:     # READY->SAFETY
        if phase == 1:
            phase = 0
    elif command == 0xB1:   # SAFETY->READY
        if phase == 0:
            phase = 1
    elif command == 0xB2:   # READY->FLIGHT
        if phase == 1:
            phase = 2
    elif command == 0xB3:   # SEP
        if (burning == False & phase == 2):
            phase = 3
    elif command == 0xB4:   # EMERGENCY
        if (phase >= 1 & phase <= 3):
            phase = 5
    elif command == 0xB5:   # RESET
        reset()
    return 0


# 割り込み対応開始
rm_uart.irq(UART.RX_ANY, priority=1, handler=command_handler, wake=IDLE)


def read():
    mission_time = ticks_ms() - init_mission_time
    if mission_time > 3600:
        mission_time = 0
        mission_time_reset = mission_time_reset + 1

    if not launched:
        ground_press = lps.getPressure()
        press_buf = [ground_press]*10

    press_LPF, press_prev_LPF = smoothing(lps.getPressure(), press_prev_LPF)
    temperature = lps.getTemperature()

    lat = gps.getLat()
    lon = gps.getLon()

    return mission_time, ground_press, press_LPF, press_prev_LPF, temperature, lat, lon


def smoothing(raw_press):
    global press_buf
    press_buf[i] = raw_press
    i = i+1
    if i == 9:
        for i in range(9):
            for j in range(9):
                if press_buf[j-1] > buf[j]:
                    temp = press_buf[j]
                    press_buf[j] = press_buf[j-1]
                    press_buf[j-1] = temp
        press_median = (press_buf[4]+press_buf[5])/2
        global press_prev_LPF
        press_prev_LPF = press_LPF
        press_LPF = press_median*coef + press_prev_LPF*(1-coef)
    return press_LPF, press_prev_LPF


def peak_detection():  # 頂点検知
    global apogee
    global press_LPF
    global press_prev_LPF
    global detection_count
    if press_prev_LPF < press_LPF:
        detection_count += 1
    else:
        detection_count = 0
    if detection_count == 5:
        apogee = True
    return


def downlink():
    mission_time_int = int(mission_time)
    mission_time_bits_A = mission_time_int >> 8 & 0xff
    mission_time_bits_B = mission_time_int >> 0 & 0xff

    flight_time_int = int(flight_time)
    flight_time_bits_A = flight_time_int >> 8 & 0xff
    flight_time_bits_B = flight_time_int >> 0 & 0xff

    press_int = int(press_LPF*100)   # 下2桁までを繰り上げして型変換
    press_bits_A = press_int >> 16 & 0xff
    press_bits_B = press_int >> 8 & 0xff
    press_bits_C = press_int >> 0 & 0xff

    temp_int = int(temperature)
    temp_bits = temp_int >> 0 & 0xff

    lat_int = int(lat*10000)  # 下4桁までを繰り上げ
    lat_bits_A = lat_int >> 16 & 0xff
    lat_bits_B = lat_int >> 8 & 0xff
    lat_bits_C = lat_int >> 0 & 0xff
    lon_int = int(lon*10000)
    lon_bits_A = lon_int >> 16 & 0xff
    lon_bits_B = lon_int >> 8 & 0xff
    lon_bits_C = lon_int >> 0 & 0xff

    send_data = [0]*50
    send_data[] = mission_time_bits_A
    send_data[] = mission_time_bits_B
    send_data[] = mission_timer_reset
    send_data[] = flight_time_bits_A
    send_data[] = flight_time_bits_B
    send_data[] = phase
    send_data[] = launched
    send_data[] = landed
    # send_data[] = relay.value()
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
    # send_data[] =

    rm.send(0x1234, send_data, 50)
    return


downlink_timer.init(period=1000, callback=downlink)


def get_gps():
    lat = gps.getLat()
    lon = gps.getLon()
    return lat, lon


def main():
    while True:
        read()
        if phase == 0:  # SAFETYモード
            break
        elif phase == 1:    # READYモード
            if flight_pin.value() == 1:
                launched = True
                flight_time_start = ticks_ms()
                phase = 2
        elif phase == 2:    # FLIGHTモード
            if (ticks_ms() - flight_time_start) > T_BURN:
                if burning == True:
                    burning = False
                    peak_detection_timer.init(
                        period=100, callback=peak_detection)
            if (not burning) and (apogee or ((ticks_ms() - flight_time_start) > T_SEP)):
                phase = 3
        elif phase == 3:  # SEPARATIONモード
            peak_detection_timer.deinit()
            sep_pin.value(1)
            if not separated:
                sep_time_start = ticks_ms()
                separated = True
            if (ticks_ms() - sep_time) > T_HEATING:
                sep_pin.value(0)
                phase = 4
        elif phase == 4:   # RECOVERYモード
            global lat
            global lon
            read_timer.denit()
            lat, lon = get_gps()
            sleep_ms(1000)
            if not landed:
                if (press_LPF > ground_press) or (ticks_ms() > flight_time_start):
                    #relay = 0
                    landed = True
        else:   # EMERGENCYモード
            sep_pin.value(0)
            break
    return


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("中断しました")
