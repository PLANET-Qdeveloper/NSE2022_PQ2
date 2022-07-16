from machine import Pin, SPI, UART, I2C, soft_reset, Timer, lightsleep
from utime import ticks_ms
import time, os
import sdcard

from PQ_LPS22HB import LPS22HB
from PQ_RM92 import RM92A
#from PQ_GPS import GPS

# I2C通信
i2c = I2C(0, scl=Pin(21), sda=Pin(20))

# SPI通信
cs = Pin(17, Pin.OUT)
spi = SPI(0, baudrate=32000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
lightsleep(100)
sd = sdcard.SDCard(spi, cs)
os.mount(sd, '/sd')

file_index = 1
file_name = '/sd/PQ2_AVIONICS'+str(file_index)+'.txt'
while True:
    try:
       file = open(file_name, 'r')
    except OSError: # 新しい番号であればエラーに拾われる
        file = open(file_name, 'w')
        init_sd_time = ticks_ms()
        break
    if file:    # 同じ番号が存在する場合引っかかる
        file.close()    # 一旦古いファイルなので閉じる
        file_index += 1
        file_name = '/sd/PQ2_AVIONICS'+str(file_index)+'.txt'

file.write("phase,mission_time,mission_time_reset,flight_time,")
file.write("flight_pin,burning,apogee,sep,separated,landed,")
file.write("pressure,temperature,lat,lon,alt\r\n")

# UART通信
rm_uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
gps_uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

# インスタンス生成
rm = RM92A(rm_uart)
#gps = GPS(gps_uart)
#lps = LPS22HB(i2c)

p2 = Pin(2, Pin.IN)  # irq用のピン
#p2.init(p2.IN, p2.PULL_UP)
led = Pin(25, Pin.OUT)

# Timerオブジェクト(周期処理用)
peak_detection_timer = Timer()
record_timer = Timer()
downlink_timer = Timer()

# 定数
signal_timing = 1000
T_BURN = 3300
T_SEP = 12200000
T_HEATING = 9000
T_RECOVERY = 300000
irq_called_time = time.ticks_ms()

# 変数整理
burning = False
block_flug = False
detect_peak = False
flight_pin = Pin(26, Pin.IN)
sep_pin = Pin(27, Pin.OUT)
sep_pin.value(0)
phase = 0
mission_timer_reset = 0
init_mission_time = ticks_ms()
mission_time = 0
mission_time_int = 0
init_flight_time = 0
flight_time = 0
flight_time_int = 0
init_sep_time = 0
sep_time = 0
peak_count = 0
press_index = 0
press_buf = [0]*10
pressure = prev_press = ground_press = temperature = 0
lat = lon = alt = 0

# bool変数
landed = False
apogee = False
separated = False

# 初期化
def init():
    print("init")
    global burning, detect_peak, flight_pin, sep_pin, phase, mission_timer_reset, init_mission_time, mission_time, mission_time_int, init_flight_time, flight_time, flight_time_int, init_sep_time, sep_time, pressure, temperature, lat, lon, alt, peak_count, apogee, separated, landed, press_index, press_buf, prev_press, ground_press, led, peak_detection_timer, read_timer, downlink_timer, temperature, block_flug, irq_called_time, gps_uart

    file_index = 1
    file_name = '/sd/PQ2_AVIONICS'+str(file_index)+'.txt'
    while True:
        try:
            file = open(file_name, 'r')
        except OSError: # 新しい番号であればエラーに拾われる
            print(file_index)
            file = open(file_name, 'w')
            break
        if file:    # 同じ番号が存在する場合引っかかる
            file.close()    # 一旦古いファイルなので閉じる
            file_index += 1
            file_name = '/sd/PQ2_AVIONICS'+str(file_index)+'.txt'
    
    block_flug = False
    irq_called_time = time.ticks_ms()

    rm_uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
    gps_uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
    i2c = I2C(0, scl=Pin(21), sda=Pin(20))

    p2 = Pin(2, Pin.IN)  # irq用のピン
    p2.init(p2.IN, p2.PULL_UP)

    rm = RM92A(rm_uart)
    #gps = GPS(gps_uart)
    #lps = LPS22HB(i2c)
    # 変数整理
    burning = False
    detect_peak = False
    flight_pin = Pin(26, Pin.IN)
    sep_pin = Pin(27, Pin.OUT)
    sep_pin.value(0)
    phase = 0
    mission_timer_reset = 0
    init_mission_time = ticks_ms()
    mission_time = 0
    mission_time_int = 0
    init_flight_time = 0
    flight_time = 0
    flight_time_int = 0
    init_sep_time = 0
    sep_time = 0

    peak_count = 0

    press_index = 0
    press_buf = [0]*10
    pressure = prev_press = ground_press = tempareture = 0
    lat = lon = alt = 0

    landed = False
    apogee = False
    separated = False

    led = Pin(25, Pin.OUT)
    peak_detection_timer = Timer()
    read_timer = Timer()
    downlink_timer = Timer()


def get_smoothed_press():
    global press_index, press_buf
    press_median = 0
    press = lps.read_pressure()
    if not press == None:
        press_buf[press_index] = press
        press_index += 1
        if press_index == 9:
            press_index = 0
            for i in range(10):
                for j in range(10-i-1):
                    if press_buf[j] > press_buf[j+1]:   # 左のほうが大きい場合
                        press_buf[j], press_buf[j+1] = press_buf[j + 1], press_buf[j]  # 前後入れ替え
        press_median = (press_buf[4]+press_buf[5])/2
        # LPFはいらないかも
    return press_median


def peak_detection(t):
    global prev_press, pressure
    global peak_count, apogee
    if pressure > prev_press:
        peak_count += 1
    else:
        peak_count = 0
    prev_press = pressure
    if peak_count == 5:
        apogee = True


def read():
    global phase
    global mission_time, mission_timer_reset, mission_time_int, init_mission_time, init_flight_time, flight_time_int, flight_time, sep_time, init_sep_time
    global pressure, temperature
    global lat, lon, alt

    # 時間関係
    mission_time = ticks_ms() - init_mission_time
    mission_time_int = int(mission_time/1000)    # 小数点以下は切り捨て
    if mission_time_int > 180:
        init_mission_time = ticks_ms()
        mission_time_int = 0
        mission_timer_reset += 1
    if phase == 2:
        flight_time = ticks_ms() - init_flight_time
        flight_time_int = int(flight_time/1000)
    if phase == 3:
        sep_time = ticks_ms() - init_sep_time

    # センサーの値を取得
    #pressure = get_smoothed_press()  # medianをとってくる
    #temperature = lps.read_temperature()
    #lat = gps.getLat()
    #lon = gps.getLon()
    #alt = gps.getAlt()

def record(t):
    global file, init_sd_time
    file.write("%d,%f,%d,%f,"%(phase, mission_time, mission_timer_reset, flight_time))
    file.write("%d,%d,%d,%d,%d,%d,"%(flight_pin.value(), burning, apogee, sep_pin.value(), separated,landed))
    file.write("%f,%f,%f,%f,%f\r\n"%(pressure, temperature, lat, lon, alt))
    if (ticks_ms() - init_sd_time > 10000):    # 10秒ごとにclose()して保存する
        file.close()
        file = open(file_name, "a")
        init_sd_time = ticks_ms()
record_timer.init(period=10, callback=record)

def debug():
    print('------------------------------------------------------------------')
    print(mission_time_int, flight_pin.value(), phase)

def downlink(t):
    global phase
    global flight_pin, sep_pin
    global burning, apogee, separated, landed
    global pressure, temperature, lat, lon, alt
    debug()
    flags = 0
    flags |= flight_pin.value() << 7
    flags |= burning << 6
    flags |= apogee << 5
    flags |= separated << 4
    flags |= sep_pin.value() << 3
    flags |= landed << 2
    flags |= 0 << 1
    flags |= 0 << 0

    press_int = int(pressure*100)  # 下2桁までを繰り上げして型変換
    press_bits_A = press_int >> 16 & 0xff
    press_bits_B = press_int >> 8 & 0xff
    press_bits_C = press_int >> 0 & 0xff

    temp_int = int(temperature)  # 小数点以下は切り捨て
    temp_bits = temp_int >> 0 & 0xff

    lat_int = int(lat*10000)    # 下4桁までを繰り上げ
    lat_bits_A = lat_int >> 16 & 0xff
    lat_bits_B = lat_int >> 8 & 0xff
    lat_bits_C = lat_int >> 0 & 0xff
    lon_int = int(lon*10000)    # 下4桁までを繰り上げ
    lon_bits_A = lon_int >> 16 & 0xff
    lon_bits_B = lon_int >> 8 & 0xff
    lon_bits_C = lon_int >> 0 & 0xff

    send_data = bytearray(16)
    send_data[0] = 0x24
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
    rm.send(0xFFFF, send_data)
    print(send_data)
downlink_timer.init(period=2000, callback=downlink)


def command_handler(p2):
    global block_irq, irq_called_time
    global signal_timing
    global phase, init_flight_time, burning
    rx_buf = bytearray(4)
    if (time.ticks_ms() - irq_called_time) > signal_timing:
        block_irq = False
    if not block_irq:
        print("called")
        rm_uart.readinto(rx_buf, 4)
        command = rx_buf[0]
        print(command)
        if command == 48:     # 0 READY->SAFETY
            if phase == 1:
                phase = 0
        elif command == 49:   # 1 SAFETY->READY
            if phase == 0:
                phase = 1
        elif command == 50:   # 2 READY->FLIGHT
            if phase == 1:
                phase = 2
                burning = True
                init_flight_time = ticks_ms()
        elif command == 51:   # 3 SEP
            if phase == 2:
                phase = 3
        elif command == 52:   # 4 EMERGENCY
            if (phase >= 1 & phase <= 3):
                phase = 5
        elif command == 127:   # RESET
            print("reset?")
            init()
        irq_called_time = time.ticks_ms()
        block_irq = True
irq_obj = p2.irq(handler=command_handler, trigger=(Pin.IRQ_FALLING | Pin.IRQ_RISING))


def main():
    global phase
    global flight_pin, sep_pin
    global init_flight_time, init_sep_time
    global burning, apogee, separated, landed

    while True:
        lightsleep(10)
        read()
        if phase == 0:  # SAFETYモード
            # print("SAFETY")
            lightsleep(100)
        elif phase == 1:    # READYモード
            # print("READY")
            if flight_pin.value() == 1:
                burning = True
                init_flight_time = ticks_ms()
                phase = 2
        elif phase == 2:    # FLIGHTモード
            # print("FLIGHT")
            if (ticks_ms() - init_flight_time) > T_BURN:
                if burning == True:
                    burning = False
                    #peak_detection_timer.init(period=100, callback=peak_detection)
            if (not burning) and (apogee or ((ticks_ms() - init_flight_time) > T_SEP)):
                phase = 3
                #peak_detection_timer.deinit()
        elif phase == 3:   # SEPモード
            print("SEP")
            sep_pin.value(1)
            lightsleep(100)
            if not separated:
                init_sep_time = ticks_ms()
                separated = True
            else:
                if (ticks_ms() - init_sep_time) > T_HEATING:
                    sep_pin.value(0)
                    phase = 4
        elif phase == 4:   # RECOVERY
            lightsleep(100)

            if not landed:
                if (ticks_ms() - init_flight_time) > T_RECOVERY:
                #if (pressure > ground_press) or ((ticks_ms() - init_flight_time) > T_RECOVERY):
                    #relay = 0
                    landed = True
            else:
                lightsleep(1000)

        elif phase == 5:   # EMERGENCY
            sep_pin.value(0)
            lightsleep(1000)

if __name__ == '__main__':
    main()
