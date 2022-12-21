'''
2022NSE(PQ1)main program
'''

from micropython import const
from machine import Pin, SPI, UART, I2C, reset, Timer, lightsleep
from utime import ticks_ms
import time, os
import sdcard
import sys

from PQ_LPS22HB import LPS22HB #気圧センサ
from PQ_RM92A import RM92A #無線機
from micropyGPS import MicropyGPS



'''
定数宣言
'''
SIGNAL_TIMING = const(1000)  
T_BURN = const(6000)  #Engine burning time
T_APOGEE = const(25790) #頂点検知
T_HEATING = const(5000) #ニクロム線加熱時間(使わんけど)
T_RECOVERY = const(120610)
T_RELAY_OFF = const(3000000)


'''
通信関係
'''
# I2C通信
i2c = I2C(0, scl=Pin(21), sda=Pin(20)) #I2C通信のやつ
#eeprom_i2c = I2C(2, scl=Pin(21), sda=Pin(20))

# UART通信
rm_uart= UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13))
gps_uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))

# SPI通信
cs = Pin(17, Pin.OUT)    #SPI通信のやつ
spi = SPI(0, baudrate=32000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))

'''
クラスからインスタンスを生成
'''

lps_gnd_pin = Pin(22, Pin.OUT)
lps_gnd_pin.value(1)

while True:
    try:
        lps = LPS22HB(i2c)
    except OSError:
        pass  #passがあったら、下の文にはいけないよ　ということでまたwhile文(^ ^)
    else:
        break  #OSERROR解消！合格！次！
rm = RM92A(rm_uart)
gps = MicropyGPS()
#ina_in = INA226(69, i2c)    # アドレス指定は1-1
#ina_out = INA226(64, i2c)   # アドレス指定はG-G

#ピンのインスタンス
p2 = Pin(14, Pin.IN) #irq用
led = Pin(25, Pin.OUT)
sep_pin_1 = Pin(0, Pin.OUT)
sep_pin_2 = Pin(1, Pin.OUT)
flight_pin = Pin(27, Pin.IN)

flight_pin_mosfet = Pin(26, Pin.OUT)

flight_pin_mosfet.value(1)
sep_pin_1.value(0)
sep_pin_2.value(0)


'''
Timerオブジェクト(周期処理用)
'''
peak_detection_timer = Timer()
downlink_timer = Timer()
read_timer = Timer()
record_timer = Timer()
gps_timer = Timer()

signal_timing = 1000
init_mission_time = ticks_ms()
irq_called_time = ticks_ms()

'''
変数宣言
'''
initialized =False  
launched = False   #フライトピンが抜けたか
burning=False   #エンジンが燃焼中か
apogee = False   #気圧による頂点検知
f_separated=False
s_separated=False
landed = False

phase = 0
mission_timer_reset = 0
mission_time = mission_time_int = 0
init_flight_time = flight_time = flight_time_int = 0
sep_time_1 = sep_time_2 = init_sep_time_1 = init_sep_time_2 = 0
peak_count = 0
press_index = 0
press_buf = [0]*10
pressure = prev_press = ground_press = temperature = 0.0
lat = lon = alt = lat_int = lon_int = lon_bits_A = lon_bits_B = lon_bits_C = lon_bits_D = 0.0
command = 48
#in_voltage = out_voltage = 0
#in_current = out_current = 0
#in_power = out_power = 0

# SD Card関係

sd = sdcard.SDCard(spi, cs)
os.mount(sd, '/sd')
file_index = 1
file_name = '/sd/PQ1_AVIONICS'+str(file_index)+'.txt'
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
        file_name = '/sd/PQ1_AVIONICS'+str(file_index)+'.txt'

file.write("phase,mission_time,mission_time_reset,flight_time,")
file.write("flight_pin,burning,apogee,sep,separated,landed,")
file.write("pressure,temperature,lat,lon,alt,in_voltage,out_voltage,in_current,out_current,\r\n")


'''
関数を予め宣言．pythonでは，関数の中身も予め言っておくのである．
'''

def get_gps(t):
    global lat, lon, alt
    len = gps_uart.any()
    if len > 0:
        b = gps_uart.read(len)
        for x in b:
            if 10 <= x <=126:
                status = gps.update(chr(x))
                if status:
                    lat = gps.latitude[0] + gps.latitude[1]/60
                    lon = gps.longitude[0] + gps.longitude[1]/60
                    alt = gps.altitude
gps_timer.init(period=2000, callback=get_gps)

def get_smoothed_press():
    global press_index, press_buf

    press = lps.read_pressure()
    if not press == None:
        press_buf[press_index] = press
        press_index += 1
        if press_index == 9:
            press_index = 0
            for i in range(10):
                for j in range(10-i-1):
                    if press_buf[j] > press_buf[j+1]:  #もし左辺の方が大きかったら
                        press_buf[j], press_buf[j+1] = press_buf[j+1], press_buf[j] #前後入れ替え
        press_median = (press_buf[4] + press_buf[5])/2
    else:
        press_median = 0
    return press_median



def read():
    global phase
    global mission_time, mission_timer_reset, mission_time_int, init_mission_time, init_flight_time
    global flight_time_int, flight_time, sep_time_1, sep_time_2, init_sep_time_1, init_sep_time_2
    global pressure, temperature
    global lat, lon, alt, in_voltage, out_voltage, in_current, out_current, in_power, out_power

    mission_time = ticks_ms() - init_mission_time
    mission_time_int = int(mission_time/1000) #小数点以下切り捨て
    if mission_time_int > 180:
        init_mission_time = ticks_ms()
        mission_time_int = 0
        mission_timer_reset += 1
    if phase >= 2:
        flight_time = ticks_ms() - init_flight_time
        flight_time_int = int(flight_time/1000)
    if phase == 3:
        sep_time_1 = ticks_ms() - init_sep_time_1
    if phase == 4:
        sep_time_2 = ticks_ms() - init_sep_time_2

    pressure = get_smoothed_press()  #中央値を取る
    temperature = lps.read_temperature()
    len = gps_uart.any()
    if len > 0:
        b = gps_uart.read(len)
        for x in b:
            if 10 <= x <= 126:
                status = gps.update(chr(x))
                if status:
                    lat = gps.latitude[0] + gps.latitude[1]/60
                    lon = gps.longitude[0] + gps.longitude[1]/60
                    alt = gps.altitude

    '''
    in_voltage = ina_in.get_voltage()
    out_voltage = ina_out.get_voltage()
    in_current = ina_in.get_current()
    out_current = ina_out.get_current()
    in_power = ina_in.get_power()
    out_power = ina_out.get_power()
    '''

def debug():
    print('-----------------------------------')
    print(phase, int(mission_time/1000), pressure, temperature, flight_pin.value(), sep_pin_1.value(), sep_pin_2.value(),lon,lat, lon_bits_A, lon_bits_B, lon_bits_C, lon_bits_D, command)
    #print(peak_count)
    
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

    
def record(t):
    global file, init_sd_time, flight_pin, sep_pin_1, sep_pin_2
    global pressure, lat, lon, alt, latX, lonX
    file.write("%d,%f,%d,%f,"%(phase, mission_time, mission_timer_reset, flight_time))
    file.write("%d,%d,%d,%d,%d,%d,%d,%d"%(flight_pin.value(), burning, apogee, sep_pin_1.value(), sep_pin_2.value(), f_separated, s_separated, landed))
    file.write("%f,%f,%f,%f,%f\r\n"%(pressure, temperature, lat, lon, alt))
    if (ticks_ms() - init_sd_time > 10000):    # 10秒ごとにclose()して保存する
        file.close()
        file = open(file_name, "a")
        init_sd_time = ticks_ms()
record_timer.init(period=100, callback=record)

def downlink(t):
    global lat_int, lon_int, lon_bits_A, lon_bits_B, lon_bits_C, lon_bits_D
    debug()
    flags = 0
    flags |= flight_pin.value() << 7
    flags |= burning << 6
    flags |= apogee << 5
    flags |= f_separated << 4
    flags |= s_separated << 3
    flags |= sep_pin_1.value() << 2
    flags |= sep_pin_2.value() << 1
    flags |= landed << 0

    press_int = int(pressure*100)
    press_bits_A = press_int >> 16 & 0xff
    press_bits_B = press_int >> 8 & 0xff
    press_bits_C = press_int >> 0 & 0xff

    temp_int = int(temperature)  # 小数点以下は切り捨て
    temp_bits = temp_int >> 0 & 0xff

    lat_bits_A = int(lat) & 0xff
    lat_bits_B = int((lat - lat_bits_A)*100) & 0xff
    lat_bits_C = int((lat - lat_bits_A - lat_bits_B/100)*10000) & 0xff
    lat_bits_D = int((lat - lat_bits_A - lat_bits_B/100 - lat_bits_C/10000)*1000000) & 0xff
    
    lon_bits_A = int(lon) & 0xff
    lon_bits_B = int((lon - lon_bits_A)*100) & 0xff
    lon_bits_C = int((lon - lon_bits_A - lon_bits_B/100)*10000) & 0xff
    lon_bits_D = int((lon - lon_bits_A - lon_bits_B/100 - lon_bits_C/10000)*1000000) & 0xff
    

    send_data = bytearray(21)
    send_data[0] = 0x24
    send_data[1] = mission_timer_reset
    send_data[2] = mission_time_int
    send_data[3] = flight_time_int
    send_data[4] = phase
    send_data[5] = flags
    send_data[6] = press_bits_A
    send_data[7] = press_bits_B
    send_data[8] = press_bits_C
#    send_data[9] = temp_bits
#    send_data[9] = lat_bits_A
    send_data[9] = lat_bits_B
    send_data[10] = lat_bits_C
    send_data[11] = lat_bits_D
#    send_data[13] = lon_bits_A
    send_data[12] = lon_bits_B
    send_data[13] = lon_bits_C
    send_data[14] = lon_bits_D

    rm.send(0xFFFF, send_data)
    
downlink_timer.init(period=2000, callback=downlink)

def command_handler(p2):
    global block_irq, irq_called_time, phase, init_mission_time, init_flight_time, burning, flight_time
    global init_sep_time_1, init_sep_time_2, f_separated, s_separated, command
    rx_buf = bytearray(10)
    if (time.ticks_ms() - irq_called_time) > signal_timing:
        block_irq = False
    if not block_irq:
        rm_uart.readinto(rx_buf, 4)
        command = rx_buf[0]
        if command == 48:     # 0 READY[1]->SAFETY[0]
            if phase == 1:
                phase = 0
        elif command == 49:   # 1 SAFETY[0] -> READY[1]
            if phase == 0:
                phase = 1
                
            else:
                phase = 1
                init_mission_time = ticks_ms
                flight_time = 0
                sep_pin_1.value(0)
                
        elif command == 50:   # 2 READY->FLIGHT
            if phase == 1:
                phase = 2
                burning = True
                init_flight_time = ticks_ms()
                
        elif command == 51:   # 3 SEP1
            if (burning == False & phase == 2):
                phase = 3
                init_sep_time_1 = ticks_ms()
                f_separated = True
            
        elif command == 52:   # 4 SEP2
            if (burning == False & phase == 3):
                phase = 4
                init_sep_time_2 = ticks_ms()
                s_separated = True
                
        elif command == 53:   # 5 EMERGENCY
            if (phase >= 1 & phase <= 3):
                phase = 5
                
        elif command == 127:   # RESET
            pass
        irq_called_time = ticks_ms()
        block_irq = True
irq_obj = p2.irq(handler=command_handler, trigger=(Pin.IRQ_FALLING | Pin.IRQ_RISING))

def main():
    global phase, sep_pin, init_flight_time, init_sep_time_1, init_sep_time_2
    global burning, f_separated, s_separated, landed

    while True:
        lightsleep(50)
        read()
#         print(peak_count)
        '''
        if phase == 0:  #SAFETYモードやったら
            pass
        '''
        if phase == 1:  #READYモードやったら
            if flight_pin.value() == 1:
                burning = True
                init_flight_time = ticks_ms()
                phase = 2

        elif phase == 2:    # FLIGHTモード
            if (ticks_ms() - init_flight_time) > T_BURN:
                if burning == True:
                    burning = False
                    peak_detection_timer.init(period=100, callback=peak_detection)
            if (not burning) and (apogee or ((ticks_ms() - init_flight_time) > T_APOGEE)):
                phase = 3
                peak_detection_timer.deinit()
            lightsleep(10)

        elif phase == 3:  #SEP1モード
            if not f_separated:
                sep_pin_1.value(1)
                f_separated = True
                init_sep_time_1 = ticks_ms()
            else:
                if (ticks_ms() - init_sep_time_1) > 5000:
                    sep_pin_1.value(0)
                if (ticks_ms() - init_sep_time_1) > 93000:
                    phase = 4         

        elif phase == 4:  #SEP2の時だよ
            sep_pin_2.value(1)
            #lightsleep(3000)
            if not s_separated:
                init_sep_time_2 = ticks_ms()
                s_separated = True
            else:
                if(ticks_ms()-init_sep_time_2)>3000:
                    sep_pin_2.value(0)
                    phase = 6

        elif phase == 6:  #RECOVERY
            lightsleep(100)
            if not landed:
                if (pressure > ground_press) or ((ticks_ms() - init_flight_time) > T_RECOVERY):
                    #relay = 0
                    landed = True
                    #phase = 0
            else:
                lightsleep(1000)

        elif phase == 5:  #EMERGENCY
            sep_pin_1.value(0)
            sep_pin_2.value(0)
            lightsleep(1000)

if __name__=='__main__':
    main()
