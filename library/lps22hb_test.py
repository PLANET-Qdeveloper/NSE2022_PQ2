from machine import Pin, I2C, lightsleep
from PQ_LPS22HB import LPS22HB

# Pico側のピン配置
# 本プログラムではSDAはGP20, SCLはGP21へ接続してます。変えるときは下の引数を変えてね。
i2c = I2C(0, scl=Pin(21), sda=Pin(20))

# インスタンスの生成。
lps = LPS22HB(i2c)

while True:
    press = lps.read_pressure()
    temperature = lps.read_temperature()
    print("press:%f,  temperature:%f"%(press, temperature))

    lightsleep(500)