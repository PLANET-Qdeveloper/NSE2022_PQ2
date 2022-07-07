from machine import Pin, I2C, lightsleep
from PQ_LPS22HB import LPS22HB

i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=100_000)

print(i2c.scan())

lps = LPS22HB(i2c)

while True:
    press = lps.read_pressure()
    temperature = lps.read_temperature()
    print("press:%f,  temperature:%f"%(press, temperature))
    lightsleep(500)