from machine import Pin, SPI
import os
import sdcard

cs = Pin(17, Pin.OUT)
spi = SPI(0, baudrate=32000000, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
sd = sdcard.SDCard(spi, cs)
os.mount(sd, '/sd')

# ファイル書き込み
file = open('/sd/test.txt', 'w')
file.write('テストテストテスト')
file.close()
