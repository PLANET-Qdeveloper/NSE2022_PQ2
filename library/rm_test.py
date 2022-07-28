from PQ_RM92 import RM92A
from machine import UART, Pin, lightsleep
import time

block_flug = False
signal_timing = 2000
irq_called_time = time.ticks_ms() 

rm_uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
rm = RM92A(rm_uart, 5)

p2 = Pin(2, Pin.IN) # irq用のピン
led = Pin(25, Pin.OUT)

#rm.set_and_begin(24, 0x0001, 0x0001, 0xFFFF, 0, 13, 0, 1, 1)

def irq_test(p2):
    global block_irq
    global irq_called_time
    global signal_timing

    if (time.ticks_ms() - irq_called_time) > signal_timing:
        block_irq = False
    if not block_irq:
        buf = bytearray(4)
        rm_uart.readinto(buf)
        led.value()
        if buf[0] == 68:    # $
            if buf[3] == 10:   # \n
                cmd = buf[1]
        irq_called_time = time.ticks_ms()
        block_irq = True
        
irq_obj = p2.irq(handler=irq_test, trigger=(Pin.IRQ_FALLING | Pin.IRQ_RISING))

while True:
    #rm.send(0xFFFF, rm_data, 1)    
    lightsleep(1000)
