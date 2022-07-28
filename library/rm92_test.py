from PQ_RM92 import RM92A

from PQ_RM92 import RM92A
from machine import UART, Pin

rm_uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

rm = RM92A(rm_uart)

rm.set_and_begin(1, 0x0001, 0x1234, 0, 13, 1, 3, 1)