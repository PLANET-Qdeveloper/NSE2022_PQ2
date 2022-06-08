from machine import UART, Pin
import time

# 使い方
# import PQ_RM92A
# rm92a = RM92A(1, 115200, 8, 9)
# rm92a.begin()
# rm92a.send(tx_data)

class RM92A():
    def __init__(self, rm_uart):   # コンストラクタの宣言
        self.rm = rm_uart
        #self.cmd_buf = bytearray(6)

    def set_and_begin(self, ch, panid, dst, unit_mode, power, bw, factor, dt_mode):
        time.sleep(0.5) # RM92Aの起動待ち.不要かも.
        self.rm.write(b"\r\n")
        time.sleep(0.1)
        self.rm.write(b"1\r\n")  # ModeはLoRaのみ
        time.sleep(0.1)

        #-----------
        # set CH
        #-----------
        self.rm.write(b"a\r\n")
        time.sleep(0.1)
        self.rm.write("{}\r\n".format(ch))
        time.sleep(0.1)

        #-----------
        # set PANID
        #-----------
        self.rm.write(b"b\r\n")
        time.sleep(0.1)
        self.rm.write(b"1\r\n")
        time.sleep(0.1)
        self.rm.write("{}\r\n".format(panid))
        time.sleep(0.1)

        #-----------
        # set Dst. 0xFFFFのときは全ノードへ一斉送信.
        #-----------
        self.rm.write(b"d\r\n")
        time.sleep(0.1)
        self.rm.write("{}\r\n".format(dst))
        time.sleep(0.1)
        
        #-----------
        # set unit-mode 0:parent, 1:child
        #-----------
        self.rm.write(b"e\r\n")
        time.sleep(0.1)
        self.rm.write("{}\r\n".format(unit_mode))
        time.sleep(0.1)

        #-----------
        # set TX-Power, BandWidth and Factor
        #-----------
        self.rm.write(b"g\r\n")
        time.sleep(0.1)
        self.rm.write(b"1\r\n")
        time.sleep(0.1)
        self.rm.write("{}\r\n".format(power))
        time.sleep(0.1)

        self.rm.write(b"g\r\n")
        time.sleep(0.1)
        self.rm.write(b"2\r\n")
        time.sleep(0.1)
        self.rm.write("{}\r\n".format(bw))
        time.sleep(0.1)

        self.rm.write(b"g\r\n")
        time.sleep(0.1)
        self.rm.write(b"3\r\n")
        time.sleep(0.1)
        self.rm.write("{}\r\n".format(factor))
        time.sleep(0.1)

        #-----------
        # set dt mode
        #-----------
        self.rm.write(b"i\r\n")
        time.sleep(0.1)
        self.rm.write("{}\r\n".format(dt_mode))
        time.sleep(0.1)

        #-----------
        # set output.
        #-----------
        self.rm.write(b"l\r\n")
        time.sleep(0.1)
        self.rm.write(b"1\r\n")
        time.sleep(0.1)
        self.rm.write(b"1\r\n")  # RSSI disable
        time.sleep(0.1)

        self.rm.write(b"l\r\n")
        time.sleep(0.1)
        self.rm.write(b"2\r\n")
        time.sleep(0.1)
        self.rm.write(b"1\r\n")   # Transfer Adress disable
        time.sleep(0.1)

        #-------------
        # save settings and start
        #-------------
        self.rm.write(b"x\r\n")
        time.sleep(0.1)
        self.rm.write(b"s\r\n")
        print("complite")
        return

    def begin(self):
        time.sleep(0.5) # RM92Aの起動待ち.不要かも.
        self.rm.write(b"\r\n")
        time.sleep(0.1)
        self.rm.write(b"1\r\n")  # ModeはLoRaのみ
        time.sleep(0.1)
        self.rm.write(b"y\r\n")
        time.sleep(0.1)
        self.rm.write(b"s\r\n")  # start!!!
        return 0

    rx_buf = []
    def rx_update(self):
        if rx_read_lock == False:
            while self.rm.any()>0:
                data = self.rm.read(1)   # 一文字ずつ読む
                self.rx_buf[rx_write_p] = data
                if data == "\n":
                    rx_write_p = 0
                    rx_read_lock = True
                    break
                else:
                    rx_write_p =+ 1
        return rx_read_lock

    def readData(self):
        return 0

    # Pico -> RM92 >>>>
    def send(self, dst, tx_data, size):
        tx_buf = [0]*(size+6)
        tx_buf[0] = "@"
        tx_buf[1] = "@"
        tx_buf[2] = "{}".format(size)
        tx_buf[3] = bin((dst >> 8) & 0xff)
        tx_buf[4] = bin((dst >> 0) & 0xff)
        for i in range(size):
            tx_buf[i+5] = "{}".format(tx_data[i])
        tx_buf[size+5] = "{}".format(0xAA)
        for i in range(size+6):
            self.rm.write(tx_buf[i])
            print(tx_buf[i])