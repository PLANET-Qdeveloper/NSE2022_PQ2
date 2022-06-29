from machine import I2C
import utime

# LPS22HBのアドレスはGND接続でお願い
LPS_I2C_ADDR_LOW = 0x5C
LPS_I2C_ADDR_HIGH = 0x5D

# LPS22HBのレジスタ
LPS_INT_CFG = 0x0B  # Interrupt register
LPS_THS_P_L = 0x0C  # Pressure threshold registers
LPS_THS_P_H = 0x0D
LPS_WHO_AM_I = 0x0F  # 0xB1が読めれば正常
LPS_CTRL_REG1 = 0x10  # ここに0b0001 0000を書きこむと動作状態になる(1Hz)
LPS_CTRL_REG2 = 0x11
LPS_CTRL_REG3 = 0x12
LPS_FIFO_CTRL = 0x14  # FIFO configuration register
LPS_REF_P_XL = 0x15  # Reference pressure registers
LPS_REF_P_L = 0x16
LPS_REF_P_H = 0x17
LPS_RPDS_L = 0x18  # Pressure offset registers
LPS_RPDS_H = 0x19
LPS_RES_CONF = 0x1A  # Resolution register
LPS_INT_SOURCE = 0x25  # Interrupt register
LPS_FIFO_STATUS = 0x26  # FIFO status register
LPS_STATUS = 0x27  # Status register
LPS_PRESS_OUT_XL = 0x28  # 気圧データが入るレジスタ(以下3バイト24ビット)
LPS_PRESS_OUT_L = 0x29
LPS_PRESS_OUT_H = 0x2A
LPS_TEMP_OUT_L = 0x2B  # 温度データが入るレジスタ
LPS_TEMP_OUT_H = 0x2C
LPS_RES = 0x33  # Filter reset register

class LPS22HB:

    def __init__(self, i2c, AD0):
        if AD0 == 0:
            self.address = LPS_I2C_ADDR_LOW
        elif AD0 == 1:
            self.address = LPS_I2C_ADDR_HIGH
        else:
            print('AD0の値が正しく有りません.HIGHの場合1,LOWの場合0です.')
        if i2c is None:
            raise ValueError('An I2C object is required.')
        self.i2c = i2c
        buf = bytearray(1)
        buf[0] = 0x40
        # 出力データのレート(ODR)を50Hzに設定(p36参照)
        self.i2c.writeto_mem(self.address, LPS_CTRL_REG1, buf)    

    # LPS22HBが正常に起動しているかチェックする関数.戻り値Trueだと正常.
    def test(self):
        buf = bytearray(1)
        self.i2c.readfrom_mem_into(self.address, LPS_WHO_AM_I, buf)
        if buf == 0xB1:
            return True
        else:
            return False
    
    def read_pressure(self):
        status = self.i2c.readfrom_mem(self.address, LPS_STATUS, 1)
        if (status[0] & 0x01) == 0x01:  # Pressure data available(p44参照)
            bytes_xl = self.readfrom_mem(self.address, LPS_PRESS_OUT_XL)
            bytes_l = self.readfrom_mem(self.address, LPS_PRESS_OUT_L)
            bytes_H = self.readfrom_mem(self.address, LPS_PRESS_OUT_H)
            PRESS_DATA = ((bytes_xl[0] << 16)+(bytes_l << 8)+bytes_H)/4096.0
        pressure = PRESS_DATA   #わざわざこうする必要ある？
        return pressure

    def read_temperature(self):
        buf = [2]
        if (self.readfrom(LPS_STATUS) & 0x02) == 0x02:   # Temperature data available(p44参照)
            buf[0] = self.readfrom(LPS_TEMP_OUT_L)
            buf[1] = self.readfrom(LPS_TEMP_OUT_H)
            TEMP_DATA = ((buf[1] << 8)+buf[0])/100.0
        temperature = TEMP_DATA
        return temperature