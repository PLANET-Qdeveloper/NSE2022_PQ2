from machine import Pin, UART, lightsleep

from micropyGPS import MicropyGPS

gps_module = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

my_gps = MicropyGPS(9)

latitude = ''
longitude = ''

def convert(parts):
    if (parts[0] == 0):
        return None
    data = parts[0]+(parts[1]/60.0)
    # parts[2] contain 'E' or 'W' or 'N' or 'S'
    if (parts[2] == 'S'):
        data = -data
    if (parts[2] == 'W'):
        data = -data

    data = '{0:.6f}'.format(data) # 小数点以下6桁に変更
    return str(data)
    
def get_gps():
    global latitude, longitude 
    length = gps_module.any()
    if length>0:
        b = gps_module.read(length)
        for x in b:
            msg = my_gps.update(chr(x))

    latitude = convert(my_gps.latitude)
    longitude = convert(my_gps.longitude)
    
    t = my_gps.timestamp
    #t[0] => hours : t[1] => minutes : t[2] => seconds
    gpsTime = '{:02d}:{:02d}:{:02}'.format(t[0], t[1], t[2])
    gpsdate = my_gps.date_string('long')
    
while True:
    get_gps()
    # 最初Noneってでるかも
    print('Lat:', latitude)
    print('Lng:', longitude)
    lightsleep(1000)
