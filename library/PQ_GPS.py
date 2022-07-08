from machine import UART, lightsleep


class GPS:
    SENTENCE_LIMIT = 90
    
    def __init__(self, gps_uart):
        self.gps_uart = gps_uart
    
    def update(self, new_char):
        if new_char == '$':
            self.new_sentence()
            return None
        

    def latitude(self):
        """Format Latitude Data Correctly"""
        if self.coord_format == 'dd':
            decimal_degrees = self._latitude[0] + (self._latitude[1] / 60)
            return [decimal_degrees, self._latitude[2]]
        elif self.coord_format == 'dms':
            minute_parts = modf(self._latitude[1])
            seconds = round(minute_parts[0] * 60)
            return [self._latitude[0], int(minute_parts[1]), seconds, self._latitude[2]]
        else:
            return self._latitude

    @property
    def longitude(self):
        """Format Longitude Data Correctly"""
        if self.coord_format == 'dd':
            decimal_degrees = self._longitude[0] + (self._longitude[1] / 60)
            return [decimal_degrees, self._longitude[2]]
        elif self.coord_format == 'dms':
            minute_parts = modf(self._longitude[1])
            seconds = round(minute_parts[0] * 60)
            return [self._longitude[0], int(minute_parts[1]), seconds, self._longitude[2]]
        else:
            return self._longitude
        
    def gpgga(self):
        try:
            # UTC Timestamp
            utc_string = self.gps_segments[1]

            # Skip timestamp if receiver doesn't have on yet
            if utc_string:
                hours = (int(utc_string[0:2]) + self.local_offset) % 24
                minutes = int(utc_string[2:4])
                seconds = float(utc_string[4:])
            else:
                hours = 0
                minutes = 0
                seconds = 0.0

            # Number of Satellites in Use
            satellites_in_use = int(self.gps_segments[7])

            # Get Fix Status
            fix_stat = int(self.gps_segments[6])

        except (ValueError, IndexError):
            return False

        try:
            # Horizontal Dilution of Precision
            hdop = float(self.gps_segments[8])
        except (ValueError, IndexError):
            hdop = 0.0

        # Process Location and Speed Data if Fix is GOOD
        if fix_stat:

            # Longitude / Latitude
            try:
                # Latitude
                l_string = self.gps_segments[2]
                lat_degs = int(l_string[0:2])
                lat_mins = float(l_string[2:])
                lat_hemi = self.gps_segments[3]

                # Longitude
                l_string = self.gps_segments[4]
                lon_degs = int(l_string[0:3])
                lon_mins = float(l_string[3:])
                lon_hemi = self.gps_segments[5]
            except ValueError:
                return False

            if lat_hemi not in self.__HEMISPHERES:
                return False

            if lon_hemi not in self.__HEMISPHERES:
                return False

            # Altitude / Height Above Geoid
            try:
                altitude = float(self.gps_segments[9])
                geoid_height = float(self.gps_segments[11])
            except ValueError:
                altitude = 0
                geoid_height = 0

            # Update Object Data
            self._latitude = [lat_degs, lat_mins, lat_hemi]
            self._longitude = [lon_degs, lon_mins, lon_hemi]
            self.altitude = altitude
            self.geoid_height = geoid_height

        # Update Object Data
        self.timestamp = [hours, minutes, seconds]
        self.satellites_in_use = satellites_in_use
        self.hdop = hdop
        self.fix_stat = fix_stat

        # If Fix is GOOD, update fix timestamp
        if fix_stat:
            self.new_fix_time()

        return True
    
    def message():
        print("                                                                           ■■ ")
        print("                                  ■                                        ■  ")
        print("   ■■■■■   ■■■■■    ■■■■    ■     ■      ■            ■            ■          ")
        print("  ■■   ■   ■    ■  ■   ■■   ■  ■■■■■■    ■     ■■     ■     ■■     ■■     ■■  ")
        print(" ■■        ■    ■  ■        ■     ■■     ■      ■     ■      ■      ■     ■   ")
        print(" ■         ■    ■  ■■■      ■     ■      ■       ■    ■       ■     ■■    ■   ")
        print(" ■    ■■■  ■■■■■     ■■■    ■     ■      ■       ■    ■       ■          ■    ")
        print(" ■      ■  ■           ■■   ■■    ■      ■       ■■   ■       ■■        ■■    ")
        print(" ■■     ■  ■            ■   ■■ ■■■■■     ■■  ■        ■■  ■             ■     ")
        print("  ■■   ■■  ■       ■   ■■   ■  ■  ■■■■    ■ ■          ■ ■            ■■      ")
        print("   ■■■■■   ■        ■■■■    ■  ■■■■        ■■           ■■           ■■       ")
        print("                                                                              ")

    def staffRoll():
        lightsleep(1000);
        print("PQ_GPS.py version 1 STAFF ROLL [2022/07/07]")
        lightsleep(1000);
        print("Author : Yazu")
        lightsleep(1000);
        print("Designer : Yazu")
        lightsleep(1000);
        print("Debug : Yazu")
        lightsleep(1000);
        print("Special Thanks : Taro")
        lightsleep(3000);
        print("THANK YOU FOR USING THIS LIBRARY!!")
        lightsleep(1000);
        print("May Exciting GPS life be with you.")
        lightsleep(1000);
        print("** This is a tribute to Gaku-chan senpai(2016) **")
        lightsleep(3000)
