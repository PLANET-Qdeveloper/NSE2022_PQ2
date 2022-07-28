import utime

class GPS(object):
    SENTENCE_LIMIT = 90

    def __init__(self, local_offset=0):
        """
        local_offset : タイムゾーンを入力する（int型）
        """
        self.local_offset = local_offset
        self.gps_segments = []
        self._latitude = [0, 0.0, 'N']
        self._longitude = [0, 0.0, 'W']
        self.altitude = 0.0

        self.fix_status = 0

    def gpgga(self):
        # GPGGAフォーマット例
        # $GPGGA,085120.307,A,3541.1493,N,13945.3994,E,1,08,1.0,6.9,M,35.9,M,,0000*5E
        # 085120.307 : UTC時刻8時51分20秒307
        # 3541.1493 : 緯度dddmm.mmmm
        # N : 北緯
        # 13945.3994 : 経度dddmm.mmmm
        # E : 東経
        # 1 : 位置特定品質.0=位置特定できない,1=SPS(標準測位サービス)モード,2=干渉測位方式モード
        # 08 : 使用衛星数
        # 1.0 : 水平精度低下率
        # 6.9 : アンテナの海抜高さ[m]
        # M : [m]
        # 35.9 : ジオイド高さ
        # M : [m]
        # 0000 : 作動基準地点ID
        # 5E : チェックサム

        try:
            utc_string = self.gps_segments[1]   # 085120.307
            # タイムスタンプ
            if utc_string:
                hours = (int(utc_string[0:2]) + self.local_offset) % 24
                minutes = int(utc_string[2:4])
                seconds = float(utc_string[4:])
            else:
                hours = 0
                minutes = 0
                seconds = 0.0
            
            fix_status = int(self.gps_segments[6])
            satellites_in_use = int(self.gps_segments[7])
        except (ValueError, IndexError):
            return False

        try:
            hdop = float(self.gps_segments[8])
        except (ValueError, IndexError):
            hdop = 0.0
        
        if fix_status:
            try:
                lat_string = self.gps_segments[2]
                lat_degs = int(lat_string[0:2])
                lat_mins = float(lat_string[2:])
                lat_hemi = self.gps_segments[3]

                # Longitude
                lon_string = self.gps_segments[4]
                lon_degs = int(lon_string[0:3])
                lon_mins = float(lon_string[3:])
                lon_hemi = self.gps_segments[5]
            except ValueError:
                return False
            
            try:
                altitude = float(self.gps_segments[9])
                geoid_height = float(self.gps_segments[11])
            except ValueError:
                altitude = 0
                geoid_height = 0
            
            self._latitude = [lat_degs, lat_mins, lat_hemi]
            self._longitude = [lon_degs, lon_mins, lon_hemi]
            self.altitude = altitude
            self.geoid_height = geoid_height
        
        self.timestamp = [hours, minutes, seconds]
        self.satellites_in_use = satellites_in_use
        self.hdop = hdop
        self.fix_stat = fix_status

        if fix_status:
            self.new_fix_time()

        return True

    def new_sentence(self):
        self.gps_segments = ['']
        self.active_segment = 0
        self.crc_xor = 0
        self.sentence_active = True
        self.process_crc = True
        self.char_count = 0

    def update(self, new_char):
        
        valid_sentence = False

        ascii_char = ord(new_char)  # unicodeに変換

        if 10 <= ascii_char <= 126:
            self.char_count += 1

            if new_char == '$':
                self.new_sentence()
                return None

            elif self.sentence_active:
                
                # 最後のチェックサムの*に達した場合
                if new_char == '*':
                    self.active_segment += 1
                    self.active_segment.append('')
                    return None
                
                elif new_char == ',':
                    self.active_segment += 1
                    self.active_segment.append('')
                
                else:
                    self.gps_segments[self.active_segment] += new_char
                    # CRC(Cyclic Redundancy Check:巡回冗長検査)のところ
                    # When CRC input is disabled, sentence is nearly complete
                    if not self.process_crc:

                        if len(self.gps_segments[self.active_segment]) == 2:
                            try:
                                final_crc = int(self.gps_segments[self.active_segment], 16)
                                if self.crc_xor == final_crc:
                                    valid_sentence = True
                                else:
                                    self.crc_fails += 1
                            except ValueError:
                                pass  # CRC Value was deformed and could not have been correct
                
                # Update CRC
                if self.process_crc:
                    self.crc_xor ^= ascii_char


                if valid_sentence:
                    self.clean_sentences += 1  # Increment clean sentences received
                    self.sentence_active = False  # Clear Active Processing Flag

                    if self.gps_segments[0] in self.supported_sentences:

                        # parse the Sentence Based on the message type, return True if parse is clean
                        if self.supported_sentences[self.gps_segments[0]](self):

                            # Let host know that the GPS object was updated by returning parsed sentence type
                            self.parsed_sentences += 1
                            return self.gps_segments[0]

                if self.char_count > self.SENTENCE_LIMIT:
                    self.sentence_active = False
        
        return None


    def new_fix_time(self):
        self.fix_time = utime.ticks_ms()

    
    if __name__ == "__main__":
        pass