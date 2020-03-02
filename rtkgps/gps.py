import serial
import pynmea2
import ubx
from NTRIPClient import NTRIPClient

class UBXMidigatorReader:
    def __init__(self, ser, callback):
        self.ser = ser
        self.callback = callback
        self.buffer = b''

    def read(self, n=1):
        nextChars = self.ser.read(n)
        nextCharsDup = [nextChars[i:i+1] for i in range(len(nextChars))]
        success = False
        for char in nextCharsDup:
            if char == b'\n':
                success = self.callback(self.buffer)
                self.buffer = b''
            else :
                self.buffer = self.buffer + char
        return nextChars



class C099F9P:
    def __init__(self,port='/dev/ttyACM0'):
        self.ser = serial.Serial(port=port, baudrate=460800)
        self.mediator = UBXMidigatorReader(self.ser, self.parseNmea)
        self.reader = ubx.Reader(self.mediator.read)

    def setUpdateRate(self):
        print(ubx.descriptions.cfg_rate)
        msg = ubx.Message(ubx.descriptions.cfg_rate.description, {'measRate':100, 'navRate':0x01, 'timeRef': 0x01})
        s = msg.serialize();
        print(":".join("{:02x}".format(c) for c in s))
        self.ser.write(s)

    def read(self):
        try:
            rawmessage = self.reader.read_rawmessage()
            self.parseRaw(rawmessage)
        except ubx.ChecksumError:
            print("Unable to parse")

    def parseRaw(self, raw):
        try:
            message = ubx.default_parser.parse(raw)
        except KeyError:
            print("No idea of the key")
        except ubx.PayloadError:
            print("Bad payload")
        else:
            print(message)

    def parseNmea(self, rawNmea):
        success = True
        try:
            nmea = pynmea2.parse(rawNmea.decode())
            self.readNmea(nmea)
        except pynmea2.ParseError:
            success = False
            print("Not nema!")
            print(rawNmea)
        except UnicodeDecodeError:
            success = False
            print("Not nmea!")
        return success

    def readNmea(self, nmea):
        if not nmea.sentence_type == "GGA":
            return
        lat = self.convertLatLon(nmea.lat)
        lon = self.convertLatLon(nmea.lon)
        print("at " + str(lat) + nmea.lat_dir + ", " + str(lon) + nmea.lon_dir + " with " + nmea.num_sats + " sats.")

    def convertLatLon(self, latLon):
        decimalIdx = latLon.index('.') - 2
        major = latLon[0:(decimalIdx)]
        minor = latLon[decimalIdx:]
        majorFloat = float(major)
        minorFloat = float(minor)
        minorFloat = minorFloat / 60
        print(major + " " + minor)
        return majorFloat + minorFloat

def lookAtNema(nmea):
    if nmea.sentence_type == "GGA":
        print("at " + nmea.lat + nmea.lat_dir + ", " + nmea.lon + nmea.lon_dir + " with " + nmea.num_sats + " sats.")
    # print(nema.__dict__)

def read():
    gps = C099F9P()
    gps.setUpdateRate()
    # return
    while (True):
        gps.read()
        print("Read")

read()
