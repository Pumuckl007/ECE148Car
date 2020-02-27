import serial
import pynmea2
import ubx

class C099F9P:
    def __init__(self,port='/dev/ttyACM0'):
        self.ser = serial.Serial(port=port, baudrate=460800)

    def read(self):
        data = self.ser.readline()
        try:
            msg = pynmea2.parse(data.decode())
            self.readNmea(msg)
        except ParseError:
            print("Unable to parse")
            print(data)

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
    while (True):
        gps.read()

read()
