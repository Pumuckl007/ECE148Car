import pycurl
import io
import socket
import time
import select

useragent = "NTRIP UCSD-ECE148-Python"



# class DelayGGAFile:
#     def read(self):
#         print("Ever called")
#         return

class NTRIPClient:
    def __init__(self, ip, port, mountpoint, callback):
        self.ip = ip
        self.port = port
        self.mountpoint = mountpoint
        self.hasMountpoint = False
        self.callback = callback
        self.gga = "$GPGGA,055047,3251.59,N,11713.93,W,1,10,1,123.5,M,-33.8,M,5,0*6E"

    def headerFunc(self, data):
        print(data)

    def handleMoutpoints(self, data):
        str = data.decode("iso-8859-1")
        if str.find(self.mountpoint) == -1:
            print("Cant find moutpoint " + self.mountpoint)
        else:
            self.hasMountpoint = True

    def handleRTCM(self, data):
        self.callback(data)

    def createHeader(self):
        return "GET /" + self.mountpoint + " HTTP/1.1\nHost: " + self.ip + ":" + str(self.port) + "\nUser-Agent: NTRIP UCSD-ECE148-Python\nAccept: */*\r\n"

    def getMountPoints(self):
        curl = pycurl.Curl()
        curl.setopt(pycurl.URL, self.ip + ":" + str(self.port))
        curl.setopt(pycurl.TIMEOUT, 20)
        curl.setopt(pycurl.CONNECTTIMEOUT, 3)
        curl.setopt(pycurl.HEADERFUNCTION, self.headerFunc)
        curl.setopt(pycurl.WRITEFUNCTION, self.handleMoutpoints)
        curl.setopt(pycurl.USERAGENT, useragent)

        curl.perform()
        curl.close()


    def startStreamingData(self):
        if not self.hasMountpoint:
            return
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
        self.socket.connect((self.ip, self.port))
        self.socket.send(self.createHeader().encode('utf-8'))
        self.lastTime = time.time()
        self.socket.setblocking(0)


    def querry(self):
        if not self.hasMountpoint:
            return
        currentTime = time.time();
        if (currentTime-self.lastTime) > 22 :
            self.lastTime = currentTime
            self.socket.send(self.gga.encode('utf-8'))
        ready = select.select([self.socket], [], [], 0)
        if not ready[0]:
            return
        chunk = self.socket.recv(2048)
        self.handleRTCM(chunk)


def onData(data):
    print(str(len(data)) + " bytes read")

if __name__ == '__main__':
    client = NTRIPClient("rtk2go.com", 2101, "ESCADERA_NTRIP", onData)
    # client = NTRIPClient("127.0.1.1", 2102, "U-BLOX", onData)
    # client = NTRIPClient("rtk2go.com:2101", "U-BLOX", onData)
    client.getMountPoints()
    client.startStreamingData();
    while True:
        client.querry();
    print("Done")
