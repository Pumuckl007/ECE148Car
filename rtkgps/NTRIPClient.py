import requests

class NTRIPClient:
    def __init__(self, ip, mountpoint, callback):
        self.ip = ip
        self.mountpoint = mountpoint
        self.callback = callback

    def getMountPoints(self):
        request = requests.get("http://127.0.0.2:2101/")
        print(request)
        print(request.content)


def onData(data):
    print(data)

if __name__ == '__main__':
    client = NTRIPClient("127.0.0.2:2101", "test", onData)
    client.getMountPoints()
    print("Done")
