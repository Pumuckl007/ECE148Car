"""
ultrasonic.py

Ultrasonic (HCSR04) part for Donkeycar framework.

@author: Saurabh Kulkarni
"""

import serial

#send a stop signal when something is less than 50cm away

class HCSR04:
    '''
    Ultrasonic sensor
    '''
    def __init__(self,port='/dev/ttyACM0', threshold=200, threshold2=70):

        #do some inits here I guess???
        self.ser = serial.Serial(port, 9600)
        self.d1 = 500
        self.d2 = 500
        self.on = True
        self.threshold = threshold
        self.threshold2 = threshold2

    #blocking single-run version
    def run(self):
        #get the two distances from the arduino via serial
        #distances are in centimeters
        self.ser.reset_input_buffer()
        self.d1, self.d2 = self.ser.readline().strip().split()
        self.d1, self.d2 = int(self.d1), int(self.d2)
        #returns output stop_cmd
        return self.d1 < self.threshold or self.d2 < self.threshold2
        #return self.d2 < self.threshold2

    #this gets called after an update
    #blindly return the values
    def run_threaded(self):
        #returns stop_cmd
        return self.d1 < self.threshold or self.d2 < self.threshold2
        #return self.d2 < self.threshold2
    #independent thread version
    def update(self):
        while self.on:
            #get the two distances from the arduino via serial
            #distances are in centimeters
            self.ser.reset_input_buffer()
            oldd1 = self.d1
            oldd2 = self.d2
            self.d1, self.d2 = self.ser.readline().strip().split()
            self.d1, self.d2 = int(self.d1), int(self.d2)
            #upper
            #if self.d1 < 5:
            #    self.d1 = 500
            #if self.d2 < 5:
            #    self.d2 = oldd2
            #print("%d, %d" % ( self.d1, self.d2))

    def shutdown(self):
        #nothing to do here really
        self.on = False
        return
        
def _main():
    """
    Run with or without a port specified.
    
    python ultrasonics.py
    
    OR
    
    python ultrasonics.py /dev/tty.usbmodem14531
    
    """
    ultrasonics = None
    
    from sys import argv
    
    if(len(argv) > 1):
        port = argv[1]
        ultrasonics = HCSR04(port)
    else:
        ultrasonics = HCSR04()
    
    try:
        print("Reading ultrasonics from arduino:")
        while(True):
            d1,d2 = ultrasonics.run()
            
            print("Distance 1: {} cm \nDistance 2: {} cm".format(d1,d2))
    except(KeyboardInterrupt):
        print("Exiting...")

if(__name__ == '__main__'):
    _main()
