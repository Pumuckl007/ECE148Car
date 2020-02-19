
# not-kiwi-bot
Definitely not trying to copy the Kiwi Bot (https://www.kiwicampus.com/). Class project for MAE 148 at UC San Diego.

# Accreditation
## Fritzing Parts
### GP-20u7
We use the GPS (GP-20U7) fritzing part from the [Johnny-Five](http://johnny-five.io/) javascript robotics platform. The author is Rick Waldron and the [johnny-five](https://github.com/rwaldron/johnny-five) repo has many fritzing parts.
### PCA9685
[PCA9685 Breakout Board Part](https://github.com/adafruit/Fritzing-Library/tree/master/parts/retired)
# Overall Wiring
<img alt="Full Wiring" src="images/full_wiring.jpg" height=300/>  

# Raspberry Pi 3 Pinout
<img alt="Raspberry Pi" src="images/raspberry_pi_pinout.png" height=300/>
<img alt="Raspberry Pi Expanded" src="images/raspberry_pi_pinout_expanded.png" height=300/>

# GPS GP-20U7
GPS module from [sparkfun](https://www.sparkfun.com/products/13740).
<img alt="GPS GP-20U7" src="images/GP-20U7_real.jpg" height=300/>
<img alt="PS GP-20U7 Wiring" src="images/GP-20U7_wiring_pi.jpg" height=300/>
## Wiring
| GP-20U7       | Raspberry Pi           | Reason  |
| ------------- |:-------------:| -----:|
| VCC     		| 3.3V 			| Power |
| GND      		| GND      		|   Ground|
| TX 			| RX      		|    Receive Data|
| RX 			| No Connect      |    Don't need to send to GPS. |



## Installations
```
pip install pyserial
```

##  Execute
``` 
$ python gps.py
```
Your output should look like this:  
```
(dk) **pi@ucsdrobocar03**:**~/not-kiwi-bot $** python gps.py
(32.88242833333334, -117.23458166666667)

(32.88242833333334, -117.23458166666667)

(32.88242833333334, -117.23458166666667)

(32.88242833333334, -117.23458166666667)
```

Debugging program running on fake data:
```
python gps.py fake
```

# Resources and Lessons Learned
## Project GPS
We build off the project team from Project GPS.
[Documentation](https://guitar.ucsd.edu/maeece148/index.php/Project_gps)
[Repository](https://github.com/MAE-ECE-148/ProjectGPS)
## Python
[Decoding Bytes to String](https://stackoverflow.com/questions/606191/convert-bytes-to-a-string)
[Bytes](https://stackoverflow.com/questions/6269765/what-does-the-b-character-do-in-front-of-a-string-literal)
## Raspberry Pi
[Wif Priority](https://raspberrypi.stackexchange.com/questions/58304/how-to-set-wifi-network-priority)
[Serial Ports](https://www.raspberrypi.org/documentation/configuration/uart.md) Bluetooth ON may switch serial from `/dev/ttyAMA0` to `/dev/serial0`.

