#!/env/usr/bin python
"""
quick_request.py
"""

import requests
import json


URL = 'http://localhost:3148'
URL = 'http://not-kiwi-bot.mothakes.com'

# File that stores the GPS markers
DATA_FILE = 'data.json'

# Empty points
CLEAR_FILE = 'clear.json'

project_gps_waypoint_1 = {
                "position": {
                    "lat": 32.8811271, 
                    "lng": -117.2342783
                }, 
                "label": "P GPS Goal 1"
              }
              
project_gps_waypoint_2 = {
                "position": {
                    "lat": 32.8812414,
                    "lng": -117.2374792
                }, 
                "label": "P GPS Goal 2"
              }
              

start =       {
                "position": {
                    "lat": 32.881322,
                    "lng": -117.235454
                }, 
                "label": "START"
              }
goal =       {
                "position": {
                    "lat": 32.881162,
                    "lng": -117.235459
                }, 
                "label": "GOAL"
              }
              
"""
lng diff: 0.000005
distance: 18.29 m
3.7 m / 0.000001

More negative is south.

"""            

def add(marker):
    r = requests.post(URL + '/markers/add', json = {'marker':marker})
    print(json.dumps(r.json(), sort_keys=True,indent=4, separators=(',', ': ')))
    
def replace(markers):
    requests.post(URL + '/markers/replace', json = {'markers':markers})
    
def clear():
    with open(CLEAR_FILE, 'r+') as f:
            data = json.load(f)
            markers = data['markers']
            replace(markers)


def _main():
    from sys import argv
    if(len(argv) < 2):
        print("Not enough arguments.")
        print("Options: replace, clear, gps1, gps2, start, goal")
        return
    
    option = argv[1]
    
    if(option == 'replace'):
        with open(DATA_FILE, 'r+') as f:
            data = json.load(f)
            markers = data['markers']
            replace(markers)
            print("Replaced markers at {}".format(URL))
        return
    
    if(option == 'clear'):
        clear()
        print("Cleared markers at {}".format(URL))
        return
    
    if(option == 'gps1'):
        add(project_gps_waypoint_1)
        print("Added ProjectGPS waypoint 1 at {}".format(URL))
        return
    
    if(option == 'gps2'):
        add(project_gps_waypoint_2)
        print("Added ProjectGPS waypoint 2 at {}".format(URL))
        return
    
    if(option == 'start'):
        add(start)
        print("Added start waypoint at {}".format(URL))
        return
    
    if(option == 'goal'):
        add(goal)
        print("Added goal waypoint at {}".format(URL))
        return
    
    print("'{}' is not a valid option.".format(option))
    
    

if(__name__ == '__main__'):
    _main()


