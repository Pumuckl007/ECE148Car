#!/env/usr/bin python
"""
gps_data_parser.py
"""
import json

DATA_FILE = 'data.json'

class GPSConverter:
    
    def __init__(self):
        pass
        
    def jsonify(self,file_path):
        """Stores GPS data into json format.
        """
        
        markers = []
        
        # Open the gps data file
        with open(file_path, 'r+') as f:
            lines = f.readlines()
            
            
            for index,line in enumerate(lines):
                # Each line looks like this
                # DEBUG:root:(Lat,Long): (32.88126233333333,-117.23551116666667)
            
                # Remove the words and last paren and newline.
                line = line.replace('DEBUG:root:(Lat,Long): (','')
                line = line.replace(')\n','')
                
                # Now it looks like this:
                # 32.881341,-117.235441
                
                
                # Extract the numbers and convert to floats
                lat_deg,long_deg = line.split(',')
                lat_deg = float(lat_deg)
                long_deg = float(long_deg)
                
                print(lat_deg,long_deg)
                
                # Label is index unless start or end.
                label = str(index)
                
                # Change the label for start and last point.
                if(index == 0):
                    label = "Start: {}".format(index)
                    
                if(index == (len(lines)-1)):
                    label = "End: {}".format(index)
                
                marker = {
                            "position": {
                                "lat": lat_deg, 
                                "lng": long_deg
                            }, 
                            "label": label
                          }
                          
                markers.append(marker)
        
        
        # Open the json file
        with open(DATA_FILE, 'r+') as f:
            data = {'markers': markers}

            f.seek(0)
            
            json.dump(data, f, indent=4)
            
            f.truncate()
        


def _main():
    from sys import argv
    if(len(argv) < 2):
        print("Not enough arguments.")
        return
    file_path = argv[1]
    
    converter = GPSConverter()
    
    converter.jsonify(file_path)
    
    
if(__name__ == '__main__'):
    _main()
