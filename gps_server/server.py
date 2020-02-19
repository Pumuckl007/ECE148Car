#!/env/usr/bin python
"""
server.py

Tool to visualize GPS coordinates for donkeycar.
"""

from flask import Flask, request, send_from_directory, jsonify
import json

# File that stores the GPS markers
DATA_FILE = 'data.json'

# JSON status messages
SUCCESS = {'status' : {'success': True}}
FAIL = {'status': {'success': False}}

app = Flask(__name__)

def overwrite(some_file, data):
    """ Overwrite the entire file with the new dictionary.
    """
    
    some_file.seek(0)                       # Go to the beginning of the file
    json.dump(data, some_file, indent=4)    # Dump all the data
    some_file.truncate()                    # Need this not sure why. TODO


@app.route('/')
def index():
    """ Serve the index page.
    """
    return send_from_directory('.', 'index.html')

@app.route('/favicon.ico')
def favicon():
    """ Serve the favicon.
    """
    return send_from_directory('.', 'favicon.ico')

@app.route('/markers')
def get_markers():
    """ Reads from the data file and returns the markers as JSON.
    """
    with open(DATA_FILE, 'r') as f:
        data = json.load(f)
    
    return jsonify(data)
    
@app.route('/markers/replace', methods=['GET','POST'])
def replace_markers():
    """ Replaces all markers in the data file with the ones given.
    """
    markers = request.json['markers']
    print(json.dumps(markers, indent=4))
    data = {'markers': markers}

    with open(DATA_FILE, 'r+') as f:
        overwrite(f,data)
        
    return jsonify(SUCCESS)
    
@app.route('/markers/add', methods=['GET','POST'])
def add_markers():
    """ Appends a given marker to the data file.
    """
    
    marker = request.json['marker']    

    with open(DATA_FILE, 'r+') as f:
        # Load the data from file as a dictionary.
        data = json.load(f)
        
        # Add the marker to the dictionary.
        data['markers'].append(marker)
        
        # Overwrite the entire file with the new dictionary.
        overwrite(f,data)
        
    return jsonify(SUCCESS)

@app.route('/markers/update', methods=['GET','POST'])
def update_markers():
    """ Update or add markers from the front-end
    """
    
    marker = request.json
    
    label = marker['label']

    with open(DATA_FILE, 'r+') as f:
        data = json.load(f)
        
        updated = False
        
        for index,marker_db in enumerate(data['markers']):
            if(marker_db['label'] == label):
                data['markers'][index]['position'] = marker['position']
                print("FOUND")
                updated = True
        
        if(not updated):
            data['markers'].append(marker)
            print("NOT FOUND")

        
        overwrite(f,data)
        
    return jsonify(SUCCESS)
	
if(__name__ == '__main__'):
    app.run(host='0.0.0.0', port=3148)
