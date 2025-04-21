import json

def get_json_data(path):
    with open(path) as f:
        data = json.load(f)
    return data

def save_json_data(path, data):
    with open(path, 'w') as f:
        json.dump(data, f, indent=4)
    
    