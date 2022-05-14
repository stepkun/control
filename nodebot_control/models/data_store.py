

class DataStore:
    def __init__(self):
        self.dict = {}  # dictionary to hold all data

    def __str__(self):
        return self.dict.__str__()

    def receive_data(self, data):
        (id, x, y, z, b) = data
        self.dict[id]['x'] = x
        self.dict[id]['y'] = y
        self.dict[id]['z'] = z
        self.dict[id]['b'] = b

