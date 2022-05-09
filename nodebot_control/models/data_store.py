

class DataStore:
    def __init__(self):
        self.dict = {}  # dictionary to hold all data

    def __str__(self):
        return self.dict.__str__()

    def receive_data(self, data):
        (id, content) = data
        self.dict[id] = content.copy()

