import numpy as np

class YoulaController():
    def __init__(self):
        self.Gc_states = np.zeros((1,7))
        self.GcA = 1
        self.GcB = 1
        self.GcC =1
        self.GcD = 1

    def get_commands(self,controller):
        steering_angle = 3.0
        return steering_angle