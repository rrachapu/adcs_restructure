import numpy as np

class guidance():
    
    # modes for guidance type
    mode = 0
    # mode 0 is Tdes DART algorithm from simulation
    mode = 1
    # mode 1 is only 0 angular velocity
    def __init__(self):
        pass

    def T_des(self):
        mode = 0
        if (mode == 0):
            return np.array([1,0,0,0])


    