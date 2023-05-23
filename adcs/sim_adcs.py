import numpy as np
from numpy import linalg as la

class sim_adcs:    
    # returns adcsM values given B_dot
    def __init__(self):
        self.t2 = 0
        self.t1 = 0
        self.Bdot = np.array([0,0,0])
        self.adcsM = np.array([0,0,0])
        self.B1 = np.array([0,0,0])
        self.B2 = np.array([0,0,0])
    
    def attributes(self):
        return {"t1": self.t1,
                "t2": self.t2,
                "B1" : self.B1,
                "B2" : self.B2,
                "Bdot" : self.Bdot,
                "adcsM" : self.adcsM}
    
    def update_Bdot(self, B1, B2, t1, t2):
        self.B1 = B1
        self.B2 = B2
        self.t1 = t1
        self.t2 = t2
        self.Bdot = (B2 - B1)/(t2 - t1)
        
        return self.Bdot
        
    def calc_M(self):
        if (round(self.t1) % 2 == 1):
            Mmax = 0.2
            dM = Mmax/127
            self.adcsM = -4*self.Bdot / la.norm(self.B2)
            self.adcsM = np.sign(self.adcsM)*np.amin(np.append(np.absolute(
                np.array([self.adcsM])), Mmax*np.ones((1,3)), axis = 0), axis = 0)
            self.adcsM = np.round(self.adcsM/dM)*dM
        else:
            self.adcsM = np.zeros(3)
            
        return self.adcsM
    
    
