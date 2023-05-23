import data_types as tp
import numpy as np

class sensors():
    def __init__(self):
        self.att = 0
        self.mag = [0,0,0]
        self.gyro = [0,0,0]
        self.time = 0

    def read_mag1(self):
        return self.mag
    
    def read_mag2(self):
        return self.mag

    def read_gyro1(self):
        return self.gyro
    
    def read_gyro2(self):
        return self.gyro

    def get_time(self):
        return self.time

    def measureB(self, sat):
        sat.data.mag1_meas.iterator += 1
        sat.data.mag1_meas.iterator %= tp.RING_SIZE
        # mag = [0] * 3
        # read mag1
        sat.data.mag1_meas.data[sat.data.mag1_meas.iterator] = np.array(sat.sensors.read_mag1())
        sat.data.mag2_meas.data[sat.data.mag2_meas.iterator] = np.array(sat.sensors.read_mag2())

        sat.data.mag1_meas.time[sat.data.mag1_meas.iterator] = self.time
        sat.data.mag2_meas.time[sat.data.mag2_meas.iterator] = self.time
        return sat.data

    def read_gyros(self, sat):
        if (sat.data.status == tp.status_t.ERROR):
            return sat.data
    
        sat.data.gyro1_meas = sat.sensors.read_gyro1()
        sat.data.gyro2_meas = sat.sensors.read_gyro2()

        sat.state_error(False)
        return sat.data
        
