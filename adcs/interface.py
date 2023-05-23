"""
interface handles all outside operations to the rest of the satellite
"""

primary_gyro_select = True
primary_magnetometer_select = True


def get_current_system_days_since_epoch(self):
    pass

def set_magnetotorquers(self, mag_moment):
    pass

# stand-in function for read_mag and read_gyro funcs in adcsinterface.c 
# need to figure out how this will talk to the IMU / IMU task....
def read_imu(self):
    pass

# stand-in function until interface to self.cubesat is figured out
# need to figure that out
def read_gps(self):
    pass

# stand-in function until startracker is figured out
def read_startracker(self):
    pass


