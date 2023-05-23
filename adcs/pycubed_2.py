# sat.cubesat.pib.set_magnetorquers(zero_mag) # arguments needed
# sat.cubesat.pib.read_mag1()
# sat.cubesat.pib.read_mag2()
# sat.cubesat.pib.set_magnetorquers(sat.data.M_moment_out)
# sat.cubesat.pib.read_gyro1()
# sat.cubesat.pib.read_gyro2()

class pib():
    def __init__(self):
        pass
    def set_magnetorquers(mag : list):
        print(mag)
    def read_mag1():
        print([0,0,0])
    def read_mag2():
        print([0,0,0])
    def read_gyro1():
        print([0,0,0])
    def read_gyro2():
        print([0,0,0])

class Satellite():
    def __init__(self):
        a = 2