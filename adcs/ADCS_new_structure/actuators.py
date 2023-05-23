class actuators():
    def __init__(self):
        self.ctrl = [0,0,0]

    def set_magnetorquers(self, mag):
        print("setting magnetorquers: " + str(mag))
        self.ctrl = mag