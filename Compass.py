class Compass():
    def __init__(self, initialBearing, trueWindInitial):
        self.vesselBearing = initialBearing
        self.windRelNorth = trueWindInitial

    def boat_frame_to_iniertial(self):
        raise NotImplemented
