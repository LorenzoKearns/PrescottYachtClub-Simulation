#*******************************************************************************************************************#
#  Project: Prescott Yacht Club - Autonomous Sailing Project
#  File: boatSim.py
#  Author: Lorenzo Kearns
#  Versions:
#   version: 0.1 4/18/2022 - Initial Program Creation
#
#  Purpose: Finalize and set a true heading for the ship, also contain future waypoints
#*******************************************************************************************************************#
from Cartographer import Cartographer
from Wayfinder import WayFinder
from utilities import Tools

class CoursePlotter():
    """
        Class is callable by Main.py and uses Wayfinder
        and Cartographer classes alongside its own
        fucntions.
    """
    def __init__(self):
        self.Way = WayFinder()
        self.Map = Cartographer(50)
        self.SAK = Tools()
        self.Map.check_shoreline()
        self.tackComplete = False
        self.boundLakelon, self.boundLakelat = self.Map.listOfLakeEdgesLon, self.Map.listOfLakeEdgesLat

    def is_safe(self, x, y, bearing):
        """
            Call to cartographer to check if the current path
            is safe, to include more functionality later
        """
        self.Map.update_boat_pos(x,y,bearing)
        return self.Map.check_shoreline()

    def is_collison(self):
        return self.Map.check_collison()

    def set_bearing(self, TWS, TWA,x,y,bearing):
        if(not self.is_safe(x,y,bearing) and self.tackComplete):
            self.tackComplete = False
        if(not self.tackComplete):
            if(not self.is_safe(x,y,bearing)):
                    self.tackComplete = True
                    if(bearing < 180):
                        return self.SAK.mod360(bearing - 250)
                    else:
                        self.tackComplete = True
                        return self.SAK.mod360(bearing - 100)
            else:
                return self.Way.get_ideal_bearing(TWS,TWA)
        else:
            return bearing

    def find_shoreline(self):
        raise NotImplemented



# """
#     Main for testing program as standalone while
#     sim is being finished
# """
# def main():
#     CourseBoi = CoursePlotter()
# #
# if __name__ == '__main__':
#     main()
