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
import time

class NoGoZone():
    def __init__(self, adjWindPos, adjWindNeg):
        self.max = adjWindPos
        self.min = adjWindNeg

class CoursePlotter():
    """
        Class is callable by Main.py and uses Wayfinder
        and Cartographer classes alongside its own
        fucntions.
    """
    def __init__(self, start, end, TWA):
        self.Way = WayFinder()
        self.Map = Cartographer(50)
        self.SAK = Tools()
        self.mode = "straight"
        self.final_adjust = False
        self.twa = TWA
        self.directPathOrient = self.SAK.direction_lookup(start[0], start[1], end[0], end[1])
        self.dist = self.SAK.Distance(start[0], start[1], end[0], end[1])
        self.pos = [start[0], start[1]]
        self.end = [end[0], end[1]]
        print(self.dist)
        windPlus = self.SAK.mod360((TWA+180)+45) # adjusted wind plus 45
        windSub = self.SAK.mod360((TWA+180)-45) # adjusted wind minus 45
        self.NoGoRange = NoGoZone(windPlus, windSub)
        print(self.NoGoRange.min, self.NoGoRange.max)
        self.tackComplete = False
        self.boundLakelon, self.boundLakelat = self.Map.listOfLakeEdgesLon, self.Map.listOfLakeEdgesLat
        self.startWayp = start
        self.endWayp = end
        self.set_path_type()
        self.timer1 = time.perf_counter()
        self.timer2 = time.perf_counter()

    def is_safe(self, x, y, bearing):
        """
            Call to cartographer to check if the current path
            is safe, to include more functionality later
        """
        self.Map.update_boat_pos(x,y,bearing)
        self.directPathOrient = self.SAK.direction_lookup(self.pos[0], self.pos[1], self.end[0], self.end[1])
        if(time.perf_counter() - self.timer2 > 2):
            self.timer2 = time.perf_counter()
            print(self.directPathOrient)
            print(self.dist)
        if(not self.final_adjust):
            if(self.dist < 100 and not self.in_no_go_range(self.SAK.mod360(self.directPathOrient-10))):
                return self.SAK.mod360(self.directPathOrient)
            if(not self.Map.check_shoreline()):
                bearing = self.adjust_path(bearing)
            return bearing
        else:
            return bearing

    def adjust_path(self, bearing):
        """
            Determine how to adjust bearing based
            on the mode of travel in effect
        """
        if(time.perf_counter() - self.timer1 > 1):
            self.timer1 = time.perf_counter()
            if(self.mode == "straight"):
                self.straight_adjust()
            elif(self.mode == "beat"):
                bearing = self.beat_adjust()
                return bearing
        return bearing

    def straight_adjust(self):
        """
            Future implement: Determines how to adjust bearing when a direct path is being followed
            Ideally should return the boat to the original path when object is avoided. could be used
            conditionallt in beat as well in later versions of the code
        """
        raise NotImplemented

    def beat_adjust(self):
        """
            determine what direction to beat in.
            unsafe right now, does not include jibe vs
            tack
        """
        if(self.beat_dir == "right"):
            self.beat_dir = "left"
            return self.SAK.mod360(self.NoGoRange.min - 12)
        elif(self.beat_dir == "left"):
            self.beat_dir = "right"
            return self.SAK.mod360(self.NoGoRange.max + 12)

    def is_collison(self):
        """
            *** Simulation specific, remove in mission code ***
            item check if the boat has collided with the shore,
            future implement: boatSim will stop and print
            error message noting an edge case has occured
            This implementation should say what the conditions
            where at the crash to determine what logic needs to be fixed
        """
        return self.Map.check_collison()

    def set_path_type(self):
        """
            Called only on initialization or great change in wind direction
            Determines what type of path, i.e beating, curved, straight, etc
            is best for the current wind conditions to reach the desired waypoint
        """
        print(self.directPathOrient)
        if(self.in_no_go_range(self.directPathOrient)):
            print("beating required")
            self.mode = "beat"
            self.beat_dir = "right"
            return self.SAK.mod360(self.NoGoRange.max + 12)
        else:
            return self.directPathOrient

    def in_no_go_range(self, directPath):
        if(self.twa <= 225 and self.twa > 134):
            if(directPath < self.NoGoRange.max or directPath > self.NoGoRange.min):
                return True
            else:
                return False
        else:
            if(directPath < self.NoGoRange.max and directPath > self.NoGoRange.min):
                return True
            else:
                return False

    def find_shoreline(self):
        """
            Future implement: should include the ability to check ahead
            of the boat some distance and see if a collision would occur
            if a path was taken, needs other functions likely.
            End goal is to test certain possible movements such as jibes
            and potential courses to see if the heading needs additional adjustment
        """
        raise NotImplemented

    def check_dist_to_target(self, posLat, posLon, end):
        self.pos[0], self.pos[1] = posLat, posLon
        self.end[0], self.end[1] = end[0], end[1]
        self.dist = self.SAK.Distance(posLat, posLon, end[0], end[1])
        return self.dist

    def define_speed(self, TWS):
        self.Way.create_lookup_table(TWS)



# """
#     Main for testing program as standalone while
#     sim is being finished
# """
# def main():
#     CourseBoi = CoursePlotter()
# #
# if __name__ == '__main__':
#     main()
