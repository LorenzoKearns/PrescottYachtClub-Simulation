#*******************************************************************************************************************#
#  Project: Prescott Yacht Club - Autonomous Sailing Project
#  File: Cartographer.py
#  Author: Lorenzo Kearns
#  Versions:
#   version: 0.1 4/18/2022 - Initial Program Creation
#
#  Purpose: Define dynamic map which holds boundaries and use this to determine safe operating angles
#*******************************************************************************************************************#
import pandas as pd
import numpy as np
from shapely.geometry import Point, LineString, Polygon
from utilities import Tools
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from gtts import gTTS
import os
from playsound import playsound
import time
# import geopandas as gpd
# from pyproj import CRS

class Cartographer():
    """
        Class is callable by other programs and runs by
        using inner functions included below
    """
    def __init__(self, shoreThresh):
        """
            Define initial conditions when class is called
        """
        self.SAK = Tools() # SAK is shorthand for Swiss Army Knife which the Tools class essentially is
        self.boatWidth = self.SAK.ft_to_latlon(1.5)
        self.boatLength = self.SAK.ft_to_latlon(6)
        self.safetyThresh = self.SAK.ft_to_latlon(shoreThresh) # defines the distance in feet that is safe
        # print(self.safetyThresh)
        LakeEdgesTemp = pd.read_csv('newOutputBoundaries2.csv') # read the definition of the lake from a csv
        LakeEdgesTemp = np.array(LakeEdgesTemp) # convert csv values into a numpy array
        self.LakeEdges = self.create_polygon_from_csv(LakeEdgesTemp) # create a polygon object that represents the lake
        self.listOfLakeEdgesLon, self.listOfLakeEdgesLat = self.create_polygon_from_csv(LakeEdgesTemp, get_list =  True)
        self.update_boat_pos(34.515501, -112.384901, 0)
        self.timer1 = time.perf_counter()

    def init_geopands(self):
        # Create GeoDataFrame
        lynxLake = gpd.GeoDataFrame([self.lakeEdges], geometry='geometry', crs={'init': 'epsg:4326'}, columns=['geometry'])

        # Print
        # print(lynxLake)

    def create_polygon_from_csv(self, temp_container, get_list = False):
            """
                Generate a new polygon object based on the given points
                passed into the function, can be used for any size and
                input need, i.e. works for rocks, boats, land etc
            """
            self.bound = []
            boundaryArrayLat = temp_container[:,1]
            boundaryArrayLon = temp_container[:,2]
            if (get_list == True):
                return boundaryArrayLon, boundaryArrayLat
            self.boundaryArrayLon = np.array(boundaryArrayLon)
            self.boundaryArrayLat = np.array(boundaryArrayLat)
            coordTuple = np.array((self.boundaryArrayLon,self.boundaryArrayLat)).T
            return Polygon(coordTuple)

    def update_boat_pos(self, x, y, trueHeading):
        """
            pass the current position of the boat
            and create a polygon object to
            represent it
        """
        boat_coords =  [
                  self.SAK.rotate(x, y - self.boatLength/2, self.SAK.mod360(trueHeading), x, y),
                  self.SAK.rotate(x - self.boatWidth/8, y - self.boatLength/2.1, self.SAK.mod360(trueHeading), x, y),
                  self.SAK.rotate(x - self.boatWidth/4, y - self.boatLength/3.42, self.SAK.mod360(trueHeading), x, y),
                  self.SAK.rotate(x - self.boatWidth/2.5, y, self.SAK.mod360(trueHeading), x, y),
                  self.SAK.rotate(x - self.boatWidth/2.9, y + self.boatLength/4.60  , self.SAK.mod360(trueHeading), x, y),
                  self.SAK.rotate(x, y + self.boatLength/2, self.SAK.mod360(trueHeading), x, y),
                  self.SAK.rotate(x + self.boatWidth/2.9, y + self.boatLength/4.60  , self.SAK.mod360(trueHeading), x, y),
                  self.SAK.rotate(x + self.boatWidth/2.5, y, self.SAK.mod360(trueHeading), x, y),
                  self.SAK.rotate(x + self.boatWidth/4, y - self.boatLength/3.42, self.SAK.mod360(trueHeading), x, y),
                  self.SAK.rotate(x + self.boatWidth/8, y - self.boatLength/2.1, self.SAK.mod360(trueHeading), x, y),
                  self.SAK.rotate(x, y - self.boatLength/2, self.SAK.mod360(trueHeading), x, y)]
        boatObj = Polygon(boat_coords)
        self.boatPos = boatObj

    def check_shoreline(self):
        """
            Check how close the boat is to
            the shoreline
        """
        # mytext = 'Danger will robinson! ooooooooooooooooooooo ooooooooo'
        # language = 'en'
        distShore = self.boatPos.exterior.distance(self.LakeEdges.exterior)
        # print(distShore)
        if(distShore - self.safetyThresh < 0):
            # print("Warning, shore collision imminent!")
            # if(time.perf_counter() - self.timer1 > 20):
            #     myobj = gTTS(text=mytext, lang=language, slow=False)
            #     myobj.save("welcome.mp3")
            #     os.system("welcome.mp3")
            #     self.timer1 = time.perf_counter()
            return False
        else:
            return True

    def check_collison(self):
        """
            Basis for CoursePlotters is_collision()
            function, Needs adjustment either here or in CoursePlotter
            Checks if the boat polygon has collided with the lake polygon
            by seeing if distance is equal to zero
        """
        distShore = self.boatPos.exterior.distance(self.LakeEdges.exterior)
        if (distShore - self.safetyThresh == 0):
            return True
        else:
            return False
