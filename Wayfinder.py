#*******************************************************************************************************************#
#  Project: Prescott Yacht Club - Autonomous Sailing Project
#  File: boatSim.py
#  Author: Lorenzo Kearns
#  Versions:
#   version: 0.1 4/18/2022 - Initial Program Creation
#
#  Purpose: Find the ideal heading based on polar charts and sensor data
#*******************************************************************************************************************#
import hrosailing.polardiagram as pol
import hrosailing.pipeline as pipe
import hrosailing.cruising as sail
import hrosailing.pipelinecomponents as pcomp
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from utilities import Tools
import numpy as np
import re

"""
    WayFinder is the weakest class currently and needs alot of restructuring and development.
"""

class WayFinder():
    """
        Class object is a part of a larger structure,
        functions included help to make up the adaptive
        path software
    """
    def __init__(self):
        self.SAK = Tools()
        self.pd = pol.from_csv("A21.csv", fmt="opencpn").symmetrize()
        data = np.array([
            self.SAK.random_shifted_pt([ws, wa, self.pd.boat_speeds[i, j]], [10, 5, 2])
            for i, ws in enumerate(self.pd.wind_angles)
            for j, wa in enumerate(self.pd.wind_speeds)
            for _ in range(6)
        ])
        self.data = data[np.random.choice(len(data), size=500)]
        self.pd = self.create_polar()



    def create_polar(self):
        """
            Creates polar charts, from data sets
        """
        ws = [4, 6, 8, 10, 12, 14, 16, 20]
        # ws = [4, 6, 8, 10, 12, 14, 16, 20]
        # self.pd.plot_polar(ws=ws, ax=plt.subplot(1, 2, 1, projection="polar"))
        # self.pd.plot_flat(ws=ws, ax=plt.subplot(1, 2, 2))
        #
        # plt.show()
        # print(self.pd)
        return self.pd

    def get_ideal_bearing(self, TWS, dir):
        """
            Legacy/ needs reworking. Should remove most hrosailing code from
            mission code so a rework of how to used polar charts for our needs
            is nescesary
        """
        heading = sail.convex_direction(self.pd, TWS, dir)
        heading_angle, percent_of_trip = [float(s) for s in re.findall(r'-?\d+\.?\d*', str(heading[0]))]
        return heading_angle

    def create_lookup_table(self, ws):
        _, wa, bsp, *sails = self.pd.get_slices(ws)
        # wa = [self.SAK.rad2deg(w) for w in wa]
        bsp = bsp.ravel()
        wa = np.rad2deg(wa)
        return wa, bsp
