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
import matplotlib.pyplot as plt
from utilities import Tools
import numpy as np
import re


class Wayfinder():
    """
        Class object is a part of a larger structure,
        functions included help to make up the adaptive
        path software
    """
    def __init__(self):
        self.SAK = Tools()
        self.pd = pol.from_csv("testdata.csv", fmt="hro").symmetrize()
        data = np.array([
            self.SAK.random_shifted_pt([ws, wa, self.pd.boat_speeds[i, j]], [10, 5, 2])
            for i, ws in enumerate(self.pd.wind_angles)
            for j, wa in enumerate(self.pd.wind_speeds)
            for _ in range(6)
        ])
        self.data = data[np.random.choice(len(data), size=500)]
        self.pd = self.create_polar()

    def create_polar(self):
        ws = [6, 8, 10, 12, 14, 16, 20]
        self.pd.plot_polar(ws=ws, ax=plt.subplot(1, 2, 1, projection="polar"))
        self.pd.plot_convex_hull(ws=ws, ax=plt.subplot(1, 2, 2, projection="polar"))
        return self.pd


"""
    Main for testing program as standalone while
    the other programs are unfinished
"""
def main():
    Way = Wayfinder()
#
if __name__ == '__main__':
    main()
