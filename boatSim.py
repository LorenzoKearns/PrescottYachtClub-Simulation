#*******************************************************************************************************************#
#  Project: Prescott Yacht Club - Autonomous Sailing Project
#  File: boatSim.py
#  Author: Lorenzo Kearns
#  Versions:
#   version: 0.1 10/25/2021 - Initial Program Creation
#
#  Purpose: simulation environment for running boat code
#
#*******************************************************************************************************************#
#
#
#****************************************************************#
# Includes:
import tkinter as tk
from tkinter import *
import pandas as pd
import numpy as np
from numpy import random
from math import *
from CoursePlotter import CoursePlotter
from glob import *
from utilities import Tools
import threading
from Compass import Compass
#****************************************************************#
#
#
class LynxLakeSimulation(tk.Frame):
    """
        Houses the GUI for the simulation as well as defines
        how the simulation functions and stuff
    """
    def __init__(self, master=None):
        tk.Frame.__init__(self,master)
        self.init_sim_utility()
        self.plot_boundaries()
        self.init_sim_conditions()
        self.init_buttons()

    def init_sim_utility(self):
        """
            defines the initial conditions of
            things that contribute to the utilites
            of the simulation
        """
        self.CourseBoi = CoursePlotter()
        self.SAK = Tools()
        self.bound = []
        self.paused = False
        frame = Frame(root)
        frame.pack()
        button_pause = Button(frame, text="Pause/Resume", command=self.pause)
        button_pause.pack(side = LEFT)
        self.TwindLabel = Text(frame, height=1, width=22)
        self.TwindLabel.pack(side = LEFT)
        self.TwindLabel.insert(END, "Wind dir rel to north:")
        self.Twind = Text(frame, height=1, width=8)
        self.Twind.pack(side = LEFT)
        self.TheadLabel = Text(frame, height=1, width=22)
        self.TheadLabel.pack(side = LEFT)
        self.TheadLabel.insert(END, "Bearing rel to north:")
        self.Theading = Text(frame, height=1, width=8)
        self.Theading.pack(side = LEFT)
        self.Dheading = Text(frame, height=1, width=8)
        self.Dheading.pack(side = LEFT)
        self.LonLabel = Text(frame, height=1, width=12)
        self.LonLabel.pack(side = LEFT)
        self.LonLabel.insert(END, "Longitude:")
        self.LocationLon = Text(frame, height=1, width=12)
        self.LocationLon.pack(side = LEFT)
        self.LatLabel = Text(frame, height=1, width=12)
        self.LatLabel.pack(side = LEFT)
        self.LatLabel.insert(END, "Lattitude:")
        self.LocationLat = Text(frame, height=1, width=12)
        self.LocationLat.pack(side = LEFT)

        self.c = Canvas(root, width=W, height=H, bg="Green", scrollregion=(0, 0, MAXW, MAXH))

    def init_sim_conditions(self):
        """
            Defines the initial conditions that run the SIM
            sets up wind and initial bearing, stuff like that
        """
        trueHeading = 90
        x = random.randint(360)
        trueWind = self.SAK.mod360(x)
        trueWind = 350
        self.compass = Compass(trueHeading, trueWind)
        self.gpsLat, self.gpsLng = WAYPOINT0
        self.gpsLatPrev, self.gpsLngPrev = self.gpsLat, self.gpsLng
        self.angleOfSail = 0.0
        # self.x_prev, self.y_prev = self.LatLontoXY(self.gpsLat, self.gpsLng)

    def init_buttons(self):
        """
            Create the buttons that are included on the GUI
            Buttons avaiale are:
             *   Pause/Resume
             *   Vertical Scrollbar
             *   Horizontal Scrollbar
        """
        self.hbar=Scrollbar(root,orient=HORIZONTAL)
        self.hbar.pack(side=BOTTOM,fill=X)
        self.hbar.config(command=self.c.xview)
        self.vbar=Scrollbar(root,orient=VERTICAL)
        self.vbar.pack(side=RIGHT,fill=Y)
        self.vbar.config(command=self.c.yview)
        self.c.config(xscrollcommand=self.hbar.set, yscrollcommand=self.vbar.set)
        self.c.pack(side=LEFT, expand=True, fill=BOTH)

    def LatLontoXY(self,lat, lon):
        """
            Converts input coordinates into
            the reference frame of pixels,
            Essentially converts the simulation
            to function based on lat and lon coordinates
        """
        x_adj = abs(((lon - LONMIN)/SCALE_LON) * MAXW)
        y_adj = abs(MAXH - (((lat - LATMIN)/SCALE_LAT) * MAXH))
        return x_adj, y_adj

    def plot_boundaries(self):
        """
            Visualize the Lakes edges, this
            boundaries version is scaled to
            the pixels of the visual engine
            and differs from the saved map
            in Cartographer, but is representative
            in scale
        """
        self.boundaryArrayLat = self.CourseBoi.boundLakelat
        self.boundaryArrayLon = self.CourseBoi.boundLakelon
        for i in range(len(self.boundaryArrayLat)):
            lon,lat = self.LatLontoXY(self.boundaryArrayLat[i], self.boundaryArrayLon[i])
            self.bound.append(lon)
            self.bound.append(lat)
        self.c.create_polygon(self.bound, fill = 'blue', outline = 'green', width = 2, tags = 'boundary')

    def pause(self):
        """
            Simple function to pause/unpause the simulation
        """
        self.paused = not self.paused

    def render(self):
        """
            Renders the boat and updates its position and orientation
        """
        x, y = self.LatLontoXY(self.gpsLat, self.gpsLng)
        points = [self.SAK.rotate(x, y - BOAT_LENGTH/2, self.SAK.mod360(self.compass.vesselBearing), x, y),
                    self.SAK.rotate(x - BOAT_WIDTH/8, y - BOAT_LENGTH/2.1, self.SAK.mod360(self.compass.vesselBearing), x, y),
                    self.SAK.rotate(x - BOAT_WIDTH/4, y - BOAT_LENGTH/3.42, self.SAK.mod360(self.compass.vesselBearing), x, y),
                    self.SAK.rotate(x - BOAT_WIDTH/2.5, y, self.SAK.mod360(self.compass.vesselBearing), x, y),
                    self.SAK.rotate(x - BOAT_WIDTH/2.9, y + BOAT_LENGTH/4.60  , self.SAK.mod360(self.compass.vesselBearing), x, y),
                    self.SAK.rotate(x, y + BOAT_LENGTH/2, self.SAK.mod360(self.compass.vesselBearing), x, y),
                    self.SAK.rotate(x + BOAT_WIDTH/2.9, y + BOAT_LENGTH/4.60  , self.SAK.mod360(self.compass.vesselBearing), x, y),
                    self.SAK.rotate(x + BOAT_WIDTH/2.5, y, self.SAK.mod360(self.compass.vesselBearing), x, y),
                    self.SAK.rotate(x + BOAT_WIDTH/4, y - BOAT_LENGTH/3.42, self.SAK.mod360(self.compass.vesselBearing), x, y),
                    self.SAK.rotate(x + BOAT_WIDTH/8, y - BOAT_LENGTH/2.1, self.SAK.mod360(self.compass.vesselBearing), x, y)]
        self.c.delete('boat') # remove the last intance of the boat and then create a new boat
        if(self.CourseBoi.is_safe(self.gpsLng, self.gpsLat, self.compass.vesselBearing)):
            self.c.create_polygon(points, fill='white', width=0, tags='boat')
        else:
            self.c.create_polygon(points, fill='red', width=0, tags='boat')
        if(self.CourseBoi.is_collison()):
            self.paused = not self.paused
        #Update the position of the rudder
        points = [*self.SAK.rotate(*self.SAK.rotate(x, y + BOAT_LENGTH/2, rudder, x, y + BOAT_LENGTH/2), self.SAK.mod360(self.compass.vesselBearing), x, y),
                    *self.SAK.rotate(*self.SAK.rotate(x, y + BOAT_LENGTH/2 + RUDDER_LENGTH, rudder, x, y + BOAT_LENGTH/2), self.SAK.mod360(self.compass.vesselBearing), x, y)]
        self.c.delete('rudder')
        self.updateLine(*points, fill='black', width=2, tags = 'rudder')
        #sail
        self.c.delete('sail')
        points = [x, y, *self.SAK.rotate(x, y - SAIL_LENGTH, self.angleOfSail, x, y)]
        self.updateLine(*points, fill='black', width=2, tags = 'sail')

    def updateLine(self, x0, y0, x1, y1, fill, width, tags = None):
            """
                Straight up makes a line, thats about it
            """
            self.c.create_line(x0, y0, x1, y1, fill=fill, width=width, tags=tags)

    def refreshCycle(self):
        """
            Threaded cycle which handles scorll lock to the boat and other aspects of the simulation
        """
        x, y = self.LatLontoXY(self.gpsLat, self.gpsLng)
        if not self.paused:
            self.refresh()
            #Generates a trail behind the boat
            self.c.create_line(self.x_prev, self.y_prev, x, y, fill='black', width=1, tags='boat_path')
            self.x_prev, self.y_prev = x, y
            #scroll functionality
            x0 = self.hbar.get()[0] * MAXW
            y0 = self.vbar.get()[0] * MAXH
            if x < x0 or x > x0 +100 or y < y0 or y > y0 +100 :
                self.c.xview_moveto((x - W/2)/MAXW)
                self.c.yview_moveto((y - H/6)/MAXH)
                root.update()
            #wind direction arrow object, tracks to boat
            self.c.delete('wind_dir')
            c_windDir = self.c.create_line( x - 6*HEADING_ARROW*sin(self.SAK.deg2rad(self.compass.windRelNorth)), y + 6*HEADING_ARROW*cos(self.SAK.deg2rad(self.compass.windRelNorth)),
                          x + 6*HEADING_ARROW*sin(self.SAK.deg2rad(self.compass.windRelNorth)), y - 6*HEADING_ARROW*cos(self.SAK.deg2rad(self.compass.windRelNorth)), fill='gray', width=2, arrow=tk.LAST, tags = 'wind_dir')
        threading.Timer(dt_refresh, self.refreshCycle).start()

    def refresh(self):
        """
            Update labels and call the render function to
            redraw items that make up the visual boat
        """
        self.render()
        self.Twind.delete('1.0', END)
        self.Twind.insert(END, self.compass.windRelNorth)
        self.Theading.delete('1.0', END)
        self.Theading.insert(END, self.compass.vesselBearing)
        self.Dheading.delete('1.0', END)
        self.Dheading.insert(END, self.angleOfSail)
        self.LocationLat.delete('1.0', END)
        self.LocationLat.insert(END, self.gpsLat)
        self.LocationLon.delete('1.0', END)
        self.LocationLon.insert(END, self.gpsLng)

    def move_forward(self):
        self.compass.vesselBearing = self.CourseBoi.set_bearing(WIND_KNOTTS, self.compass.windRelNorth,self.gpsLng, self.gpsLat,self.compass.vesselBearing)
        self.angleOfSail = self.SAK.mod360(self.compass.windRelNorth - self.compass.vesselBearing)
        boatSpeed = abs(MAX_SPEED*sin(self.SAK.deg2rad(self.angleOfSail)))
        if self.angleOfSail < 225 and self.angleOfSail > 135:
            boatSpeed = -boatSpeed
        dist = boatSpeed * dt
        lat1 = self.SAK.deg2rad(self.gpsLat)
        lng1 = self.SAK.deg2rad(self.gpsLng)
        bearing = self.SAK.deg2rad(self.compass.vesselBearing)
        lat2 = asin(sin(lat1)*cos(dist) + cos(lat1)*sin(dist)*cos(bearing))
        lng2 = lng1 + atan2(sin(bearing)*sin(dist)*cos(lat1), cos(dist) - sin(lat1)*sin(lat2))
        self.gpsLat, self.gpsLng = self.SAK.rad2deg(lat2), self.SAK.rad2deg(lng2)
        threading.Timer(dt, self.move_forward).start()


"""
    Main loop for GUI, keeps it all spinning along
"""
root = Tk()
root.wm_state('zoomed')
app = LynxLakeSimulation(master = root)
app.render()
app.x_prev, app.y_prev = app.LatLontoXY(app.gpsLat, app.gpsLng)
app.move_forward()
app.refreshCycle()
app.mainloop()
