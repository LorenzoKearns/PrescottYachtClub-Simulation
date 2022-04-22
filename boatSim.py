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
from utilities import Tools
import threading
from Compass import Compass
from turtle import *
#****************************************************************#
#
#
## Per run changable constants:
STARTING_POS =   34.515501, -112.384901
WAYPOINT1 =   34.521701, -112.3858601
WIND_SPEED_KNOTTS = 20
SCALE = 10

## Strict definitions:
W = 1051.05971816
H = 1200.0
MAXW = SCALE*W
MAXH = SCALE*H
LATMIN = 34.5150517
LONMIN = -112.388989
SCALE_LON = 0.00698
SCALE_LAT = 0.0079691
# Boat size parameters
BOAT_LENGTH = ((5.6667*1.0)/2915) * MAXH
BOAT_BOW = ((5.6667*0.3)/2915) * MAXH
BOAT_WIDTH = (1.0/2018) * MAXW
RUDDER_LENGTH = (1.2/2915) * MAXH
SAIL_LENGTH = (1.8/2018) * MAXW
DOT_RADIUS = (0.2/2018) * MAXW
WIND_COMPASS = (2.8/2915) * MAXH
HEADING_ARROW = (2.8/2915) * MAXH * 10/SCALE
#rudder movement
RUDDER_RESPONSE = 5000000.005
RUDDER_MIN_ANGLE = -30
RUDDER_MAX_ANGLE = 30
# time refresh
dt = 0.001
dt_refresh = 4*dt

class LynxLakeSimulation(tk.Frame):
    """
        Houses the GUI for the simulation as well as defines
        how the simulation functions and stuff
    """
    def __init__(self, master=None):
        tk.Frame.__init__(self,master)
        self.init_sim_utility()
        self.init_sim_conditions()
        self.init_buttons()
        self.CourseBoi = CoursePlotter(STARTING_POS, WAYPOINT1, self.compass.windRelNorth)
        self.plot_boundaries()
        self.compass.vesselBearing = self.CourseBoi.set_path_type()
        self.CourseBoi.define_speed(WIND_SPEED_KNOTTS)
        self.desHeading = self.compass.vesselBearing

    def init_sim_utility(self):
        """
            defines the initial conditions of
            things that contribute to the utilites
            of the simulation
        """
        self.SAK = Tools()
        self.bound = []
        self.paused = False
        self.stopped = False
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
        self.finishedLab = Text(frame, height=1, width=40)
        self.finishedLab.pack(side = LEFT)
        self.finishedLab.delete('1.0', END)
        # self.finishedLab.insert(END, "Waypoint Reached, Simulation complete")
        self.c = Canvas(root, width=W, height=H, bg="Green", scrollregion=(0, 0, MAXW, MAXH))

    def init_sim_conditions(self):
        """
            Defines the initial conditions that run the SIM
            sets up wind and initial bearing, stuff like that
        """
        trueHeading = 0
        x = random.randint(360)
        trueWind = self.SAK.mod360(x)
        self.compass = Compass(trueHeading, trueWind)
        self.gpsLat, self.gpsLng = STARTING_POS
        self.gpsLatPrev, self.gpsLngPrev = self.gpsLat, self.gpsLng
        self.angleOfSail = 0.0
        self.speed = WIND_SPEED_KNOTTS/1000000
        self.rudder = 0.0
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

    def adjustRudder(self):
        if (abs(self.compass.vesselBearing - self.desHeading) < 0.5):
            self.rudder = 0
        elif (self.compass.vesselBearing < self.desHeading):
            self.rudder = RUDDER_MAX_ANGLE
        elif (self.compass.vesselBearing > self.desHeading):
            self.rudder = RUDDER_MIN_ANGLE

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
        # if(self.CourseBoi.is_safe(self.gpsLng, self.gpsLat, self.compass.vesselBearing)):
        self.c.create_polygon(points, fill='white', width=0, tags='boat')
        # else:
        #     self.c.create_polygon(points, fill='red', width=0, tags='boat')
        if(self.CourseBoi.is_collison()):
            self.paused = not self.paused
        #Update the position of the rudder
        points = [*self.SAK.rotate(*self.SAK.rotate(x, y + BOAT_LENGTH/2, self.rudder, x, y + BOAT_LENGTH/2), self.SAK.mod360(self.compass.vesselBearing), x, y),
                    *self.SAK.rotate(*self.SAK.rotate(x, y + BOAT_LENGTH/2 + RUDDER_LENGTH, self.rudder, x, y + BOAT_LENGTH/2), self.SAK.mod360(self.compass.vesselBearing), x, y)]
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
        if not self.paused and not self.stopped:
            self.refresh()
            #Generates a trail behind the boat
            self.c.create_line(self.x_prev, self.y_prev, x, y, fill='black', width=1, tags='boat_path')
            self.x_prev, self.y_prev = x, y
            #scroll functionality
            x0 = self.hbar.get()[0] * MAXW
            y0 = self.vbar.get()[0] * MAXH
            if x < x0 or x > x0 + 100*(SCALE/3) or y < y0 or y > y0 + 100*(SCALE/3):
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
        self.Dheading.insert(END, self.desHeading)
        self.LocationLat.delete('1.0', END)
        self.LocationLat.insert(END, self.gpsLat)
        self.LocationLon.delete('1.0', END)
        self.LocationLon.insert(END, self.gpsLng)

    def move_cycle(self):
        if not self.paused:
            self.desHeading = self.CourseBoi.find_course(self.gpsLng, self.gpsLat, self.compass.vesselBearing)
            # print(self.desHeading)
            self.adjustRudder()
            self.move_forward()

        threading.Timer(dt, self.move_cycle).start()

    def move_forward(self):
        # self.compass.vesselBearing = self.CourseBoi.set_bearing(WIND_KNOTTS, self.compass.windRelNorth,self.gpsLng, self.gpsLat,self.compass.vesselBearing)
        self.CourseBoi.check_dist_to_target(self.gpsLat,self.gpsLng, WAYPOINT1)

        if(abs(self.gpsLat - WAYPOINT1[0]) < 0.00003 and abs(self.gpsLng - WAYPOINT1[1]) < 0.00003):
            print("satisified positional reqs")
            self.stopped = not self.stopped
            self.finishedLab.delete('1.0', END)
            self.finishedLab.insert(END, "Waypoint Reached, Simulation complete")
        if(not self.stopped):
            self.angleOfSail = self.SAK.mod360(self.compass.windRelNorth - self.compass.vesselBearing)
            boatSpeed = abs(self.speed*sin(self.SAK.deg2rad(self.angleOfSail)))
            if self.angleOfSail < 225 and self.angleOfSail > 135:
                boatSpeed = boatSpeed

            dist = boatSpeed * dt
            lat1 = self.SAK.deg2rad(self.gpsLat)
            lng1 = self.SAK.deg2rad(self.gpsLng)
            bearing = self.SAK.deg2rad(self.compass.vesselBearing)
            lat2 = asin(sin(lat1)*cos(dist) + cos(lat1)*sin(dist)*cos(bearing))
            lng2 = lng1 + atan2(sin(bearing)*sin(dist)*cos(lat1), cos(dist) - sin(lat1)*sin(lat2))

            self.gpsLat, self.gpsLng = self.SAK.rad2deg(lat2), self.SAK.rad2deg(lng2)
            self.compass.vesselBearing += -RUDDER_RESPONSE*self.rudder*self.speed*dt
            self.compass.vesselBearing = self.SAK.mod360(self.compass.vesselBearing)


    def gps_create_line(self, x0, y0, x1, y1, fill, width, tags):
        self.c.create_line(*self.LatLontoXY(x0, y0), *self.LatLontoXY(x1, y1), fill=fill, width=width, tags=tags)

    def drawDot(self, x1, y1, r, color, tag):
        points = [x1, y1, x1, y1]
        self.c.create_oval(points, fill=color, width=0, tags=tag)

    def gps_drawCircle(self, lat, lng, r, color, tag):
        phi = self.SAK.rad2deg(r/6371000.0)
        points = [*self.LatLontoXY(lat - phi, lng - phi),
                  *self.LatLontoXY(lat + phi, lng + phi)]
        self.c.create_oval(points, outline=color, width=1, tags=tag)
"""
    Main loop for GUI, keeps it all spinning along
"""
root = Tk()
root.wm_state('zoomed')
app = LynxLakeSimulation(master = root)
app.render()
app.drawDot(*app.LatLontoXY(*WAYPOINT1), DOT_RADIUS, 'red', 'path')
RADIUS = 7.0
app.gps_drawCircle(*WAYPOINT1, RADIUS, 'red', 'path')
app.x_prev, app.y_prev = app.LatLontoXY(app.gpsLat, app.gpsLng)
app.move_cycle()
app.refreshCycle()
app.mainloop()
