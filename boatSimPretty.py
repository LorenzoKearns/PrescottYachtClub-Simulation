#*******************************************************************************************************************#
#  Project: Prescott Yacht Club - Autonomous Sailing Project
#  File: boatSim.py
#  Author: Lorenzo Kearns
#
#  Purpose: simulation environment for running boat code
#*******************************************************************************************************************#
#
#
"""
    Bug log: a useful section for known bugs and what causes them
    Infinte spin on jibe: This is a very common bug. There are two causes of this bug I have encountered so far.
        1. There is an permanent offset(whose cause in some cases is currently unknown) between the desired heading and current headingChange
            Current solution: When the case is only turning when in danger and only once, not two left turns, the current solution here is to increase the tolerance of the heading
            to match the offset, in the cases I encountered this was about 2.9 so i ser tolerance to 3
        2. In the case of final adjustment the angle becomes offset highe than 3, I believe the cause of this bug to be requirement of two left hand or two right hand jibes in a row
"""
#
"""
Notable Edge cases:
 Wind direction 143, makes it close to target but spins infinitely a but from the target, 4 degree error in position versus desired target is cause
 Wind direction 199, gets even closer than 143, but again spins infinitely due to 4 degree error, future edge cases will probably follow a similar trend with this same issue
Likely cause and potential fix:
 It is most likely that this issue comes from the simplicity of the adjust_path() function in Cartographer. The descision of whether to beat right or left is determined
 As a literal back and forth, i.e. currently if the boat is going right it will always set beat to be the opposite, left. This makes cases where two left beats are required impossible.
 The initial implication of this is that a turn to the right and undershoot requires another right hand jibe. This breaks the system as it tries to achieve a left hand jibe using right hand jibe inputs
"""
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
"""
Variable Constants:

These constants are designed to be variable between full runs of the program, i.e. they are cosntant
for the duration of the program, but can be changed to test different conditions during new runs.
    STARTING_POS - Give a position where the boat will start, in the form of Lattitude, Longitude
    WAYPOINT1 - Give a position where the boat will sail to, in the form of Lattitude, Longitude. Currently simulation only accepts 1 waypoint so this is also the end waypoint
    WIND_SPEED_KNOTTS - Give the wind speed, in knotts, that you wish to test for the simulation. Polar chart only accepts the values: 6, 8, 10, 12, 14, 16, 20
    SCALE - This is the desired zoom factor for the simulation, a value of 1 is the default zoom. this shows the entire map with a barely visible boat. 30 is the highest zoom reccomended.
        but 20 is ideal for still seeing items in relation to the shoreline. anything higher than 30 will not reall show anything. any values inbetween 1-20 are great options.
"""
STARTING_POS =   34.518501, -112.3855601
WAYPOINT1 =   34.521701, -112.3858601
WIND_SPEED_KNOTTS = 20
SCALE = 5
#
"""
 Strict Constants:

 These Constnats are fully defined and should not be changed on a whim like the above constants.
 Many of these constants are scaled and change based on the above parameters but the base structure
 has been worked out through heavy analysis and testing and should only be changed as additonal analysis requires.
"""
W = 1051.05971816
H = 1200.0
MAXW = SCALE*W
MAXH = SCALE*H
LATMIN = 34.5150517
LONMIN = -112.388989
SCALE_LON = 0.00698
SCALE_LAT = 0.0079691
"""
Boat size parameters
"""
BOAT_LENGTH = ((5.6667*1.0)/2915) * MAXH
BOAT_BOW = ((5.6667*0.3)/2915) * MAXH
BOAT_WIDTH = (1.0/2018) * MAXW
RUDDER_LENGTH = (1.2/2915) * MAXH
SAIL_LENGTH = (1.8/2018) * MAXW
DOT_RADIUS = (0.2/2018) * MAXW
WIND_COMPASS = (2.8/2915) * MAXH
HEADING_ARROW = (2.8/2915) * MAXH * 10/SCALE
# rudder movement
RUDDER_RESPONSE = 5000000.005
RUDDER_MIN_ANGLE = -30
RUDDER_MAX_ANGLE = 30
SAIL_MIN = -90
SAIL_MAX = 90
# time refresh
dt = 0.0001
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
        self.boatAngles, self.boatSpeeds = self.CourseBoi.define_speed(WIND_SPEED_KNOTTS)
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
        self.BoatSpeedLabel = Text(frame, height=1, width=18)
        self.BoatSpeedLabel.pack(side = LEFT)
        self.BoatSpeedLabel.insert(END, "Boat Speed(Knots):")
        self.BoatSpeed = Text(frame, height=1, width=8)
        self.BoatSpeed.pack(side = LEFT)
        self.finishedLab = Text(frame, height=1, width=40)
        self.finishedLab.pack(side = LEFT)
        self.finishedLab.delete('1.0', END)


        # self.finishedLab.insert(END, "Waypoint Reached, Simulation complete")
        self.c = Canvas(root, width=W, height=H, bg="#036303", scrollregion=(0, 0, MAXW, MAXH))

    def init_sim_conditions(self):
        """
            Defines the initial conditions that run the SIM
            sets up wind and initial bearing, stuff like that
        """
        trueHeading = 0
        x = random.randint(360)
        trueWind = self.SAK.mod360(x)
        trueWind = 180
        self.compass = Compass(trueHeading, trueWind)
        self.gpsLat, self.gpsLng = STARTING_POS
        self.gpsLatPrev, self.gpsLngPrev = self.gpsLat, self.gpsLng
        self.angleOfSail = 0.0
        # self.speed = WIND_SPEED_KNOTTS/1000000
        self.rudder = 0.0
        self.sailPos = 0.0
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
        self.c.create_polygon(self.bound, fill = '#006994', outline = '#036303', width = 2, tags = 'boundary')

    def pause(self):
        """
            Simple function to pause/unpause the simulation
        """
        self.paused = not self.paused

    def adjustRudder(self):
        """
            Determine where to set the rudder based on desired heading
        """
        if (abs(self.compass.vesselBearing - self.desHeading) == 0):
            self.rudder = 0
        elif (self.compass.vesselBearing < self.desHeading):
            self.rudder = RUDDER_MAX_ANGLE
        elif (self.compass.vesselBearing > self.desHeading):
            self.rudder = RUDDER_MIN_ANGLE

    def adjust_sail(self):
        self.apparentWind = self.SAK.mod360(self.compass.windRelNorth - self.compass.vesselBearing)
        # print(self.apparentWind)
        if(not(self.apparentWind > SAIL_MAX and self.apparentWind < self.SAK.mod360(SAIL_MIN))):
            if(self.apparentWind > 180):
                self.sailPos = self.SAK.mod360(self.compass.windRelNorth - 90)
            else:
                self.sailPos = self.SAK.mod360(self.compass.windRelNorth + 90)
            # print(self.sailPos)
        else:
            self.sailPos = self.SAK.mod360(self.compass.vesselBearing - 180)

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
        self.adjust_sail()
        points = [x, y, *self.SAK.rotate(x, y - SAIL_LENGTH, self.sailPos, x, y)]
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
            self.c.create_line(self.x_prev, self.y_prev, x, y, fill='black', width=2, tags='boat_path')
            self.x_prev, self.y_prev = x, y
            #scroll functionality
            x0 = self.hbar.get()[0] * MAXW
            y0 = self.vbar.get()[0] * MAXH
            if x < x0 or x > x0 + 100*(SCALE/2) or y < y0 or y > y0 + 100*(SCALE/2):
                self.c.xview_moveto((x - W/2)/MAXW)
                self.c.yview_moveto((y - H/2)/MAXH)
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
        self.LocationLat.delete('1.0', END)
        self.LocationLat.insert(END, self.gpsLat)
        self.LocationLon.delete('1.0', END)
        self.LocationLon.insert(END, self.gpsLng)
        self.BoatSpeed.delete('1.0', END)
        self.BoatSpeed.insert(END, self.boatSpeed * 364000 / 1.6781)

    def move_cycle(self):
        """
            Threaded loop for moving the position of the boat and rudder.
            Also calls CoursePlotter class to determine the course response of the boat
        """
        if not self.paused:
            self.desHeading = self.CourseBoi.find_course(self.gpsLng, self.gpsLat, self.compass.vesselBearing)
            # print(self.desHeading)
            self.adjustRudder()
            self.move_forward()

        threading.Timer(dt, self.move_cycle).start()

    def move_forward(self):
        """
            Handles the operation of moving the boat forward, does this by calculating the boats speed as a relation to the apparent wind angle.
            This speed is used in concurrance with the time step constant dt to determine how far the boat object should be translated.
            The final result is the boat position creeping forward tiny steps at a time which when run, creates a smooth animation of movement.
        """
        # self.compass.vesselBearing = self.CourseBoi.set_bearing(WIND_KNOTTS, self.compass.windRelNorth,self.gpsLng, self.gpsLat,self.compass.vesselBearing)
        self.CourseBoi.check_dist_to_target(self.gpsLat,self.gpsLng, WAYPOINT1)

        if(abs(self.gpsLat - WAYPOINT1[0]) < 0.00003 and abs(self.gpsLng - WAYPOINT1[1]) < 0.00003):
            self.stopped = not self.stopped
            self.finishedLab.delete('1.0', END)
            self.finishedLab.insert(END, "Waypoint Reached, Simulation complete")
        if(not self.stopped):
            self.angleOfSail = self.SAK.mod360(self.compass.windRelNorth - self.compass.vesselBearing)
            rel_angle_pos = self.find_nearest(self.boatAngles, self.angleOfSail)
            # print(self.boatAngles[rel_angle_pos])
            self.boatSpeed = (self.boatSpeeds[rel_angle_pos] * 1.6781)/364000
            dist = self.boatSpeed * dt

            lat1 = self.SAK.deg2rad(self.gpsLat)
            lng1 = self.SAK.deg2rad(self.gpsLng)
            bearing = self.SAK.deg2rad(self.compass.vesselBearing)
            # find the next point for the boat to go based on the bearing
            lat2 = asin(sin(lat1)*cos(dist) + cos(lat1)*sin(dist)*cos(bearing))
            lng2 = lng1 + atan2(sin(bearing)*sin(dist)*cos(lat1), cos(dist) - sin(lat1)*sin(lat2))

            # Set the boats current lat, lon position to the new lat2, lng2 determined above
            self.gpsLat, self.gpsLng = self.SAK.rad2deg(lat2), self.SAK.rad2deg(lng2)
            # Adjust the current compas bearing of the boat based on the rudder position and response
            self.compass.vesselBearing += -RUDDER_RESPONSE*self.rudder*self.boatSpeed*dt
            self.compass.vesselBearing = self.SAK.mod360(self.compass.vesselBearing)

    def drawDot(self, x1, y1, r, color, tag):
        """
            Place a dot on the map for the waypoint
        """
        points = [x1, y1, x1, y1]
        self.c.create_oval(points, fill=color, width=0, tags=tag)

    def gps_drawCircle(self, lat, lng, r, color, tag):
        """
            Draw a bigger, non-filled circle around the dot that was placed
            to better show the range that our waypoint is satisfied
        """
        phi = self.SAK.rad2deg(r/6371000.0)
        points = [*self.LatLontoXY(lat - phi, lng - phi),
                  *self.LatLontoXY(lat + phi, lng + phi)]
        self.c.create_oval(points, outline=color, width=1, tags=tag)

    def find_nearest(self, array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return idx
        # return array[idx]
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
