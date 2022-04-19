import tkinter as tk
from tkinter import *
from polarChart import Sailing
from polarChart import Polar
import pandas as pd
import numpy as np
from numpy import random
import threading
from shapely.geometry import Point, LineString, Polygon
from math import *
import time
from navHeading import Helmsman

SCALE = 2
WIND_KNOTTS = 20
WAYPOINT0 =   34.5161, -112.3849
WAYPOINT1 =   34.5217, -112.38586
MAXDEV = 500 # how far away can the boat go from the ideal path

TACKMODE_DIRECTLY = 0
TACKMODE_ADJ_POS = 1
TACKMODE_ADJ_NEG = 2
TACKMODE_MAXDEV_POS = 3
TACKMODE_MAXDEV_NEG = 4

W = 1051.05971816
H = 1200.0
MAXW = SCALE*W
MAXH = SCALE*H
LATMIN = 34.5150517
LONMIN = -112.388989
SCALE_LON = 0.00698
SCALE_LAT = 0.0079691
MAX_SPEED = ((WIND_KNOTTS * 1.6878111113736) / ((2915 + 2018)/2)) / 60 / 60

BOAT_LENGTH = ((5.6667*1.0)/2915) * MAXH
BOAT_BOW = ((5.6667*0.3)/2915) * MAXH
BOAT_WIDTH = (1.0/2018) * MAXW
RUDDER_LENGTH = (1.2/2915) * MAXH
SAIL_LENGTH = (1.8/2018) * MAXW
DOT_RADIUS = (0.2/2018) * MAXW
WIND_COMPASS = (2.8/2915) * MAXH
HEADING_ARROW = (2.8/2915) * MAXH

FLAP_NORMAL = 10.0
FLAP_MAX = 15.0
TACK_SAIL_CRITICAL_ANGLE = FLAP_MAX/2
FLAP_ITERATION = 0.1

RUDDER_RESPONSE = 10000000.005
RUDDER_COEFF = 40.0
RUDDER_MIN_ANGLE = -30
RUDDER_MAX_ANGLE = 35

SAIL_ANGLE_MAX = 90.0
SAIL_ANGLE_MIN = -90.0

MAXDEV_OK_FACTOR = 0.75
MAXDEV_OK = MAXDEV * MAXDEV_OK_FACTOR

dt = 0.001
dt_refresh = 4*dt
mux = -1
isBehindPath = 0
magDec = 0.0

class LatLon():
    def __init__(self, lattitude, longitude):
        self.lat = lattitude
        self.lon = longitude

class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self,master)
        boat = Polar()
        self.p_chart = boat.plot_polar()
        self.sailor = Sailing(self.p_chart)
        self.scaleValue = 10
        self.target = LatLon(34.5217, -112.38586)
        self.Helm = Helmsman(self.p_chart, WAYPOINT0, WAYPOINT1)
        self.timer1 = time.perf_counter()
        frame = Frame(root)
        frame.pack()
        button_pause = Button(frame, text="Pause/Resume", command=self.pauseResume)
        button_pause.pack(side = LEFT)
        self.Twind = Text(frame, height=1, width=10)
        self.Twind.pack(side = LEFT)
        self.Theading = Text(frame, height=1, width=10)
        self.Theading.pack(side = LEFT)
        self.Dheading = Text(frame, height=1, width=10)
        self.Dheading.pack(side = LEFT)
        self.LocationLat = Text(frame, height=1, width=10)
        self.LocationLat.pack(side = LEFT)
        self.LocationLon = Text(frame, height=1, width=10)
        self.LocationLon.pack(side = LEFT)
        self.c = Canvas(root, width=W, height=H, bg="white", scrollregion=(0, 0, MAXW, MAXH))
        self.init_buttons()
        self.create_polygon()
        self.plot_boundaries()
        self.desHeading = self.Helm.find_heading(WIND_KNOTTS, self.mod360(trueWind))

    def init_buttons(self):
        global trueWind
        x = random.randint(360)
        trueWind = self.mod360(x)
        # trueWind = 0
        self.hbar=Scrollbar(root,orient=HORIZONTAL)
        self.hbar.pack(side=BOTTOM,fill=X)
        self.hbar.config(command=self.c.xview)

        self.vbar=Scrollbar(root,orient=VERTICAL)
        self.vbar.pack(side=RIGHT,fill=Y)
        self.vbar.config(command=self.c.yview)
        self.c.config(xscrollcommand=self.hbar.set, yscrollcommand=self.vbar.set)
        self.c.pack(side=LEFT, expand=True, fill=BOTH)

    def create_polygon(self):
            self.bound = []
            temp_container = pd.read_csv('newOutputBoundaries2.csv')
            temp_container = np.array(temp_container)
            boundaryArrayLat = temp_container[:,1]
            boundaryArrayLon = temp_container[:,2]
            self.boundaryArrayLon = np.array(boundaryArrayLon)
            self.boundaryArrayLat = np.array(boundaryArrayLat)
            coordTuple = np.array((self.boundaryArrayLon,self.boundaryArrayLat)).T
            self.poly = Polygon(coordTuple)
            # self.boat_poly = Polygon(boat_dimensions)
            for i in range(len(self.boundaryArrayLat)):
                lon,lat = self.LatLontoXY(self.boundaryArrayLat[i], self.boundaryArrayLon[i])
                self.bound.append(lon)
                self.bound.append(lat)
            # print(self.bound)

    def plot_boundaries(self):
        self.c.create_polygon(self.bound, fill = 'white', outline = 'green', width = 2, tags = 'boundary') #, outline = 'blue', width = 5

    def LatLontoXY(self,lat, lon):
        x_adj = abs(((lon - LONMIN)/SCALE_LON) * MAXW)
        y_adj = abs(((lat - LATMIN)/SCALE_LAT) * MAXH)
        return x_adj, y_adj

    def gps_create_line(self, x0, y0, x1, y1, fill, width, tags):
        self.c.create_line(*self.LatLontoXY(x0, y0), *self.LatLontoXY(x1, y1), fill=fill, width=width, tags=tags)

    def rot(self, x1, y1, a1, x0, y0):
        b = self.deg2rad(a1)
        return x0 + (x1 - x0)*cos(b) - (y1 - y0)*sin(b), \
                y0 + (y1 - y0)*cos(b) + (x1 - x0)*sin(b)

    def changeWind(self):
        global trueWind
        if(time.perf_counter() - self.timer1 > 480):
            x = random.randint(360)
            trueWind = self.mod360(180 - x)
            self.timer1 = time.perf_counter()
            self.refresh()

    def drawBoat(self):
        global gps_lat, gps_lng, \
        c_boat, c_sail, c_rudder, trueHeading, sail_angle

        x, y = self.LatLontoXY(gps_lat, gps_lng)
        # print(self.LatLontoXY(gps_lat, gps_lng))
        #boat
        points = [self.rot(x, y - BOAT_LENGTH/2, self.mod360(360-trueHeading), x, y),
                  self.rot(x - BOAT_WIDTH/8, y - BOAT_LENGTH/2.1, self.mod360(360-trueHeading), x, y),
                  self.rot(x - BOAT_WIDTH/4, y - BOAT_LENGTH/3.42, self.mod360(360-trueHeading), x, y),
                  self.rot(x - BOAT_WIDTH/2.5, y, self.mod360(360-trueHeading), x, y),
                  self.rot(x - BOAT_WIDTH/2.9, y + BOAT_LENGTH/4.60  , self.mod360(360-trueHeading), x, y),
                  self.rot(x, y + BOAT_LENGTH/2, self.mod360(360-trueHeading), x, y),
                  self.rot(x + BOAT_WIDTH/2.9, y + BOAT_LENGTH/4.60  , self.mod360(360-trueHeading), x, y),
                  self.rot(x + BOAT_WIDTH/2.5, y, self.mod360(360-trueHeading), x, y),
                  self.rot(x + BOAT_WIDTH/4, y - BOAT_LENGTH/3.42, self.mod360(360-trueHeading), x, y),
                  self.rot(x + BOAT_WIDTH/8, y - BOAT_LENGTH/2.1, self.mod360(360-trueHeading), x, y),
                  self.rot(x, y - BOAT_LENGTH/2, self.mod360(360-trueHeading), x, y)]

        # [self.rot(x - BOAT_WIDTH/2, y + BOAT_LENGTH/2, self.mod360(360-trueHeading), x, y),
        #           self.rot(x - BOAT_WIDTH/2, y - BOAT_LENGTH/2 + BOAT_BOW, self.mod360(360-trueHeading), x, y),
        #           self.rot(x, y - BOAT_LENGTH/2, self.mod360(360-trueHeading), x, y),
        #           self.rot(x + BOAT_WIDTH/2, y - BOAT_LENGTH/2 + BOAT_BOW, self.mod360(360-trueHeading), x, y),
        #           self.rot(x + BOAT_WIDTH/2, y + BOAT_LENGTH/2, self.mod360(360-trueHeading), x, y)]

        #if not c_boat is None:
        self.c.delete('boat')

        c_boat = self.c.create_polygon(points, fill='blue', width=0, tags='boat')

        #rudder
        points = [*self.rot(*self.rot(x, y + BOAT_LENGTH/2, rudder, x, y + BOAT_LENGTH/2), self.mod360(360-trueHeading), x, y),
                    *self.rot(*self.rot(x, y + BOAT_LENGTH/2 + RUDDER_LENGTH, rudder, x, y + BOAT_LENGTH/2), self.mod360(360-trueHeading), x, y)]
        c_rudder = self.updateLine(c_rudder, *points, fill='black', width=2)

        #sail
        points = [x, y, *self.rot(x, y - SAIL_LENGTH, sail_angle, x, y)]
        c_sail = self.updateLine(c_sail, *points, fill='black', width=2)

    def refresh(self):
        global trueWind, trueHeading, sail_angle, gps_lat, gps_lng
        self.drawBoat()
        self.Twind.delete('1.0', END)
        self.Twind.insert(END, trueWind)
        self.Theading.delete('1.0', END)
        self.Theading.insert(END, trueHeading)
        self.Dheading.delete('1.0', END)
        self.Dheading.insert(END, self.desHeading)
        self.LocationLat.delete('1.0', END)
        self.LocationLat.insert(END, gps_lat)
        self.LocationLon.delete('1.0', END)
        self.LocationLon.insert(END, gps_lng)

    def rad2deg(self, x):
        return float(x) / pi * 180

    def deg2rad(self, x):
        return float(x) / 180 * pi

    def moveCycle(self):
        global trueWind
        if not paused:
            self.changeWind()
            self.move()
            self.adjustRudder()

        threading.Timer(dt, self.moveCycle).start()

    def adjustRudder(self):
        global paused, closestPoint, ghostPoint, gps_lat, gps_lng, \
        trueHeading, ghostHeading, goHeading, error, rudder, speed, trueWind, adjAngle1, adjAngle2, \
        tackmode, ghostHeading_initialized, \
        sail_angle

        if (abs(trueHeading - self.desHeading) < 0.5): #not reversed
            rudder = 0
        elif (trueHeading < self.desHeading):
            rudder = RUDDER_MAX_ANGLE
        elif (trueHeading > self.desHeading):
            rudder = RUDDER_MIN_ANGLE

    def orientedDiff(self, angle_from, angle_to):
        return self.mod360(angle_to - angle_from)

    def isBetweenOrientedAngles(self, angle, angle_from, angle_to):
        return self.orientedDiff(angle_from, angle) < self.orientedDiff(angle_from, angle_to)

    def isTurnAgainstWind(self, windAngle, angle_from, angle_to):
        return self.isBetweenOrientedAngles(self.mod360(windAngle + 180), angle_from, angle_to)

    def refreshCycle(self):
        global x_prev, y_prev, paused, ghostHeading, error, goHeading, trueWind, trueHeading, adjAngle1, adjAngle2, \
        c_compass1, c_compass2, c_compass3, c_compass4, c_closestPoint, c_ghostPoint, c_ghostHeading, c_windDir, c_goHeading, c_adjHeading1, c_adjHeading2

        x, y = self.LatLontoXY(gps_lat, gps_lng)

        if not paused:

            self.refresh()

            #trail path
            self.c.create_line(x_prev, y_prev, x, y, fill='black', width=1, tags='boat_path')
            x_prev, y_prev = x, y

            #scroll
            x0 = self.hbar.get()[0] * MAXW
            y0 = self.vbar.get()[0] * MAXH
            if x < x0 or x > x0 +100 or y < y0 or y > y0 +100 :
                self.c.xview_moveto((x - W/2)/MAXW)
                self.c.yview_moveto((y - H/6)/MAXH)
                root.update()

            #wind direction
            c_windDir = self.updateLine(c_windDir, x - HEADING_ARROW/3*sin(self.deg2rad(trueWind)), y + HEADING_ARROW/3*cos(self.deg2rad(trueWind)),
                          x + HEADING_ARROW*sin(self.deg2rad(trueWind)), y - HEADING_ARROW*cos(self.deg2rad(trueWind)), fill='gray', width=1)


        threading.Timer(dt_refresh, self.refreshCycle).start()

    def calcSpeed(self):
        global trueHeading, trueWind, sail_angle
        sail_angle = self.mod360(trueWind - trueHeading)
        ret = abs(MAX_SPEED*sin(self.deg2rad(sail_angle)))
        if sail_angle < 180 or sail_angle > 180:
            ret = -ret
        return ret

    def move(self):
        global speed, dt, gps_lat, gps_lng, trueHeading, rudder

        speed = self.calcSpeed()

        gps_lat, gps_lng = self.Destination(gps_lat, gps_lng, trueHeading, speed*dt)
        trueHeading += -RUDDER_RESPONSE*rudder*speed*dt
        trueHeading = self.mod360(trueHeading)

    def mod360(self,a):
        while a > 360:
            a -= 360
        while a < 0:
            a += 360
        return a

    def sq(self, x):
        return x*x

    def limits(self, x, x_min, x_max):
        if x < x_min:
            return x_min;
        elif x > x_max:
            return x_max;
        return x;

    def Destination(self, lat1, lng1, bearing, d):
        lat1 = self.deg2rad(lat1)
        lng1 = self.deg2rad(lng1)
        bearing = self.deg2rad(bearing)
        lat2 = asin(sin(lat1)*cos(d) + cos(lat1)*sin(d)*cos(bearing))
        lng2 = lng1 + atan2(sin(bearing)*sin(d)*cos(lat1), cos(d) - sin(lat1)*sin(lat2))
        return self.rad2deg(lat2), self.rad2deg(lng2)

    def pauseResume(self):
        global paused
        paused = not paused

    def drawDot(self, x1, y1, r, color, tag):
        points = [x1 - r, y1 - r, x1 + r, y1 + r]
        self.c.create_oval(points, fill=color, width=0, tags=tag)

    def gps_drawCircle(self, lat, lng, r, color, tag):
        phi = self.rad2deg(r/6371000.0)
        points = [*self.LatLontoXY(lat - phi, lng - phi),
                  *self.LatLontoXY(lat + phi, lng + phi)]
        self.c.create_oval(points, outline=color, width=1, tags=tag)

    def updateDot(self, c_var, x1, y1, r, color):
        points = [x1 - r, y1 - r, x1 + r, y1 + r]
        if c_var is None:
            c_var = self.c.create_oval(points, fill=color, width=0)
        else:
            self.c.coords(c_var, points)
        return c_var

    def trueBearing(self, lat1, lng1, lat2, lng2):
        lat1 = self.deg2rad(lat1)
        lng1 = self.deg2rad(lng1)
        lat2 = self.deg2rad(lat2)
        lng2 = self.deg2rad(lng2)
        angle = atan2(sin(lng2 - lng1) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lng2 - lng1))
        return self.rad2deg(angle)

    def updateLine(self, c_var, x0, y0, x1, y1, fill, width):
        if c_var is None:
            c_var = self.c.create_line(x0, y0, x1, y1, fill=fill, width=width)
        else:
            self.c.coords(c_var, x0, y0, x1, y1)
        return c_var

global x_prev, y_prev, gps_lat, gps_lng, trueHeading, tackmode, trueWind, ghostPoint, closestPoint, \
        ghostHeading, ghostHeading_instant, goHeading, error, sail_angle, crossTrack, pathAngle, rudder, \
        speed, paused, c_closestPoint, c_ghostPoint, c_ghostHeading, c_windDir, flap, flap_final,\
        c_goHeading, c_adjHeading1, c_adjHeading2, c_boat, c_sail, c_rudder, c_compass1, \
        c_compass2, c_compass3, c_compass4, tackmode_str, ghostHeading_initialized
gps_lat, gps_lng = WAYPOINT0
gps_lat_prev, gps_lng_prev = gps_lat, gps_lng
x_prev = 0.0
y_prev = 0.0
trueHeading = 180
tackmode = TACKMODE_DIRECTLY
trueWind = 0.0
ghostPoint = 0.0, 0.0
closestPoint = 0.0, 0.0
ghostHeading = 0.0
ghostHeading_instant = 0.0
goHeading = 0.0
error = 0.0
flap = FLAP_NORMAL
flap_final = flap
sail_angle = 0.0
crossTrack = 0.0
pathAngle = 0.0
rudder = 0.0
speed = 0.0
paused = False
c_boat, c_sail, c_rudder = None, None, None
c_compass1, c_compass2, c_compass3, c_compass4, \
c_closestPoint, c_ghostPoint, c_ghostHeading, c_windDir, c_goHeading, c_adjHeading1, c_adjHeading2 \
    = None, None, None, None, None, None, None, None, None, None, None
tackmode_str = ['TACKMODE_DIRECTLY', 'TACKMODE_ADJ_POS', 'TACKMODE_ADJ_NEG', 'TACKMODE_MAXDEV_POS', 'TACKMODE_MAXDEV_NEG']
ghostHeading_initialized = False
root = Tk()
root.wm_state('zoomed')
app = Application(master = root)
app.drawDot(*app.LatLontoXY(*WAYPOINT0), DOT_RADIUS, 'red', 'path')
app.drawDot(*app.LatLontoXY(*WAYPOINT1), DOT_RADIUS, 'red', 'path')
RADIUS = 7.0
app.gps_drawCircle(*WAYPOINT1, RADIUS, 'red', 'path')
app.drawBoat()
x_prev, y_prev = app.LatLontoXY(gps_lat, gps_lng)
app.moveCycle()
app.refreshCycle()
app.mainloop()
