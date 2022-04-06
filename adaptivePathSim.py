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

TACKMODE_DIRECTLY = 0
TACKMODE_ADJ_POS = 1
TACKMODE_ADJ_NEG = 2
TACKMODE_MAXDEV_POS = 3
TACKMODE_MAXDEV_NEG = 4

W = 1051.05971816
H = 1200.0
MAXW = 20*W
MAXH = 20*H
LATMIN = 34.5150517
LONMIN = -112.388989
SCALE_LON = 0.00698
SCALE_LAT = 0.0079691

SIZEADJUST = 1

MAX_SPEED = 200.0

BOAT_LENGTH = 50/SIZEADJUST
BOAT_BOW = 20/SIZEADJUST
BOAT_WIDTH = 10/SIZEADJUST
RUDDER_LENGTH = 20/SIZEADJUST
SAIL_LENGTH = 40/SIZEADJUST
DOT_RADIUS = 2/SIZEADJUST
WIND_COMPASS = 100/SIZEADJUST
HEADING_ARROW = 100/SIZEADJUST

WAYPOINT0 =   34.5161, -112.3849
WAYPOINT1 =   34.5217, -112.38586

HEADING_OK_LIMIT = 10.0
RUDDER_RESPONSE = 0.005
RUDDER_COEFF = 40.0
RUDDER_MIN_ANGLE = -30
RUDDER_MAX_ANGLE = 35

SAIL_ANGLE_MAX = 90.0
SAIL_ANGLE_MIN = -90.0

R_MEAN = 6371000.0

MAXDEV = 500
MAXDEV_OK_FACTOR = 0.75
MAXDEV_OK = MAXDEV * MAXDEV_OK_FACTOR

GHOST_DISTANCE = 20000.0

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
        self.target = LatLon(34.5217, -112.38586)
        self.timer1 = time.perf_counter()
        frame = Frame(root)
        frame.pack()
        button_pause = Button(frame, text="Pause/Resume", command=self.pauseResume)
        button_pause.pack(side = LEFT)
        self.Twind = Text(frame, height=1, width=10)
        self.Twind.pack(side = LEFT)
        self.Theading = Text(frame, height=1, width=10)
        self.Theading.pack(side = LEFT)
        self.c = Canvas(root, width=W, height=H, bg="white", scrollregion=(0, 0, MAXW, MAXH))
        self.init_buttons()
        self.create_polygon()
        self.plot_boundaries()

    def init_buttons(self):
        global trueWind
        x = random.randint(360)
        trueWind = self.mod360(180 - x)
        self.hbar=Scrollbar(root,orient=HORIZONTAL)
        self.hbar.pack(side=BOTTOM,fill=X)
        self.hbar.config(command=self.c.xview)

        self.vbar=Scrollbar(root,orient=VERTICAL)
        self.vbar.pack(side=RIGHT,fill=Y)
        self.vbar.config(command=self.c.yview)
        self.c.config(xscrollcommand=self.hbar.set, yscrollcommand=self.vbar.set)
        self.c.pack(side=LEFT, expand=True, fill=BOTH)

        # self.c.bind("<Button-1>", self.changeWind)
        # self.c.bind("<B1-Motion>", self.changeWind)

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
        self.c.create_polygon(self.bound, fill = 'white', outline = 'green', width = 2) #, outline = 'blue', width = 5

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
        if(time.perf_counter() - self.timer1 > 30):
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
        points = [self.rot(x - BOAT_WIDTH/2, y + BOAT_LENGTH/2, trueHeading, x, y),
                  self.rot(x - BOAT_WIDTH/2, y - BOAT_LENGTH/2 + BOAT_BOW, trueHeading, x, y),
                  self.rot(x, y - BOAT_LENGTH/2, trueHeading, x, y),
                  self.rot(x + BOAT_WIDTH/2, y - BOAT_LENGTH/2 + BOAT_BOW, trueHeading, x, y),
                  self.rot(x + BOAT_WIDTH/2, y + BOAT_LENGTH/2, trueHeading, x, y)]

        #if not c_boat is None:
        self.c.delete('boat')

        c_boat = self.c.create_polygon(points, fill='blue', width=0, tags='boat')

        #rudder
        points = [*self.rot(*self.rot(x, y + BOAT_LENGTH/2, rudder, x, y + BOAT_LENGTH/2), trueHeading, x, y),
                    *self.rot(*self.rot(x, y + BOAT_LENGTH/2 + RUDDER_LENGTH, rudder, x, y + BOAT_LENGTH/2), trueHeading, x, y)]
        c_rudder = self.updateLine(c_rudder, *points, fill='black', width=2)

        #sail
        points = [x, y, *self.rot(x, y - SAIL_LENGTH, sail_angle, x, y)]
        c_sail = self.updateLine(c_sail, *points, fill='black', width=2)

    def refresh(self):
        global trueWind, trueHeading, tackmode, tackmode_str
        self.drawBoat()
        self.Twind.delete('1.0', END)
        self.Twind.insert(END, trueWind)
        self.Theading.delete('1.0', END)
        self.Theading.insert(END, trueHeading)

    def rad2deg(self, x):
        return float(x) / pi * 180

    def deg2rad(self, x):
        return float(x) / 180 * pi

    def moveCycle(self):
        global trueWind
        if not paused:
            self.changeWind()
            self.move()
            self.intensiveTasks()
            self.where2go()
            self.adjustRudder()

        threading.Timer(dt, self.moveCycle).start()

    def intensiveTasks(self):
        global paused, closestPoint, ghostPoint, gps_lat, gps_lng, \
        trueHeading, ghostHeading, goHeading, error, rudder, speed, trueWind, adjAngle1, adjAngle2, \
        tackmode, ghostHeading_initialized, ghostHeading_instant, \
        ghostHeading_instant, crossTrack, pathAngle

        startAngle = self.trueBearing(*WAYPOINT0, *WAYPOINT1)
        pathLength = self.Distance(*WAYPOINT0, *WAYPOINT1)
        d13 = self.Distance(*WAYPOINT0, gps_lat, gps_lng)
        crossTrack = self.crossTrackDistance(*WAYPOINT0, *WAYPOINT1, gps_lat, gps_lng, d13)

        if self.Distance(gps_lat, gps_lng, *WAYPOINT1) > pathLength:
            alongTrack = 0
        else:
            alongTrack = self.alongTrackDistance(*WAYPOINT0, *WAYPOINT1, gps_lat, gps_lng, d13, crossTrack)

        if alongTrack + GHOST_DISTANCE > pathLength:
            ghostPoint = WAYPOINT1
        else:
            ghostPoint = self.Destination(*WAYPOINT0, startAngle, alongTrack + GHOST_DISTANCE)

        closestPoint = self.Destination(*WAYPOINT0, startAngle, alongTrack)
        pathAngle = self.trueBearing(*closestPoint, *WAYPOINT1)
        ghostHeading_instant = self.trueBearing(gps_lat, gps_lng, *ghostPoint)

    def where2go(self):
        global paused, closestPoint, ghostPoint, gps_lat, gps_lng, \
        trueHeading, ghostHeading, goHeading, error, rudder, speed, trueWind, adjAngle1, adjAngle2, \
        tackmode, ghostHeading_initialized, \
        crossTrack, sail_angle
        #update ghostHeading only if heading is OK
        if not ghostHeading_initialized or self.headingOK(error):
            ghostHeading = ghostHeading_instant

        ghostHeading_initialized = True
        #change tackmode, crossTrack is positive on the right from the path, negative on the left
        if crossTrack > MAXDEV:
            tackmode = TACKMODE_MAXDEV_POS
        elif crossTrack < -MAXDEV:
            tackmode = TACKMODE_MAXDEV_NEG

        if tackmode == TACKMODE_MAXDEV_POS and crossTrack < MAXDEV_OK \
        or tackmode == TACKMODE_MAXDEV_NEG and crossTrack > -MAXDEV_OK:
            tackmode = TACKMODE_DIRECTLY
        #if it can't go directly, calculate adjusted angles
        canGoDirectly, adjAngle1, adjAngle2 = self.calcAdjustedAngles(ghostHeading, trueWind)
        if tackmode == TACKMODE_MAXDEV_NEG:
            if canGoDirectly:
                goHeading = ghostHeading
            else:
                if not self.isBetweenOrientedAngles(adjAngle1, pathAngle, pathAngle + 140):
                    goHeading = adjAngle2
                elif not self.isBetweenOrientedAngles(adjAngle2, pathAngle, pathAngle + 140):
                    goHeading = adjAngle1
                else:
                    goHeading, tm_ = self.bestAdjHeading(adjAngle1, adjAngle2, trueHeading, trueWind)
        elif tackmode == TACKMODE_MAXDEV_POS:
            if canGoDirectly:
                goHeading = ghostHeading
            else:
                if not self.isBetweenOrientedAngles(adjAngle1, pathAngle - 140, pathAngle):
                    goHeading = adjAngle2
                elif not self.isBetweenOrientedAngles(adjAngle2, pathAngle - 140, pathAngle):
                    goHeading = adjAngle1
                else:
                    goHeading, tm_ = self.bestAdjHeading(adjAngle1, adjAngle2, trueHeading, trueWind)
        else:
            if canGoDirectly:
                goHeading = ghostHeading
                tackmode = TACKMODE_DIRECTLY
            else:
                goHeading, tackmode = self.bestAdjHeading(adjAngle1, adjAngle2, trueHeading, trueWind)

        sail_angle = self.limits(self.mod360(trueWind - trueHeading), SAIL_ANGLE_MIN, SAIL_ANGLE_MAX)

    def adjustRudder(self):
        global paused, closestPoint, ghostPoint, gps_lat, gps_lng, \
        trueHeading, ghostHeading, goHeading, error, rudder, speed, trueWind, adjAngle1, adjAngle2, \
        tackmode, ghostHeading_initialized, \
        sail_angle

        #calculate error and adjust it based on the wind direction (boat must not turn against the wind)

        angle_positive = self.orientedDiff(trueHeading, goHeading)

        if not self.isTurnAgainstWind(trueWind, trueHeading, goHeading):
            error = angle_positive;
        else:
            error = -(360 - angle_positive);

        #boat is against the wind and is reversed, do a special maneuver to safely switch forward (can't be the opposite way... proven)

        if sail_angle < 180  or sail_angle > 180 : #reversed
            if sail_angle > 180 - 45 and sail_angle < 180: #against the wind
                rudder = RUDDER_MAX_ANGLE #boat "turning left", but right backwards
            elif sail_angle < 180 + 45 and sail_angle > 180:
                rudder = RUDDER_MIN_ANGLE
    #            else:
    #                rudder = 0 #not needed, rather prevent rudder jitter
        else: #not reversed
            rudder = self.limits(round(-RUDDER_COEFF / 100 * error), RUDDER_MIN_ANGLE, RUDDER_MAX_ANGLE)

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

            #goHeading
            c_goHeading = self.updateLine(c_goHeading, x, y, x + HEADING_ARROW*sin(self.deg2rad(goHeading)), y - HEADING_ARROW*cos(self.deg2rad(goHeading)), fill='magenta', width=1)

            #adjustedHeading
            c_adjHeading1 = self.updateLine(c_adjHeading1, x, y, x + HEADING_ARROW/1.5*sin(self.deg2rad(adjAngle1)), y - HEADING_ARROW/1.5*cos(self.deg2rad(adjAngle1)), fill='blue', width=1)
            c_adjHeading2 = self.updateLine(c_adjHeading2, x, y, x + HEADING_ARROW/1.5*sin(self.deg2rad(adjAngle2)), y - HEADING_ARROW/1.5*cos(self.deg2rad(adjAngle2)), fill='blue', width=1)

        threading.Timer(dt_refresh, self.refreshCycle).start()

    def calcSpeed(self):
        global trueHeading, trueWind
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

    def trueBearing(self, lat1, lng1, lat2, lng2):
        lat1 = self.deg2rad(lat1)
        lng1 = self.deg2rad(lng1)
        lat2 = self.deg2rad(lat2)
        lng2 = self.deg2rad(lng2)
        angle = atan2(sin(lng2 - lng1) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lng2 - lng1))
        return self.rad2deg(angle)

    def Destination(self, lat1, lng1, bearing, d):
        lat1 = self.deg2rad(lat1)
        lng1 = self.deg2rad(lng1)
        bearing = self.deg2rad(bearing)
        lat2 = asin(sin(lat1)*cos(d/R_MEAN) + cos(lat1)*sin(d/R_MEAN)*cos(bearing))
        lng2 = lng1 + atan2(sin(bearing)*sin(d/R_MEAN)*cos(lat1), cos(d/R_MEAN) - sin(lat1)*sin(lat2))
        return self.rad2deg(lat2), self.rad2deg(lng2)

    def Distance(self, lat1, lng1, lat2, lng2):
        lat1 = self.deg2rad(lat1)
        lng1 = self.deg2rad(lng1)
        lat2 = self.deg2rad(lat2)
        lng2 = self.deg2rad(lng2)
        hav = self.sq(sin((lat2-lat1)/2)) + cos(lat1) * cos(lat2) * self.sq(sin((lng2-lng1)/2))
        if hav < 0:
            hav = 0 #shouldn't happen
        if hav > 1:
            hav = 1 #shouldn't happen
        return 2 * R_MEAN * atan2(sqrt(hav), sqrt(1-hav))

    def crossTrackDistance(self, lat1, lng1, lat2, lng2, lat3, lng3, d13):
        b13 = self.deg2rad(self.trueBearing(lat1, lng1, lat3, lng3))
        b12 = self.deg2rad(self.trueBearing(lat1, lng1, lat2, lng2))
        return R_MEAN*asin(sin(d13/R_MEAN)*sin(b13 - b12))

    def alongTrackDistance(self, lat1, lng1, lat2, lng2, lat3, lng3, d13, crossTrack):
        return R_MEAN*acos(cos(d13/R_MEAN)/cos(crossTrack/R_MEAN))

    def headingOK(self, error):
        return abs(error) < HEADING_OK_LIMIT

    def orientedDiff(self, angle_from, angle_to):
        return self.mod360(angle_to - angle_from)

    def isBetweenOrientedAngles(self, angle, angle_from, angle_to):
        return self.orientedDiff(angle_from, angle) < self.orientedDiff(angle_from, angle_to)

    def isTurnAgainstWind(self, windAngle, angle_from, angle_to):
        return self.isBetweenOrientedAngles(self.mod360(windAngle + 180), angle_from, angle_to)

    def limits(self, x, x_min, x_max):
        if x < x_min:
            return x_min;
        elif x > x_max:
            return x_max;
        return x;

    def calcAdjustedAngles(self, ghostHeading, trueWind):
        angle1 = self.mod360(trueWind - 45) #with the wind
        angle2 = self.mod360(trueWind + 45) #with the wind
        angle3 = self.mod360(trueWind + 135) #against the wind
        angle4 = self.mod360(trueWind - 135) #against the wind

        canGoDirectly = False
        adjAngle1 = 0.0
        adjAngle2 = 0.0

        if self.isBetweenOrientedAngles(ghostHeading, angle1, angle2):
            canGoDirectly = False
            adjAngle1 = angle1
            adjAngle2 = angle2
        elif self.isBetweenOrientedAngles(ghostHeading, angle2, angle3):
            canGoDirectly = True
            adjAngle1 = angle2
            adjAngle2 = angle3
        elif self.isBetweenOrientedAngles(ghostHeading, angle3, angle4):
            canGoDirectly = False
            adjAngle1 = angle3
            adjAngle2 = angle4
        elif self.isBetweenOrientedAngles(ghostHeading, angle4, angle1):
            canGoDirectly = True
            adjAngle1 = angle4
            adjAngle2 = angle1

        return canGoDirectly, adjAngle1, adjAngle2

    def bestAdjHeading(self, adjAngle1, adjAngle2, trueHeading, trueWind):
        opWind = self.mod360(trueWind + 180)
        if (self.isBetweenOrientedAngles(trueWind, adjAngle1, trueHeading) \
        or self.isBetweenOrientedAngles(opWind, adjAngle1, trueHeading)) \
        and (self.isBetweenOrientedAngles(trueWind, trueHeading, adjAngle1) \
        or self.isBetweenOrientedAngles(opWind, trueHeading, adjAngle1)):
            return adjAngle2, TACKMODE_ADJ_POS
        elif (self.isBetweenOrientedAngles(trueWind, adjAngle2, trueHeading) \
        or self.isBetweenOrientedAngles(opWind, adjAngle2, trueHeading)) \
        and (self.isBetweenOrientedAngles(trueWind, trueHeading, adjAngle2) \
        or self.isBetweenOrientedAngles(opWind, trueHeading, adjAngle2)):
            return adjAngle1, TACKMODE_ADJ_NEG
        else:
            T.delete('1.0', END)
            T.insert(END, "wtf")
        return 0, 0

    def pauseResume(self):
        global paused
        paused = not paused

    def drawDot(self, x1, y1, r, color, tag):
        points = [x1 - r, y1 - r, x1 + r, y1 + r]
        self.c.create_oval(points, fill=color, width=0, tags=tag)

    def gps_drawCircle(self, lat, lng, r, color, tag):
        phi = self.rad2deg(r/R_MEAN)
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

    def updateLine(self, c_var, x0, y0, x1, y1, fill, width):
        if c_var is None:
            c_var = self.c.create_line(x0, y0, x1, y1, fill=fill, width=width)
        else:
            self.c.coords(c_var, x0, y0, x1, y1)
        return c_var

global x_prev, y_prev, gps_lat, gps_lng, trueHeading, tackmode, trueWind, ghostPoint, closestPoint, \
        ghostHeading, ghostHeading_instant, goHeading, error, sail_angle, crossTrack, pathAngle, rudder, \
        speed, paused, c_closestPoint, c_ghostPoint, c_ghostHeading, c_windDir, \
        c_goHeading, c_adjHeading1, c_adjHeading2, c_boat, c_sail, c_rudder, c_compass1, \
        c_compass2, c_compass3, c_compass4, tackmode_str, ghostHeading_initialized
gps_lat, gps_lng = WAYPOINT0
gps_lat_prev, gps_lng_prev = gps_lat, gps_lng
x_prev = 0.0
y_prev = 0.0
trueHeading = 180.0
tackmode = TACKMODE_DIRECTLY
trueWind = 0.0
ghostPoint = 0.0, 0.0
closestPoint = 0.0, 0.0
ghostHeading = 0.0
ghostHeading_instant = 0.0
goHeading = 0.0
error = 0.0
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
