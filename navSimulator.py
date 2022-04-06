from math import *
from tkinter import *
import time
import threading
import numpy
import random
import serial

TACKMODE_DIRECTLY = 0
TACKMODE_ADJ_POS = 1
TACKMODE_ADJ_NEG = 2
TACKMODE_MAXDEV_POS = 3
TACKMODE_MAXDEV_NEG = 4

MENU = 4
W = 1500.0
H = 750.0
MAXW = 10*W
MAXH = 10*H

SIZEADJUST = 2

BOAT_LENGTH = 50/SIZEADJUST
BOAT_BOW = 20/SIZEADJUST
BOAT_WIDTH = 10/SIZEADJUST
RUDDER_LENGTH = 20/SIZEADJUST
SAIL_LENGTH = 40/SIZEADJUST
FLAP_LENGTH = 20/SIZEADJUST
DOT_RADIUS = 2/SIZEADJUST
WIND_COMPASS = 100/SIZEADJUST
HEADING_ARROW = 100/SIZEADJUST

HEADING_OK_LIMIT = 10.0
RUDDER_RESPONSE = 0.00005
RUDDER_COEFF = 40.0
RUDDER_MIN_ANGLE = -30
RUDDER_MAX_ANGLE = 35

tackmode_str = ['TACKMODE_DIRECTLY', 'TACKMODE_ADJ_POS', 'TACKMODE_ADJ_NEG', 'TACKMODE_MAXDEV_POS', 'TACKMODE_MAXDEV_NEG']

FLAP_NORMAL = 10.0
FLAP_MAX = 15.0
TACK_SAIL_CRITICAL_ANGLE = FLAP_MAX/2
FLAP_ITERATION = 0.1

R_MEAN = 6371000.0

MAXDEV = 500000
MAXDEV_OK_FACTOR = 0.75
MAXDEV_OK = MAXDEV * MAXDEV_OK_FACTOR

# WAYPOINT0 = 46.913520, -52.998886
# WAYPOINT1 = 48.0, -13.0

WAYPOINT0 =  34.5217, -112.38586,
WAYPOINT1 =  34.5161, -112.3849

tackmode = TACKMODE_DIRECTLY

gps_lat, gps_lng = WAYPOINT0
gps_lat_prev, gps_lng_prev = gps_lat, gps_lng

trueHeading = 45.0
MAX_SPEED = 2000000.0
GHOST_DISTANCE = 5.0
trueWind = 0.0

dt = 0.001
dt_refresh = 4*dt

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
flap = FLAP_NORMAL
flap_final = flap

x_prev = 0.0
y_prev = 0.0

speed = 0.0

paused = False

ghostHeading_initialized = False

sim_active = False;

adjAngle1 = 0.0 #global only for debugging
adjAngle2 = 0.0 #global only for debugging

c_boat, c_sail, c_rudder, c_flap = None, None, None, None
c_compass1, c_compass2, c_compass3, c_compass4, \
c_closestPoint, c_ghostPoint, c_ghostHeading, c_windDir, c_goHeading, c_adjHeading1, c_adjHeading2 \
    = None, None, None, None, None, None, None, None, None, None, None

ser = None

startchar = ''

sim_nextWaypoint = -1
mux = -1
isBehindPath = 0
magDec = 0.0


def gps2xy(lat, lng):
    SCALE = 100.0
    x, y = lng, lat
    wp0 = WAYPOINT0[1], WAYPOINT0[0]
    return SCALE*(x - wp0[0]) + 100, -SCALE*(y - wp0[1]) + MAXH/2

def gps_create_line(x0, y0, x1, y1, fill, width, tags):
    c.create_line(*gps2xy(x0, y0), *gps2xy(x1, y1), fill=fill, width=width, tags=tags)

def gps_create_greatcircle(lat0, lng0, lat1, lng1, maxDev, fill, width, tags):
    MOVE_ITER = 5000
    dest = lat0, lng0
    new_dest = dest
    while 1:
        bearing = trueBearing(*dest, lat1, lng1)
        new_dest = Destination(*dest, bearing, MOVE_ITER)
        gps_create_line(*dest, *new_dest, fill=fill, width=width, tags=tags)

        dest_plus_maxdev = Destination(*dest, bearing + 90, maxDev)
        new_dest_plus_maxdev = Destination(*new_dest, bearing + 90, maxDev)
        gps_create_line(*dest_plus_maxdev, *new_dest_plus_maxdev, fill='yellow', width=width, tags=tags)

        dest_minus_maxdev = Destination(*dest, bearing - 90, maxDev)
        new_dest_minus_maxdev = Destination(*new_dest, bearing - 90, maxDev)
        gps_create_line(*dest_minus_maxdev, *new_dest_minus_maxdev, fill='yellow', width=width, tags=tags)

        dest_plus_maxdev_ok = Destination(*dest, bearing + 90, maxDev*MAXDEV_OK_FACTOR)
        new_dest_plus_maxdev_ok = Destination(*new_dest, bearing + 90, maxDev*MAXDEV_OK_FACTOR)
        gps_create_line(*dest_plus_maxdev_ok, *new_dest_plus_maxdev_ok, fill='yellow', width=width, tags=tags)

        dest_minus_maxdev_ok = Destination(*dest, bearing - 90, maxDev*MAXDEV_OK_FACTOR)
        new_dest_minus_maxdev_ok = Destination(*new_dest, bearing - 90, maxDev*MAXDEV_OK_FACTOR)
        gps_create_line(*dest_minus_maxdev_ok, *new_dest_minus_maxdev_ok, fill='yellow', width=width, tags=tags)

        #if dest[0] < lat1 and new_dest[0] > lat1:
            #break
        #if dest[0] > lat1 and new_dest[0] < lat1:
        #    break
        if dest[1] < lng1 and new_dest[1] > lng1:
            break
        if dest[1] > lng1 and new_dest[1] < lng1:
            break

        dest = new_dest

def rot(x1, y1, a1, x0, y0):
    b = deg2rad(a1)
    return x0 + (x1 - x0)*cos(b) - (y1 - y0)*sin(b), \
            y0 + (y1 - y0)*cos(b) + (x1 - x0)*sin(b)


def refresh():
    global flap, flap_final, trueWind, trueHeading, tackmode, tackmode_str
    drawBoat()
    Tflap_final.delete('1.0', END)
    Tflap_final.insert(END, flap_final)
    Tflap.delete('1.0', END)
    Tflap.insert(END, flap)
    Twind.delete('1.0', END)
    Twind.insert(END, trueWind)
    Theading.delete('1.0', END)
    Theading.insert(END, trueHeading)
    Ttack.delete('1.0', END)
    Ttack.insert(END, tackmode_str[tackmode])
    Tnextwp.delete('1.0', END)
    Tnextwp.insert(END, sim_nextWaypoint)
    Tmux.delete('1.0', END)
    Tmux.insert(END, mux)
    Tbehindpath.delete('1.0', END)
    Tbehindpath.insert(END, isBehindPath)
    Tmagdec.delete('1.0', END)
    Tmagdec.insert(END, magDec)

def flipflap():
    global flap_final
    flap_final = -flap_final
    refresh()

def drawDot(x1, y1, r, color, tag):
    points = [x1 - r, y1 - r, x1 + r, y1 + r]
    c.create_oval(points, fill=color, width=0, tags=tag)

def gps_drawCircle(lat, lng, r, color, tag):
    phi = rad2deg(r/R_MEAN)
    points = [*gps2xy(lat - phi, lng - phi),
              *gps2xy(lat + phi, lng + phi)]
    c.create_oval(points, outline=color, width=1, tags=tag)

def updateDot(c_var, x1, y1, r, color):
    points = [x1 - r, y1 - r, x1 + r, y1 + r]
    if c_var is None:
        c_var = c.create_oval(points, fill=color, width=0)
    else:
        c.coords(c_var, points)
    return c_var

def changeWind(event):
    global trueWind
    trueWind = 180 - rad2deg(atan2(event.x - W/2, event.y - H/2))
    refresh()

def drawBoat():
    global gps_lat, gps_lng, \
    c_boat, c_sail, c_rudder, c_flap

    x, y = gps2xy(gps_lat, gps_lng)

    #boat
    points = [rot(x - BOAT_WIDTH/2, y + BOAT_LENGTH/2, trueHeading, x, y),
              rot(x - BOAT_WIDTH/2, y - BOAT_LENGTH/2 + BOAT_BOW, trueHeading, x, y),
              rot(x, y - BOAT_LENGTH/2, trueHeading, x, y),
              rot(x + BOAT_WIDTH/2, y - BOAT_LENGTH/2 + BOAT_BOW, trueHeading, x, y),
              rot(x + BOAT_WIDTH/2, y + BOAT_LENGTH/2, trueHeading, x, y)]

    #if not c_boat is None:
    c.delete('boat')

    c_boat = c.create_polygon(points, fill='blue', width=0, tags='boat')

    #rudder
    points = [*rot(*rot(x, y + BOAT_LENGTH/2, rudder, x, y + BOAT_LENGTH/2), trueHeading, x, y),
                *rot(*rot(x, y + BOAT_LENGTH/2 + RUDDER_LENGTH, rudder, x, y + BOAT_LENGTH/2), trueHeading, x, y)]
    c_rudder = updateLine(c_rudder, *points, fill='black', width=2)

    #sail
    points = [x, y, *rot(x, y - SAIL_LENGTH, trueWind - flap, x, y)]
    c_sail = updateLine(c_sail, *points, fill='black', width=2)

    #flap
    flap_start = rot(x, y - SAIL_LENGTH, trueWind - flap, x, y)
    flap_end = rot(x, y - SAIL_LENGTH - FLAP_LENGTH, trueWind - flap, x, y)
    points = [*flap_start, *rot(*flap_end, flap, *flap_start)]
    c_flap = updateLine(c_flap, *points, fill='red', width=3)

def updateLine(c_var, x0, y0, x1, y1, fill, width):
    if c_var is None:
        c_var = c.create_line(x0, y0, x1, y1, fill=fill, width=width)
    else:
        c.coords(c_var, x0, y0, x1, y1)
    return c_var

def rad2deg(x):
    return float(x) / pi * 180

def deg2rad(x):
    return float(x) / 180 * pi

def moveCycle():
    global trueWind

    if not paused and (not startchar == 'S'):

        #trueWind += 4*random.randint(-1, 1)

        move()

        intensiveTasks()
        where2go()
        adjustRudder()
        adjustFlap()

    threading.Timer(dt, moveCycle).start()

def intensiveTasks():
    global paused, closestPoint, ghostPoint, gps_lat, gps_lng, \
    trueHeading, ghostHeading, goHeading, error, rudder, speed, trueWind, adjAngle1, adjAngle2, \
    tackmode, ghostHeading_initialized, ghostHeading_instant, flap, flap_final, \
    ghostHeading_instant, crossTrack, pathAngle

    startAngle = trueBearing(*WAYPOINT0, *WAYPOINT1)
    pathLength = Distance(*WAYPOINT0, *WAYPOINT1)
    d13 = Distance(*WAYPOINT0, gps_lat, gps_lng)
    crossTrack = crossTrackDistance(*WAYPOINT0, *WAYPOINT1, gps_lat, gps_lng, d13)

    if Distance(gps_lat, gps_lng, *WAYPOINT1) > pathLength:
        alongTrack = 0
    else:
        alongTrack = alongTrackDistance(*WAYPOINT0, *WAYPOINT1, gps_lat, gps_lng, d13, crossTrack)

    if alongTrack + GHOST_DISTANCE > pathLength:
        ghostPoint = WAYPOINT1
    else:
        ghostPoint = Destination(*WAYPOINT0, startAngle, alongTrack + GHOST_DISTANCE)

    closestPoint = Destination(*WAYPOINT0, startAngle, alongTrack)
    pathAngle = trueBearing(*closestPoint, *WAYPOINT1)
    ghostHeading_instant = trueBearing(gps_lat, gps_lng, *ghostPoint)

def where2go():
    global paused, closestPoint, ghostPoint, gps_lat, gps_lng, \
    trueHeading, ghostHeading, goHeading, error, rudder, speed, trueWind, adjAngle1, adjAngle2, \
    tackmode, ghostHeading_initialized, flap, flap_final, \
    crossTrack, sail_angle

    #update ghostHeading only if heading is OK

    if not ghostHeading_initialized or headingOK(error):
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

    canGoDirectly, adjAngle1, adjAngle2 = calcAdjustedAngles(ghostHeading, trueWind)

    if tackmode == TACKMODE_MAXDEV_NEG:
        if canGoDirectly:
            goHeading = ghostHeading
        else:
            if not isBetweenOrientedAngles(adjAngle1, pathAngle, pathAngle + 140):
                goHeading = adjAngle2
            elif not isBetweenOrientedAngles(adjAngle2, pathAngle, pathAngle + 140):
                goHeading = adjAngle1
            else:
                goHeading, tm_ = bestAdjHeading(adjAngle1, adjAngle2, trueHeading, trueWind)
    elif tackmode == TACKMODE_MAXDEV_POS:
        if canGoDirectly:
            goHeading = ghostHeading
        else:
            if not isBetweenOrientedAngles(adjAngle1, pathAngle - 140, pathAngle):
                goHeading = adjAngle2
            elif not isBetweenOrientedAngles(adjAngle2, pathAngle - 140, pathAngle):
                goHeading = adjAngle1
            else:
                goHeading, tm_ = bestAdjHeading(adjAngle1, adjAngle2, trueHeading, trueWind)
    else:
        if canGoDirectly:
            goHeading = ghostHeading
            tackmode = TACKMODE_DIRECTLY
        else:
            goHeading, tackmode = bestAdjHeading(adjAngle1, adjAngle2, trueHeading, trueWind)

    sail_angle = mod360(trueWind - trueHeading - flap)

def adjustRudder():
    global paused, closestPoint, ghostPoint, gps_lat, gps_lng, \
    trueHeading, ghostHeading, goHeading, error, rudder, speed, trueWind, adjAngle1, adjAngle2, \
    tackmode, ghostHeading_initialized, flap, flap_final, \
    sail_angle

    #calculate error and adjust it based on the wind direction (boat must not turn against the wind)

    angle_positive = orientedDiff(trueHeading, goHeading)

    if not isTurnAgainstWind(trueWind, trueHeading, goHeading):
        error = angle_positive;
    else:
        error = -(360 - angle_positive);

    #boat is against the wind and is reversed, do a special maneuver to safely switch forward (can't be the opposite way... proven)

    if sail_angle < 180 and flap > 0 or sail_angle > 180 and flap < 0: #reversed
        if sail_angle > 180 - 45 - 2*FLAP_NORMAL and sail_angle < 180 and flap > 0: #against the wind
            rudder = RUDDER_MAX_ANGLE #boat "turning left", but right backwards
        elif sail_angle < 180 + 45 + 2*FLAP_NORMAL and sail_angle > 180 and flap < 0:
            rudder = RUDDER_MIN_ANGLE
#            else:
#                rudder = 0 #not needed, rather prevent rudder jitter
    else: #not reversed
        rudder = limits(round(-RUDDER_COEFF / 100 * error), RUDDER_MIN_ANGLE, RUDDER_MAX_ANGLE)

def adjustFlap():
    global paused, closestPoint, ghostPoint, gps_lat, gps_lng, \
    trueHeading, ghostHeading, goHeading, error, rudder, speed, trueWind, adjAngle1, adjAngle2, \
    tackmode, ghostHeading_initialized, flap, flap_final, \
    sail_angle

    #ADJUSTING FLAP (flap_final)

    if sail_angle < 180 and flap > 0 and flap_final > 0:
        flap_final = -FLAP_NORMAL

    elif error < 0 and sail_angle > 360 - TACK_SAIL_CRITICAL_ANGLE and flap > 0 and flap_final > 0:
        flap_final = -FLAP_MAX

    elif sail_angle > 180 and flap < 0 and flap_final < 0:
        flap_final = FLAP_NORMAL

    elif error > 0 and sail_angle < 0 + TACK_SAIL_CRITICAL_ANGLE and flap < 0 and flap_final < 0:
        flap_final = FLAP_MAX

    #FLAP FROM MAX BACK TO NORMAL (flap_final)

    elif flap < 0 and flap_final <= -FLAP_MAX and sail_angle < 180 and sail_angle > 45:
        flap_final = -FLAP_NORMAL

    elif flap > 0 and flap_final >= FLAP_MAX and sail_angle > 180 and sail_angle < 360 - 45:
        flap_final = FLAP_NORMAL

def refreshCycle():
    global x_prev, y_prev, paused, ghostHeading, error, goHeading, trueWind, trueHeading, adjAngle1, adjAngle2, \
    c_compass1, c_compass2, c_compass3, c_compass4, c_closestPoint, c_ghostPoint, c_ghostHeading, c_windDir, c_goHeading, c_adjHeading1, c_adjHeading2

    x, y = gps2xy(gps_lat, gps_lng)

    if not paused:

        refresh()

        #trail path
        c.create_line(x_prev, y_prev, x, y, fill='black', width=1, tags='boat_path')
        x_prev, y_prev = x, y

        #scroll
        x0 = hbar.get()[0] * MAXW
        y0 = vbar.get()[0] * MAXH
        if x < x0 or x > x0 + W or y < y0 or y > y0 + H:
            c.xview_moveto((x - W/3)/MAXW)
            c.yview_moveto((y - H/2)/MAXH)
            root.update()

        #compass
        x0 = hbar.get()[0] * MAXW
        y0 = vbar.get()[0] * MAXH

        c_compass1 = updateLine(c_compass1, x0 + W/2 - WIND_COMPASS, y0 + H/2, x0 + W/2 + WIND_COMPASS, y0 + H/2, fill='gray', width=1)
        c_compass2 = updateLine(c_compass2, x0 + W/2, y0 + H/2 - WIND_COMPASS, x0 + W/2, y0 + H/2 + WIND_COMPASS, fill='gray', width=1)
        c_compass3 = updateLine(c_compass3, x0 + W/2 - WIND_COMPASS, y0 + H/2 - WIND_COMPASS, x0 + W/2 + WIND_COMPASS, y0 + H/2 + WIND_COMPASS, fill='gray', width=1)
        c_compass4 = updateLine(c_compass4, x0 + W/2 - WIND_COMPASS, y0 + H/2 + WIND_COMPASS, x0 + W/2 + WIND_COMPASS, y0 + H/2 - WIND_COMPASS, fill='gray', width=1)

        #closestPoint
        c_closestPoint = updateDot(c_closestPoint, *gps2xy(*closestPoint), DOT_RADIUS, 'green')

        #ghostPoint
        c_ghostPoint = updateDot(c_ghostPoint, *gps2xy(ghostPoint[0], ghostPoint[1]), DOT_RADIUS, 'green')
        c_ghostHeading = updateLine(c_ghostHeading, x, y, *gps2xy(ghostPoint[0], ghostPoint[1]), fill='green', width=1)

        #wind direction
        c_windDir = updateLine(c_windDir, x - HEADING_ARROW/3*sin(deg2rad(trueWind)), y + HEADING_ARROW/3*cos(deg2rad(trueWind)),
                      x + HEADING_ARROW*sin(deg2rad(trueWind)), y - HEADING_ARROW*cos(deg2rad(trueWind)), fill='gray', width=1)

        #goHeading
        c_goHeading = updateLine(c_goHeading, x, y, x + HEADING_ARROW*sin(deg2rad(goHeading)), y - HEADING_ARROW*cos(deg2rad(goHeading)), fill='magenta', width=1)

        #adjustedHeading
        c_adjHeading1 = updateLine(c_adjHeading1, x, y, x + HEADING_ARROW/1.5*sin(deg2rad(adjAngle1)), y - HEADING_ARROW/1.5*cos(deg2rad(adjAngle1)), fill='blue', width=1)
        c_adjHeading2 = updateLine(c_adjHeading2, x, y, x + HEADING_ARROW/1.5*sin(deg2rad(adjAngle2)), y - HEADING_ARROW/1.5*cos(deg2rad(adjAngle2)), fill='blue', width=1)

    threading.Timer(dt_refresh, refreshCycle).start()

def calcSpeed():
    global trueHeading, trueWind, flap
    sail_angle = mod360(trueWind - trueHeading - flap)
    ret = abs(MAX_SPEED*sin(deg2rad(sail_angle)))
    if sail_angle < 180 and flap > 0 or sail_angle > 180 and flap < 0:
        ret = -ret
    return ret

def move():
    global speed, dt, gps_lat, gps_lng, trueHeading, rudder, flap

    speed = calcSpeed()

    gps_lat, gps_lng = Destination(gps_lat, gps_lng, trueHeading, speed*dt)
    trueHeading += -RUDDER_RESPONSE*rudder*speed*dt
    trueHeading = mod360(trueHeading)

    if flap < flap_final:
        flap += FLAP_ITERATION
    else:
        flap -= FLAP_ITERATION

def mod360(a):
    while a > 360:
        a -= 360
    while a < 0:
        a += 360
    return a

def sq(x):
    return x*x

def trueBearing(lat1, lng1, lat2, lng2):
    lat1 = deg2rad(lat1)
    lng1 = deg2rad(lng1)
    lat2 = deg2rad(lat2)
    lng2 = deg2rad(lng2)
    angle = atan2(sin(lng2 - lng1) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lng2 - lng1))
    return rad2deg(angle)

def Destination(lat1, lng1, bearing, d):
    lat1 = deg2rad(lat1)
    lng1 = deg2rad(lng1)
    bearing = deg2rad(bearing)
    lat2 = asin(sin(lat1)*cos(d/R_MEAN) + cos(lat1)*sin(d/R_MEAN)*cos(bearing))
    lng2 = lng1 + atan2(sin(bearing)*sin(d/R_MEAN)*cos(lat1), cos(d/R_MEAN) - sin(lat1)*sin(lat2))
    return rad2deg(lat2), rad2deg(lng2)

def Distance(lat1, lng1, lat2, lng2):
    lat1 = deg2rad(lat1)
    lng1 = deg2rad(lng1)
    lat2 = deg2rad(lat2)
    lng2 = deg2rad(lng2)
    hav = sq(sin((lat2-lat1)/2)) + cos(lat1) * cos(lat2) * sq(sin((lng2-lng1)/2))
    if hav < 0:
        hav = 0 #shouldn't happen
    if hav > 1:
        hav = 1 #shouldn't happen
    return 2 * R_MEAN * atan2(sqrt(hav), sqrt(1-hav))

def crossTrackDistance(lat1, lng1, lat2, lng2, lat3, lng3, d13):
    b13 = deg2rad(trueBearing(lat1, lng1, lat3, lng3))
    b12 = deg2rad(trueBearing(lat1, lng1, lat2, lng2))
    return R_MEAN*asin(sin(d13/R_MEAN)*sin(b13 - b12))

def alongTrackDistance(lat1, lng1, lat2, lng2, lat3, lng3, d13, crossTrack):
    return R_MEAN*acos(cos(d13/R_MEAN)/cos(crossTrack/R_MEAN))

def headingOK(error):
    return abs(error) < HEADING_OK_LIMIT

def orientedDiff(angle_from, angle_to):
    return mod360(angle_to - angle_from)

def isBetweenOrientedAngles(angle, angle_from, angle_to):
    return orientedDiff(angle_from, angle) < orientedDiff(angle_from, angle_to)

def isTurnAgainstWind(windAngle, angle_from, angle_to):
    return isBetweenOrientedAngles(mod360(windAngle + 180), angle_from, angle_to)

def limits(x, x_min, x_max):
    if x < x_min:
        return x_min;
    elif x > x_max:
        return x_max;
    return x;

def calcAdjustedAngles(ghostHeading, trueWind):
    smallfix = FLAP_NORMAL
    angle1 = mod360(trueWind - 45 + smallfix) #with the wind
    angle2 = mod360(trueWind + 45 - smallfix) #with the wind
    angle3 = mod360(trueWind + 135 - smallfix) #against the wind
    angle4 = mod360(trueWind - 135 + smallfix) #against the wind

    canGoDirectly = False
    adjAngle1 = 0.0
    adjAngle2 = 0.0

    if isBetweenOrientedAngles(ghostHeading, angle1, angle2):
        canGoDirectly = False
        adjAngle1 = angle1
        adjAngle2 = angle2
    elif isBetweenOrientedAngles(ghostHeading, angle2, angle3):
        canGoDirectly = True
        adjAngle1 = angle2
        adjAngle2 = angle3
    elif isBetweenOrientedAngles(ghostHeading, angle3, angle4):
        canGoDirectly = False
        adjAngle1 = angle3
        adjAngle2 = angle4
    elif isBetweenOrientedAngles(ghostHeading, angle4, angle1):
        canGoDirectly = True
        adjAngle1 = angle4
        adjAngle2 = angle1

    return canGoDirectly, adjAngle1, adjAngle2

def bestAdjHeading(adjAngle1, adjAngle2, trueHeading, trueWind):
    opWind = mod360(trueWind + 180)
    if (isBetweenOrientedAngles(trueWind, adjAngle1, trueHeading) \
    or isBetweenOrientedAngles(opWind, adjAngle1, trueHeading)) \
    and (isBetweenOrientedAngles(trueWind, trueHeading, adjAngle1) \
    or isBetweenOrientedAngles(opWind, trueHeading, adjAngle1)):
        return adjAngle2, TACKMODE_ADJ_POS
    elif (isBetweenOrientedAngles(trueWind, adjAngle2, trueHeading) \
    or isBetweenOrientedAngles(opWind, adjAngle2, trueHeading)) \
    and (isBetweenOrientedAngles(trueWind, trueHeading, adjAngle2) \
    or isBetweenOrientedAngles(opWind, trueHeading, adjAngle2)):
        return adjAngle1, TACKMODE_ADJ_NEG
    else:
        T.delete('1.0', END)
        T.insert(END, "wtf")
    return 0, 0

def pauseResume():
    global paused
    paused = not paused

random.seed()
root = Tk()
root.wm_state('zoomed')

frame = Frame(root)
frame.pack()
button_flap = Button(frame, text="Flip Flap", command=flipflap)
button_flap.pack(side = LEFT)
button_pause = Button(frame, text="Pause/Resume", command=pauseResume)
button_pause.pack(side = LEFT)

Tflap_final = Text(frame, height=1, width=10)
Tflap_final.pack(side = LEFT)
Tflap = Text(frame, height=1, width=10)
Tflap.pack(side = LEFT)
Twind = Text(frame, height=1, width=10)
Twind.pack(side = LEFT)
Theading = Text(frame, height=1, width=10)
Theading.pack(side = LEFT)
Ttack = Text(frame, height=1, width=20)
Ttack.pack(side = LEFT)
Tnextwp = Text(frame, height=1, width=5)
Tnextwp.pack(side = LEFT)
Tmux = Text(frame, height=1, width=3)
Tmux.pack(side = LEFT)
Tbehindpath = Text(frame, height=1, width=3)
Tbehindpath.pack(side = LEFT)
Tmagdec = Text(frame, height=1, width=7)
Tmagdec.pack(side = LEFT)

Tsail_angle = Text(frame, height=1, width=10)
Tsail_angle.pack(side = LEFT)
T = Text(frame, height=1, width=10)
T.pack(side = LEFT)

c = Canvas(root, width=W, height=H, bg="white", scrollregion=(0, 0, MAXW, MAXH))

hbar=Scrollbar(root,orient=HORIZONTAL)
hbar.pack(side=BOTTOM,fill=X)
hbar.config(command=c.xview)

vbar=Scrollbar(root,orient=VERTICAL)
vbar.pack(side=RIGHT,fill=Y)
vbar.config(command=c.yview)

c.config(xscrollcommand=hbar.set, yscrollcommand=vbar.set)
c.pack(side=LEFT, expand=True, fill=BOTH)

c.bind("<Button-1>", changeWind)
c.bind("<B1-Motion>", changeWind)

gps_create_greatcircle(*WAYPOINT0, *WAYPOINT1, 5.0, fill='red', width=1, tags='path')

drawDot(*gps2xy(*WAYPOINT0), DOT_RADIUS, 'red', 'path')
drawDot(*gps2xy(*WAYPOINT1), DOT_RADIUS, 'red', 'path')

RADIUS = 700.0

gps_drawCircle(*WAYPOINT1, RADIUS, 'red', 'path')

x_prev, y_prev = gps2xy(gps_lat, gps_lng)

moveCycle()

refreshCycle()
mainloop()
