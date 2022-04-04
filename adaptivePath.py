from tkinter import *
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
NavigationToolbar2Tk)
from shapely import affinity
import cartopy.io.img_tiles as cimgt
import matplotlib.pyplot as plt
import numpy as np
import cartopy.crs as ccrs
from shapely.geometry import Point, LineString, Polygon
import pandas as pd
import matplotlib.animation as animation
from matplotlib import style
from polarChart import Sailing
from polarChart import Polar
import time

class LatLon():
    def __init__(self, lattitude, longitude):
        self.lat = lattitude
        self.lon = longitude

class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self,master)
        self.wind_angle = 30 # wind angle in relation ot north
        self.true_heading = 0 # boat anlge relative to north
        self.moving_right = True # tracks if the boat is moving right, if this is true the next tack is a left tack
        boat = Polar()
        self.p_chart = boat.plot_polar()
        self.sailor = Sailing(self.p_chart)
        self.target = LatLon(34.5217, -112.38586)
        self.create_polygon()
        self.createWidgets()

    def createWidgets(self):
        fig = Figure(figsize = (5, 5),
                     dpi = 100)
        self.a1 = fig.add_axes([0,0,1,1])
        self.a1.plot(*self.boat_poly.exterior.xy)
        self.a1.plot(*self.poly.exterior.xy)
        self.a1.set_title('Boat Location')
        self.canvas = FigureCanvasTkAgg(fig,
                                   master = root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack()

        self.plotbutton = tk.Button(master=root, text="plot", command=lambda: self.plot(self.canvas,self.a1))
        self.movebutton = tk.Button(master = root, text = "move boat forward", command = self.run_adaptive_path)
        # self.plotbutton.pack()
        # self.movebutton.pack()
        while(True):
            self.run_adaptive_path()

    def plot(self,canvas,a1):
        self.a1.clear()         # clear axes from previous plot
        self.a1.plot(*self.boat_poly.exterior.xy)
        self.a1.plot(*self.poly.exterior.xy)
        self.a1.set_xlim(-112.388989, -112.382)
        self.a1.set_ylim(34.5150517, 34.5230208)
        self.canvas.draw()
        self.min_rec_dist = 0.000136
        safety_threshold = self.boat_poly.exterior.distance(self.poly.exterior) - self.min_rec_dist
        if(safety_threshold > 0):
            tk.Label(root, text = "                                                                                         ").place(x = 500, y = 600)
            tk.Label(root, text = "The boat is safe and not too close to the shore").place(x = 500, y = 600)
        else:
            tk.Label(root, text = "                                                                                          ").place(x = 500, y = 600)
            tk.Label(root, text = "Danger the boat is at risk of collision").place(x = 500, y = 600)

    def create_polygon(self):
            temp_container = pd.read_csv('newOutputBoundaries2.csv')
            temp_container = np.array(temp_container)
            boundaryArrayLat = temp_container[:,1]
            boundaryArrayLon = temp_container[:,2]
            boundaryArrayLon = np.array(boundaryArrayLon)
            boundaryArrayLat = np.array(boundaryArrayLat)
            coordTuple = np.array((boundaryArrayLon,boundaryArrayLat)).T
            self.poly = Polygon(coordTuple)
            boat_dimensions = [(-112.3849, 34.5161), (-112.3849004, 34.51610205), (-112.3849005, 34.5161041), (-112.38490051, 34.51610605), (-112.38490051, 34.51610905), (-112.38490031, 34.51611205), (-112.3849, 34.516116), (-112.38489969, 34.51611205),(-112.38489949, 34.51610905), (-112.38489949, 34.51610905), (-112.38489949, 34.51610605), (-112.38489950, 34.5161041), (-112.3848996, 34.51610205)]
            # boat_dimensions = [(-112.3849, 34.51611), (-112.3849, 34.5161), (-112.384799, 34.5161), (-112.384802, 34.5161)]
            self.boat_poly = Polygon(boat_dimensions)

    def move_forward(self):
        x = 0.000036 * np.cos(np.pi/180 * self.true_heading)
        y = 0.000036 * np.sin(np.pi/180 * self.true_heading)
        self.boat_poly = affinity.translate(self.boat_poly, x, y)
        self.plot(self.canvas,self.a1)

    def set_heading(self, ideal_heading):
        while(self.true_heading != ideal_heading):
            if (self.true_heading < 0):
                self.true_heading = 360 - self.true_heading
            if (self.true_heading >= 360):
                self.true_heading = 0
            if(self.true_heading < ideal_heading):
                self.true_heading = self.true_heading + 1
                self.boat_poly = affinity.rotate(self.boat_poly, 1)
                self.plot(self.canvas,self.a1)
            elif(self.true_heading > self.wind_angle):
                self.true_heading = self.true_heading - 1
                self.boat_poly = affinity.rotate(self.boat_poly, -1)
                self.plot(self.canvas,self.a1)

    def run_adaptive_path(self):
        self.speed = 20
        self.min_rec_dist = 0.000136
        twa = self.true_heading - self.wind_angle # find the true wind angle relative to the boat
        if(twa < 0):
            twa = 360 + twa
        print(twa)
        heading_angle_starboard, starboard_distance, heading_angle_port, port_distance, distance =  self.sailor.get_cruise(self.speed, twa)
        # self.h_angle, p_time, self.second_h_angle = self.sailor.get_optimal_direction(int(self.speed), int(twa))
        print(heading_angle_starboard)
        print(heading_angle_port)
        distance = distance/ 111320
        print(distance)
        self.set_heading(heading_angle_starboard)
        safety_threshold = self.boat_poly.exterior.distance(self.poly.exterior) - self.min_rec_dist
        if(safety_threshold > 0):
            self.move_forward()
        # else:
        #     if(distance )
            # self.move_forward()
        # time.sleep(0.5)

root = Tk()
# setting the title
root.title('Plotting in Tkinter')
app = Application(master = root)
app.mainloop()
# window.mainloop()
