#*******************************************************************************************************************#
#  Project: Prescott Yacht Club - Autonomous Sailing Project
#  File: waypointNav.py
#  Author: Lorenzo Kearns
#  Versions:
#   version: 0.1 10/25/2021 - Initial Program Creation
#   version: 1.0 10/26/2021 - Working map of lynx lake with accurate longitude and lattitude overlay
#   version: 2.0 11/01/2021 - Reconfigure of the program without initial startup map, working navigation map which saves longitude
#                  and lattitude of mouse click. Removed excess code features, renamed variables to be more intuitve, cleaned up code and added comments
#  Purpose: GUI for selecting waypoints and displaying desired data from boat sensors.
#*******************************************************************************************************************#
#
#
#****************************************************************#
# Includes:
import io
import numpy as np
import tkinter as tk
from PIL import Image
from tkinter import *
import cartopy.crs as ccrs
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
import cartopy.feature as cfeature
import cartopy.io.img_tiles as cimgt
from urllib.request import urlopen, Request
from tkinter.filedialog import askopenfilename
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
#****************************************************************#
#
#
        # give a message to tell the order in which the boundaries of the naviagtion console will be constrained
def image_spoof(self, tile): # this function pretends not to be a Python script
    url = self._image_url(tile) # get the url of the street map API
    req = Request(url) # start request
    req.add_header('User-agent','Anaconda 3') # add user agent to request
    fh = urlopen(req)
    im_data = io.BytesIO(fh.read()) # get image
    fh.close() # close url
    img = Image.open(im_data) # open image with PIL
    img = img.convert(self.desired_tile_form) # set image format
    return img, self.tileextent(tile), 'lower' # reformat for cartopy

def plotStuff():
    cimgt.OSM.get_image = image_spoof # reformat web request for street map spoofing
    map = cimgt.OSM()
    fig = plt.figure(figsize = (8, 6), dpi = 100)
    ax1 = plt.axes(projection = map.crs)
    lakeCenter = [34.52, -112.386]
    zoomRatio = 0.005
    extent = [lakeCenter[1]-(zoomRatio*0.6),lakeCenter[1]+(zoomRatio*0.8),lakeCenter[0]-(zoomRatio),lakeCenter[0]+(zoomRatio*0.6)] # adjust to zoom
    ax1.set_extent(extent) # set extents
    scale = np.ceil(-np.sqrt(2)*np.log(np.divide(zoomRatio,350.0))) # empirical solve for scale based on zoom
    scale = (scale<20) and scale or 19 # scale cannot be larger than 19
    ax1.add_image(map, int(scale)) # add OSM with zoom specification
    x = [-112.388989, -112.-112.3857]
    y = [34.5150517, 34.519631]
    plt.plot(x, y, 'r--')
    plt.show() # show the plot
#**********************************************************************************************#
# Define the main functionality of the program
#**********************************************************************************************#
def main():
    global lock
    lock = True
    lonOrig = -112.388989 # set the longitidue at the origin
    latOrig = 34.5150517 # set the lattitude at the origin
    extentLat = -0.007983 # set the extent of the lattitude from orgin to edge
    extentLon = 0.006978 # set the extent of the Longitude from orgin to edge
        # Create a tkinter object
    root = tk.Tk()
        #create canvas for the controlCenter
    controlCenter = Canvas(root, width=1920, height=1080)
    controlCenter.pack()
        # process an image for the waypoint selection window
    File = 'newLynxLake.png' # path to image that shall be used
    original = Image.open(File)
    img = ImageTk.PhotoImage(original) # create a Tk image from the file
    waypointTitle = tk.Label(controlCenter, text = "Waypoint Nav Selection").place(x = 180, y = 10)
    controlCenter.create_image(30, 30, image=img, anchor="nw") # add the image to the controlCenter canvas
        # set values for the scaling factor to be used in defining canvas boundaries
    xmLon = extentLon
    ymLat = extentLat
        # set the initial positions in lat and longitude at the origin
    xInitial = lonOrig
    yInitial = latOrig
#********#
# Ill be real, I Just totally gave up on proper function formatting since they are all in the damn loop.
#  At some point I think I can put all this into an object using classes or something,
#   for now avert your eyes future me and pretend there are now functions here
#
    # Determine the origin by clicking
    def getorigin(eventorigin):
        global x0,y0
        x0 = 34
        y0 = 609
        print(x0,y0)
        controlCenter.bind("<Button 1>",getextentx)
    # mouseclick event
    controlCenter.bind("<Button 1>",getorigin)
    # Determine the extent of the figure in the x direction (Temperature)
    def getextentx(eventextentx):
        global xe
        xe = 453
        print(xe)
        controlCenter.bind("<Button 1>",getextenty)
    # Determine the extent of the figure in the y direction (Pressure)
    def getextenty(eventextenty):
        global ye
        ye = 33
        print(ye)
        tk.messagebox.showinfo("All good!", "The grid is all good. Start Navigating")
        controlCenter.bind("<Button 1>",printcoords)

    # print the coords to the console
    def printcoords(event):
        global lock
        # if(lock == True):
        if(lock == False):
            xDelta = xe-x0
            xm = xmLon/xDelta
            yDelta = ye-y0
            ym = -ymLat/yDelta
            # Perform coordinate transformation to normalize results
            lonWaypoint = (event.x-x0)*(xm)+xInitial
            latWaypoint = (event.y-y0)*(ym)+yInitial
            #outputting Longitude and Latitude coords to console
            print (lonWaypoint,latWaypoint)
            longitude = tk.Label(controlCenter, text = "Selected Longitude: "+ str(lonWaypoint)+ "              ").place(x = 30, y = 610)
            lattitude = tk.Label(controlCenter, text = "Selected Lattitude: "+ str(latWaypoint) + "             ").place(x = 30, y = 630)
            lock = True # sets lock after one waypoint selection, to select a new waypoint a fresh click of place waypoint must be done
    def cycleLock():
        global lock
        if (lock == False):
            lock = True
        elif (lock == True):
            lock = False
        # make a little button boi, he says "Place waypoint" and when clicked allows you to..... Place a waypoint
    tk.Button(controlCenter, text='Place Waypoint', command = cycleLock).place(x = 460, y = 30)
    tk.Button(controlCenter, text='Show real Plot', command = plotStuff). place(x = 460, y = 60)
        # loop until the code inevitably crashes again because tkinter is ass
    root.mainloop()
# end of the main function
#**********************************************************************************************#
#*#
#*#
#*#
#**********************************************************************************************#
# Set rule to run main program and execute code
#**********************************************************************************************#
if __name__ == '__main__':
    main()
# END OF ACTIVE CODE
#**********************************************************************************************#
