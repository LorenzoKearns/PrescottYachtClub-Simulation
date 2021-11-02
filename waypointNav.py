#*******************************************************************************************************************#
#  Project: Prescott Yacht Club -
#  File: simEnviroment.py
#  Author: Lorenzo Kearns
#  Versions:
#   version: 0.1 10/25/2021 - Initial Program Creation
#   version: 1.0 10/26/2021 - Working map of lynx lake with accurate longitude and lattitude overlay
#  Purpose: Take power budget stuff and perform simulations over time
#*******************************************************************************************************************#
#
#
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
# from cartopy.io.img_tiles import OSM
#
#
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
#
#
#**********************************************************************************************#
# Define the main functionality of the program
#**********************************************************************************************#
def main():
    global lock
    lock = True
    cimgt.OSM.get_image = image_spoof # reformat web request for street map spoofing
    map = cimgt.OSM()
    fig = plt.figure(figsize = (8, 6), dpi = 100)
    ax1 = plt.axes(projection = map.crs)
    lakeCenter = [34.52, -112.386]
    lonOrig = -112.388989 # set the longitidue at the origin
    latOrig = 34.5150517 # set the lattitude at the origin
    # maxLon = -112.382011
    # maxLat = 34.523000
    extentLat = -0.007983
    extentLon = 0.006978
    zoomRatio = 0.005
    extent = [lakeCenter[1]-(zoomRatio*0.6),lakeCenter[1]+(zoomRatio*0.8),lakeCenter[0]-(zoomRatio),lakeCenter[0]+(zoomRatio*0.6)] # adjust to zoom
    ax1.set_extent(extent) # set extents
    scale = np.ceil(-np.sqrt(2)*np.log(np.divide(zoomRatio,350.0))) # empirical solve for scale based on zoom
    scale = (scale<20) and scale or 19 # scale cannot be larger than 19
    ax1.add_image(map, int(scale)) # add OSM with zoom specification
    plt.show() # show the plot
    # grab mouse position of a tkinter gui
    root = tk.Tk()
    #setting up a tkinter canvas
    controlCenter = Canvas(root, width=1920, height=1080)
    controlCenter.pack()
    #adding the image
    # File = askopenfilename(parent=root, initialdir="./",title='Select an image')
    File = 'newLynxLake.png'
    original = Image.open(File)
    # original = original.resize((1000,1000)) #resize image
    img = ImageTk.PhotoImage(original)
    controlCenter.create_image(50, 50, image=img, anchor="nw")

    xmt = extentLon
    ymp = extentLat
    #ask for real PT values at origin
    # xInitial = tk.simpledialog.askfloat("Longitude", "Longitude at origin")
    # yInitial = tk.simpledialog.askfloat("Lattitude", "Lattitude at origin")

    xInitial = lonOrig
    yInitial = latOrig
    #instruction on 3 point selection to define grid
    tk.messagebox.showinfo("Set plot bounds", "Click Order: \n"
                                                "1) Origin \n"
                                                "2) Longitude end \n"
                                                "3) Lattitude end")
    # Determine the origin by clicking
    def getorigin(eventorigin):
        global x0,y0
        x0 = eventorigin.x
        y0 = eventorigin.y
        print(x0,y0)
        controlCenter.bind("<Button 1>",getextentx)
    #mouseclick event
    controlCenter.bind("<Button 1>",getorigin)
    # Determine the extent of the figure in the x direction (Temperature)
    def getextentx(eventextentx):
        global xe
        xe = eventextentx.x
        print(xe)
        controlCenter.bind("<Button 1>",getextenty)
    # Determine the extent of the figure in the y direction (Pressure)
    def getextenty(eventextenty):
        global ye
        ye = eventextenty.y
        print(ye)
        tk.messagebox.showinfo("All good!", "The grid is all good. Start Navigating")
        controlCenter.bind("<Button 1>",printcoords)
    def printcoords(event):
        global lock
        print(lock)
        if(lock == False):
            xmpx = xe-x0
            xm = xmt/xmpx
            ympx = ye-y0
            ym = -ymp/ympx
            #coordinate transformation
            lonWaypoint = (event.x-x0)*(xm)+xInitial
            latWaypoint = (event.y-y0)*(ym)+yInitial
            #outputting x and y coords to console
            print (lonWaypoint,latWaypoint)
            lock = True
    def cycleLock():
        global lock
        if (lock == False):
            lock = True
        elif (lock == True):
            lock = False
    tk.Button(controlCenter, text='Place Waypoint', command = cycleLock).place(x=500,y=100)

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











# CODE GRAVEYARD
