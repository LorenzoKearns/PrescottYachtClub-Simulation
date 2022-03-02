#*******************************************************************************************************************#
#  Project: Prescott Yacht Club - Autonomous Sailing Project
#  File: waypointNav.py
#  Author: Lorenzo Kearns
#  Versions:
#   version: 0.1 10/25/2021 - Initial Program Creation
#   version: 1.0 10/26/2021 - Working map of lynx lake with accurate longitude and lattitude overlay
#   version: 2.0 11/01/2021 - Reconfigure of the program without initial startup map, working navigation map which saves longitude
#                  and lattitude of mouse click. Removed excess code features, renamed variables to be more intuitve, cleaned up code and added comments
#   version: 3.0 11/08/2021 - Fixed GUI so you no longer have to set the boundaries
#  Purpose: GUI for selecting waypoints and displaying desired data from boat sensors.
#*******************************************************************************************************************#
#
#
#****************************************************************#
# Includes:
import io
import time
import serial
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
from matplotlib.transforms import offset_copy
from tkinter.filedialog import askopenfilename
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
#****************************************************************#
#
#
# arduino = serial.Serial(port='COM10', baudrate=115200, timeout=.1)
#***************************************************#
# Beginning of functions living outside the GUI loop
#****************************************************************#
# Function to create the interactive image for the GUI
#****************************************************************#
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
# End of image_spoof
#****************************************************************#
#
#
#****************************************************************#
# Function to create a plot showing user selected waypoints
#****************************************************************#
def plotStuff():
     # Create a Stamen Terrain instance.
    global plot_array_x
    global plot_array_y
    stamen_terrain = cimgt.OSM()
    # Create a GeoAxes in the tile's projection.
    ax = plt.axes(projection=stamen_terrain.crs)
    zoomRatio = 0.005
    waypoint_num = 1
    # Limit the extent of the map to a small longitude/latitude range.
    ax.set_extent([-112.388989, -112.382, 34.5150517, 34.5230208])
    # ax.set_extent([-55, -45, 40, 50])
    # Add the Stamen data at zoom level 8.
    scale = np.ceil(-np.sqrt(2)*np.log(np.divide(zoomRatio,350.0))) # empirical solve for scale based on zoom
    scale = (scale<20) and scale or 19 # scale cannot be larger than 19
    ax.add_image(stamen_terrain, int(scale))
    # plot the line the boat would ideally take
    plt.title("Waypoint tracked path")
    plt.plot(plot_array_x,plot_array_y, 'r-', marker='o', color='black', markersize=1, transform=ccrs.Geodetic())
    plt.plot(plot_array_x[4], plot_array_y[4], marker='x', color='red', markersize=7,
                 alpha=0.7, transform=ccrs.Geodetic())
    geodetic_transform = ccrs.Geodetic()._as_mpl_transform(ax)
    text_transform = offset_copy(geodetic_transform, units='dots', x=-25)
    plt.text(plot_array_x[4], plot_array_y[4], u'Major Waypoint',
                 verticalalignment='top', horizontalalignment='left',
                 transform=text_transform,
                 bbox=dict(facecolor='sandybrown', alpha=0.5, boxstyle='round'))
    # show waypoints
    for i in range(4):
        plt.plot(plot_array_x[i], plot_array_y[i], marker='x', color='red', markersize=4,
        alpha=0.7, transform=ccrs.Geodetic())
        geodetic_transform = ccrs.Geodetic()._as_mpl_transform(ax)
        text_transform = offset_copy(geodetic_transform, units='dots', x=-25)

        # Add text 25 pixels to the left of the volcano.
        plt.text(plot_array_x[i], plot_array_y[i], u'Waypoint %d'%(waypoint_num),
        verticalalignment='center', horizontalalignment='right',
        transform=text_transform,
        bbox=dict(facecolor='sandybrown', alpha=0.5, boxstyle='round'))
        waypoint_num += 1
    plt.show()
    plt.gcf().canvas.draw()
    fig = plt.figure()
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.get_tk_widget().grid(row=1,column=24)
    canvas.draw()
# End of plotStuff
#****************************************************************#
#
#
#
#**********************************************************************************************#
# Define the main functionality of the program
#**********************************************************************************************#
def main():
    global lock
    global plot_array_x
    global plot_array_y
    global setWay
    global cnt
    cnt = 0
    setWay = False
    plot_array_x = np.zeros(5)
    plot_array_y = np.zeros(5)
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
    # Determine the origin
    def getorigin(eventorigin):
        tk.messagebox.showinfo("Startup","Not setup yet, spam click till you see the all good message")
        global x0,y0
        x0 = 34
        y0 = 609
        print(x0,y0)
        controlCenter.bind("<Button 1>",getextentx)
    # mouseclick event
    controlCenter.bind("<Button 1>",getorigin)
    # Determine the extent of the figure in the x direction (Longitude)
    def getextentx(eventextentx):
        global xe
        xe = 453
        print(xe)
        controlCenter.bind("<Button 1>",getextenty)
    # Determine the extent of the figure in the y direction (Lattitude)
    def getextenty(eventextenty):
        global ye
        ye = 33
        print(ye)
        tk.messagebox.showinfo("All good!", "The grid is all good. Start Navigating")
        controlCenter.bind("<Button 1>",printcoords)

    # print the coords to the console
    def printcoords(event):
        global lock
        global plot_array_x
        global plot_array_y
        global setWay
        global cnt
        if(setWay == True):
            cnt += 1
            if ((cnt-1) < 5):
                    xDelta = xe-x0
                    xm = xmLon/xDelta
                    yDelta = ye-y0
                    ym = -ymLat/yDelta
                    # Perform coordinate transformation to normalize results
                    lonWaypoint = (event.x-x0)*(xm)+xInitial
                    latWaypoint = (event.y-y0)*(ym)+yInitial
                    plot_array_y[cnt-1] = latWaypoint
                    plot_array_x[cnt-1] = lonWaypoint
                    longitude = tk.Label(controlCenter, text = "Waypoint " + str(cnt) + " stored ").place(x = 500, y = 310)
                    if(cnt == 5):
                        longitude = tk.Label(controlCenter, text = "Waypoint setting complete, click one more time to finalize").place(x = 500, y = 310)
            else:
                cnt = 0
                setWay = False
                longitude = tk.Label(controlCenter, text = "Waypoint 1 at  Lon: " + str(plot_array_x[0]) + ", Lat: " + str(plot_array_y[0]) + "              ").place(x = 500, y = 310)
                longitude = tk.Label(controlCenter, text = "Waypoint 2 at  Lon: " + str(plot_array_x[1]) + ", Lat: " + str(plot_array_y[1]) + "              ").place(x = 500, y = 330)
                longitude = tk.Label(controlCenter, text = "Waypoint 3 at  Lon: " + str(plot_array_x[2]) + ", Lat: " + str(plot_array_y[2]) + "              ").place(x = 500, y = 350)
                longitude = tk.Label(controlCenter, text = "Waypoint 4 at  Lon: " + str(plot_array_x[3]) + ", Lat: " + str(plot_array_y[3]) + "              ").place(x = 500, y = 370)
                longitude = tk.Label(controlCenter, text = "Waypoint 5 at  Lon: " + str(plot_array_x[4]) + ", Lat: " + str(plot_array_y[4]) + "              ").place(x = 500, y = 390)
            print (plot_array_x, plot_array_y)
        else:
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

    def grab_five_waypoints():
        global setWay
        if (setWay == False):
            setWay = True
        elif (setWay == True):
            setWay = False

    def cycleLock():
        global lock
        if (lock == False):
            lock = True
        elif (lock == True):
            lock = False
        # make a little button boi, he says "Place waypoint" and when clicked allows you to..... Place a waypoint

    def write_gps_to_serial():
        arduino.flush()
        arduino.write(bytes(x, 'utf-8'))
        time.sleep(0.05)

    def send_serial():
        write_read(str(plot_array_x[i]))
        data = arduino.readline().decode('utf-8').rstrip()
        print(data) # printing the value
        # print(float(value))
    tk.Button(controlCenter, text='Get Lon/Lat', command = cycleLock).place(x = 460, y = 30)
    tk.Button(controlCenter, text='Show real Plot', command = plotStuff). place(x = 460, y = 60)
    tk.Button(controlCenter, text='Set Boat Waypoint', command = grab_five_waypoints).place(x = 460, y = 90)
    tk.Button(controlCenter, text='Send Serial', command = write_gps_to_serial).place(x = 460, y = 120)


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



#**********************************************************************************************#
# code Graveyard :
#**********************************************************************************************#

    # cimgt.OSM.get_image = image_spoof # reformat web request for street map spoofing
    # map = cimgt.OSM()
    # fig = plt.figure(figsize = (8, 6), dpi = 100)
    # ax1 = plt.axes(projection = map.crs)
    # lakeCenter = [34.52, -112.386]
    # zoomRatio = 0.005
    # extent = [lakeCenter[1]-(zoomRatio*0.6),lakeCenter[1]+(zoomRatio*0.8),lakeCenter[0]-(zoomRatio),lakeCenter[0]+(zoomRatio*0.6)] # adjust to zoom
    # ax1.set_extent(extent) # set extents
    # scale = np.ceil(-np.sqrt(2)*np.log(np.divide(zoomRatio,350.0))) # empirical solve for scale based on zoom
    # scale = (scale<20) and scale or 19 # scale cannot be larger than 19
    # ax1.add_image(map, int(scale)) # add OSM with zoom specification
    # x = [-112.386 , -112.384]
    # y = [34.515, 34.52]
    # ax1.plot(x, y, 'rx-',label='Waypoint 1', linewidth = 3)
    # ax1.legend()
    # plt.show() # show the plot



    # plot_array_x = np.zeros(5)
    # plot_array_y = np.zeros(5)
    # line graphing for 1 waypoint
    # for i in range(5):
    #     # plt.plot(waypoint_long, waypoint_lat, marker='x', color='red', markersize=4,
    #     #          alpha=0.7, transform=ccrs.Geodetic())
    #     plot_array_y[i] = waypoint_lat
    #     plot_array_x[i] = waypoint_long
    #     if (waypoint_long == -112.3856):
    #         waypoint_long += np.random.rand(0.0012)
    #     else:
    #         waypoint_long -= 0.0008
    #     waypoint_lat += 0.0014



    # waypoint_long = -112.3856
    # waypoint_lat = 34.516
    # tack_right = 0
    # multiple waypoint setting
    # for i in range(5):
    #     plt.plot(waypoint_long, waypoint_lat, marker='x', color='red', markersize=4,
    #              alpha=0.7, transform=ccrs.Geodetic())
    #     geodetic_transform = ccrs.Geodetic()._as_mpl_transform(ax)
    #     text_transform = offset_copy(geodetic_transform, units='dots', x=-25)
    #
    #     # Add text 25 pixels to the left of the volcano.
    #     plt.text(waypoint_long, waypoint_lat, u'Waypoint %d'%(waypoint_num),
    #              verticalalignment='center', horizontalalignment='right',
    #              transform=text_transform,
    #              bbox=dict(facecolor='sandybrown', alpha=0.5, boxstyle='round'))
    #     num_offset = np.random.random()*(0.001-0.0005) + 0.0005
    #     if (tack_right == 0):
    #         waypoint_long += num_offset
    #         tack_right = 1
    #     else:
    #         waypoint_long -= num_offset
    #         tack_right = 0
    #     waypoint_lat += np.random.random()*(0.0014-0.0008) + 0.0008
    #     waypoint_num += 1
