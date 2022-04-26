from math import *
import math
import numpy as np

class Tools():
    def rotate(self, x1, y1, a1, x0, y0):
        """
            Rotate an object for tkinter
        """
        b = self.deg2rad(a1)
        return x0 + (x1 - x0)*cos(b) - (y1 - y0)*sin(b), \
                y0 + (y1 - y0)*cos(b) + (x1 - x0)*sin(b)

    def ft_to_latlon(self, x):
        """
            Convert a value of feet into a comprable
            value in lat or longitude difference
        """
        x = x/364000
        return x

    def mod360(self,a):
        """
            return the mod360 value of an input
            to normalize the variable
        """
        while a > 360:
            a -= 360
        while a < 0:
            a += 360
        return a

    def rad2deg(self, x):
        """
            convert radians to degrees
        """
        return float(x) / pi * 180

    def deg2rad(self, x):
        """
            convert degrees to radians
        """
        return float(x) / 180 * pi

    def random_shifted_pt(self, pt, mul):
        """
            randomly shifts a point, idk
        """
        pt = np.array(pt)
        rand = np.random.random(pt.shape) - 0.5
        rand *= np.array(mul)
        return pt + rand

    def direction_lookup(self, lat1, lng1, lat2, lng2):
        lat1 = self.deg2rad(lat1)
        lng1 = self.deg2rad(lng1)
        lat2 = self.deg2rad(lat2)
        lng2 = self.deg2rad(lng2)
        angle = atan2(sin(lng2 - lng1) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lng2 - lng1))
        return self.mod360(self.rad2deg(angle))

    def sq(self, x):
        return x*x

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
        return 2 * 6371000.0 * atan2(sqrt(hav), sqrt(1-hav))
