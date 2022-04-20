from math import *
import numpy as np

class Tools():
    def rot(self, x1, y1, a1, x0, y0):
        """
            Rotate an object
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
