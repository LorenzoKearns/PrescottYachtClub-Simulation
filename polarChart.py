import hrosailing.polardiagram as pol
import hrosailing.pipeline as pipe
import hrosailing.cruising as sail
import hrosailing.pipelinecomponents as pcomp
import numpy as np
import matplotlib.pyplot as plt
import re

def random_shifted_pt(pt, mul):
    pt = np.array(pt)
    rand = np.random.random(pt.shape) - 0.5
    rand *= np.array(mul)
    return pt + rand

class Polar():
    def __init__(self):
        self.pd = pol.from_csv("testdata.csv", fmt="hro").symmetrize()
        data = np.array([
            random_shifted_pt([ws, wa, self.pd.boat_speeds[i, j]], [10, 5, 2])
            for i, ws in enumerate(self.pd.wind_angles)
            for j, wa in enumerate(self.pd.wind_speeds)
            for _ in range(6)
        ])
        self.data = data[np.random.choice(len(data), size=500)]

    def create_polar_chart(self):
        pol_pips = [
            pipe.PolarPipeline(
                handler=pcomp.ArrayHandler(),
                extension=pipe.TableExtension()
            ),
            pipe.PolarPipeline(
                handler=pcomp.ArrayHandler(),
                extension=pipe.PointcloudExtension()
            ),
            pipe.PolarPipeline(
                handler=pcomp.ArrayHandler(),
                extension=pipe.CurveExtension()
            )
        ]
        pds = [pol_pip((self.data, ["Wind angle", "Wind speed", "Boat speed"])) for pol_pip in pol_pips]
        for i, self.pd in enumerate(pds):
            self.pd.plot_polar(ws=ws, ax=plt.subplot(1, 3, i+1, projection="polar"))
        plt.tight_layout()
        plt.show()

    def plot_polar(self):
        ws = [6, 8, 10, 12, 14, 16, 20]
        self.pd.plot_polar(ws=ws, ax=plt.subplot(1, 2, 1, projection="polar"))
        self.pd.plot_convex_hull(ws=ws, ax=plt.subplot(1, 2, 2, projection="polar"))
        # self.pd.plot_flat(ws=ws, ax=plt.subplot(2, 2, 3))
        # self.pd.plot_color_gradient(ax=plt.subplot(2, 2, 4))
        # plt.show()
        print(self.pd)
        return self.pd

class Sailing():
    def __init__(self,polar_chart):
        self.pd = polar_chart
    def get_optimal_direction(self, speed, angle):
        heading = sail.convex_direction(self.pd, speed, angle)
        heading_angle, percent_of_trip = [float(s) for s in re.findall(r'-?\d+\.?\d*', str(heading[0]))]
        if (percent_of_trip < 85):
            tack_angle = 360 - heading_angle
        else:
            tack_angle = 0
        print(heading[0])
        return heading_angle, percent_of_trip, tack_angle
        # print("The wind is coming from " + str(angle) + " degrees, at a speed of " + str(speed) +" Knotts") # commented out print statements, use for debug
        # print(heading_angle)
        # print(percent_of_trip)
        # print(tack_angle)

# def main():
#     boat = Polar()
#     p_chrt = boat.plot_polar()
#     sailor = Sailing(p_chrt)
#     sailor.get_optimal_direction(8, 45)
#
# if __name__ == '__main__':
#     main()
