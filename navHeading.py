import hrosailing.cruising as sail

class Helmsman():
    def __init__(self, p_data, start, end):
        self.p_chart = p_data
        self.start = start
        self.end = end

    def update_cruise_vals(self, tws, twa):
        heading_angle_starboard, starboard_distance, heading_angle_port, port_distance, distance =  self.get_cruise()
        self.h_a_star = heading_angle_starboard
        self.star_dist = starboard_distance
        self.h_a_port = heading_angle_port
        self.port_dist = port_distance
        self.dist_tot = distance


    def find_heading(self, tws, twa):
        self.tws = tws
        self.twa = twa
        self.update_cruise_vals(tws, twa)
        return  self.h_a_star

    def get_total_distance(self):
        self.distance = self.dist_tot * 3280.84
        return self.distance

    def get_cruise(self):
        onek = 10000
        print(self.start[0] * onek, self.start[1] * onek, self.end[0] * onek, self.end[1] * onek)
        a1, t1, a2, t2, dist = sail.cruise(self.p_chart , self.tws, self.twa , (self.start[0] * onek, self.start[1] * onek), (self.end[0] * onek, self.end[1] * onek))
        print(a1, t1, a2, t2, dist)
        return a1, t1, a2, t2, dist


# placeholder from cartographer
def convert_to_visual_points(self):
    for i in range(len(self.boundaryArrayLat)):
        lon,lat = self.LatLontoXY(self.boundaryArrayLat[i], self.boundaryArrayLon[i])
        self.bound.append(lon)
        self.bound.append(lat)
        return self.bound
