import copy
import math
import json
import sys
import numpy as np
from collections import deque
from ..utils import remap, to_degrees, euclidean_dist

def neighbors(x,y):
    return [(x+1, y), (x-1,y),
            (x,y+1), (x,y-1)]

class GridMap:
    """A grid map is a collection of locations, walls, and some locations have obstacles.
    The coordinate convention differs from THOR. Horizontal axis is x, vertical axis is y.

    Note that the coordinates in grid map starts from (0,0) to (width-1,length-1).
    This differs from THOR which could have negative coordinates

    The purpose of this grid map is to go from the floating-point coordinates in
    the discrete map in Thor to integer indices.
    """

    def __init__(self, width, length, obstacles, name="grid_map",
                 ranges_in_thor=None):
        """
        xpos (list): list of x coordinates for free cells
        ypos (list): list of y coordinates for free cells
        obstacles (set): a set of locations for the obstacles
        ranges_in_thor (tuple): A tuple (thor_gx_range, thor_gy_range)
            where thor_gx_range are the min max range for grid x coordiates in thor
            where thor_gy_range are the min max range for grid y coordiates in thor
            Note that thor grid coordinates do not originate in (0,0) but at some other
                point determined by Unity.
        """
        self.width = width
        self.length = length

        self.obstacles = obstacles
        self.free_locations = {(x,y) for x in range(width)
                               for y in range(length)
                               if (x,y) not in obstacles}

        self.name = name
        self.ranges_in_thor = ranges_in_thor

        # Caches the computations
        self._geodesic_dist_cache = {}
        self._blocked_cache = {}

    def to_thor_pose(self, x, y, th, grid_size=None):
        return (*self.to_thor_pos(x, y, grid_size=grid_size), to_degrees(th))

    def to_thor_pos(self, x, y, grid_size=None):
        """
        Given a point (x,y) in the grid map, convert it to (x,z) in
        the THOR coordinte system (grid size is accounted for).
        If grid_size is None, will return the integers
        for the corresponding coordinate.
        """
        # Note that y is z in Unity
        thor_gx_min, thor_gx_max = self.ranges_in_thor[0]
        thor_gy_min, thor_gy_max = self.ranges_in_thor[1]
        thor_gx = remap(x, 0, self.width, thor_gx_min, thor_gx_max)
        thor_gy = remap(y, 0, self.length, thor_gy_min, thor_gy_max)
        if grid_size is not None:
            # Snap to grid
            return (grid_size * round((thor_gx * grid_size) / grid_size),
                    grid_size * round((thor_gy * grid_size) / grid_size))
        else:
            return (thor_gx, thor_gy)

    def to_grid_pos(self, thor_x, thor_z, grid_size=None, avoid_obstacle=False):
        """
        Convert thor location to grid map location. If grid_size is specified,
        then will regard thor_x, thor_z as the original Unity coordinates.
        If not, then will regard them as grid indices but with origin not at (0,0).
        """
        if grid_size is not None:
            thor_gx = int(round(thor_x / grid_size))
            thor_gy = int(round(thor_z / grid_size))
        else:
            thor_gx = thor_x
            thor_gy = thor_z

        # remap coordinates to be nonnegative (origin AT (0,0))
        thor_gx_min, thor_gx_max = self.ranges_in_thor[0]
        thor_gy_min, thor_gy_max = self.ranges_in_thor[1]
        gx = int(remap(thor_gx, thor_gx_min, thor_gx_max, 0, self.width, enforce=True))
        gy = int(remap(thor_gy, thor_gy_min, thor_gy_max, 0, self.length, enforce=True))
        if avoid_obstacle and (gx, gy) not in self.free_locations:
            return self.closest_free_cell((gx, gy))
        else:
            return gx, gy

    def free_region(self, x, y):
        """Given (x,y) location, return a set of locations
        that are free and connected to (x,y)"""
        region = set()
        q = deque()
        q.append((x,y))
        visited = set()
        while len(q) > 0:
            loc = q.popleft()
            region.add(loc)
            for nb_loc in neighbors(*loc):
                if nb_loc in self.free_locations:
                    if nb_loc not in visited:
                        visited.add(nb_loc)
                        q.append(nb_loc)
        return region

    def boundary_cells(self, thickness=1):
        """
        Returns a set of locations corresponding to
        obstacles that lie between free space and occluded spaces.
        These are usually locations where objects are placed.
        """
        last_boundary = set()
        for i in range(thickness):
            boundary = set()
            for x, y in self.obstacles:
                for nx, ny in neighbors(x, y):
                    if (nx, ny) in self.free_locations\
                       or (nx, ny) in last_boundary:
                        boundary.add((x,y))
                        break
            last_boundary.update(boundary)
        return last_boundary

    def closest_free_cell(self, loc):
        """Snaps given loc (x,y) to the closest grid cell"""
        return min(self.free_locations,
                   key=lambda l: euclidean_dist(l, loc))

    def shortest_path(self, gloc1, gloc2):
        """
        Computes the shortest distance between two locations.
        The two locations will be snapped to the closest free cell.
        """
        def get_path(s, t, prev):
            v = t
            path = [t]
            while v != s:
                v = prev[v]
                path.append(v)
            return path

        # BFS; because no edge weight
        visited = set()
        q = deque()
        q.append(gloc1)
        prev = {gloc1:None}
        while len(q) > 0:
            loc = q.popleft()
            if loc == gloc2:
                return get_path(gloc1, gloc2, prev)
            for nb_loc in neighbors(*loc):
                if nb_loc in self.free_locations:
                    if nb_loc not in visited:
                        q.append(nb_loc)
                        visited.add(nb_loc)
                        prev[nb_loc] = loc
        return None

    def geodesic_distance(self, loc1, loc2):
        """Reference: https://arxiv.org/pdf/1807.06757.pdf
        The geodesic distance is the shortest path distance
        in the environment.

        Geodesic distance: the distance between two vertices
        in a graph is the number of edges in a shortest path.

        NOTE: This is NOT the real geodesic distance in
        the THOR environment, but an approximation for
        POMDP agent's behavior. The Unit here is No.GridCells

        This is computed by first snapping loc1, loc2
        to the closest free grid cell then find the
        shortest path on the grid between them.

        Args:
           loc1, loc2 (tuple) grid map coordinates
        """
        _key = tuple(loc1), tuple(loc2)
        if _key in self._geodesic_dist_cache:
            return self._geodesic_dist_cache[_key]
        else:
            path = self.shortest_path(loc1, loc2)
            if path is not None:
                dist = len(path)
            else:
                dist = float("inf")
            self._geodesic_dist_cache[_key] = dist
            return dist

    def blocked(self, loc1, loc2, nsteps=40):
        """
        Returns True if:
        - loc1 is not free,
        OR
        - loc1 is a reachable location AND
        - the line segment between loc1 and loc2 goes through
          an obstacle, and then goes through a free cell
          (i.e. blocked by an obstacle)

        This is checked by simulating a straightline from loc1 to loc2 and check
        if any step on the line is at an obstacle.
        """
        if loc1 == loc2:
            return False

        if loc1 not in self.free_locations:
            return True

        _key = tuple(loc1), tuple(loc2)
        if _key in self._blocked_cache:
            return self._blocked_cache[_key]

        # vec = np.array([px - rx, py - ry]).astype(float)
        # vec /= np.linalg.norm(vec)
        loc1 = np.asarray(loc1)
        x1, y1 = loc1
        x2, y2 = loc2
        vec = np.array([x2 - x1, y2 - y1]).astype(float)
        vec /= np.linalg.norm(vec)

        # Check points along the line from robot pose to the point
        status = "start"
        dist = euclidean_dist(loc1, loc2)
        step_size = dist / nsteps
        t = 0
        while t < nsteps:
            line_point = tuple(np.round(loc1 + (t*step_size*vec)).astype(int))
            if line_point in self.obstacles:
                status = "hits_obstacle"
            elif line_point in self.free_locations:
                if status == "hits_obstacle":
                    status = "blocked"
                    break
            t += 1
        result = status == "blocked"
        self._blocked_cache[_key] = result
        return result
