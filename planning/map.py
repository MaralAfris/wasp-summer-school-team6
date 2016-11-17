#!/usr/bin/env python

import settings

import yaml, sys
from scipy.misc import imread

from copy import deepcopy
from enum import Enum
import heapq

import numpy as np
import scipy.ndimage as ndimage

from astar import GridWithWeights, a_star_search

from numpy.linalg import norm
from scipy.spatial import Delaunay
from scipy.spatial import Voronoi
from scipy.spatial import ConvexHull

# TODO look at all uses of translation and use center/round if applicable
# TODO use http://blancosilva.github.io/post/2014/10/28/Computational-Geometry-in-Python.html
# TODO prune based on angle
class Map(object):

    """This class is used to calculate a mesh from occupancy grid and obtain 
    good paths to use for the planner.

    Attributes:
        occupancy_grid (OccupancyGrid): occupancy grid from turtlebot scan
        points (list): list containing the initial set of Point
    """
    def __init__(self, occupancy_grid, points):
        self.occupancy_grid = occupancy_grid
        self.points = points

        # create occupancy with turtlebot sized grid (35cm)
        fact = int(round(0.35/self.occupancy_grid.meta['resolution']))
        self.tb_grid = self.occupancy_grid.downsample_grid(fact)

        # check initial points for consistency
        for point in self.points:
            px = occupancy_grid.coords_to_pixel(point.p)
            pxtb = self.tb_grid.coords_to_pixel(point.p)
            reset = False
            if occupancy_grid.grid[px[0], px[1]] < occupancy_grid.free_thresh():
                print('warning: initial point ' + str(point) + ' is occupied, relocating')
                reset = True
            elif self.tb_grid.grid[pxtb[0], pxtb[1]] < self.tb_grid.free_thresh():
                print('warning: initial point ' + str(point) + ' does not fit a turtlebot, relocating')
                reset = True
            if reset:
                repoint = self.tb_grid.closest_free(point.p)
                point.p = repoint

        # create optimistic mesh
        self._split_triangles(set())
        self.triangles = self._delaunay_triangulate()
        
        self.edges = []
        edge_set = set()
        # create edges
        for tri in self.triangles.simplices:
            a = self.points[tri[0]]
            b = self.points[tri[1]]
            c = self.points[tri[2]]
            def _add_edge(edge, point):
                if edge in edge_set:
                    return
                # check angles in triangle
                a1 = edge.p1.dist(point)
                b1 = edge.p2.dist(point)
                c1 = edge.p1.dist(edge.p2)
                edge_set.add(edge)
                self.edges.append(edge)
            _add_edge(Edge(a,b), c)
            _add_edge(Edge(a,c), b)
            _add_edge(Edge(b,c), a)

        # pruning on regions
        for e in self.edges:
            # check occupied
            px1 = self.tb_grid.coords_to_pixel(e.p1.p)
            px2 = self.tb_grid.coords_to_pixel(e.p2.p)
            if self.tb_grid.grid[px1[0], px1[1]] < self.tb_grid.occupied_thresh() or \
                    self.tb_grid.grid[px2[0], px2[1]] < self.tb_grid.occupied_thresh():
                e.pruned = Pruned.access
                continue

            came_from, cost_so_far = self.tb_grid.shortest_path(e.p1.p, e.p2.p)
            px1 = self.tb_grid.coords_to_pixel(e.p1.p)
            px2 = self.tb_grid.coords_to_pixel(e.p2.p)
            if px2 not in cost_so_far:
                e.pruned = Pruned.access
            self._prune_path(came_from, cost_so_far, e)
            #elif len(self._regions_for_path(came_from, cost_so_far, e)) > 3:
            #curr = came_from[px2]
            #currp = self.tb_grid.pixel_to_coords(px2)
            #xs = [currp[0]+self.tb_grid.meta['resolution']/2]
            #ys = [currp[1]+self.tb_grid.meta['resolution']/2]
            #import matplotlib.pyplot as plt
            #while curr is not None:
                #currp = self.tb_grid.pixel_to_coords(curr)
                #xs.append(currp[0]+self.tb_grid.meta['resolution']/2)
                #ys.append(currp[1]+self.tb_grid.meta['resolution']/2)
                #curr = came_from[curr]
            #xs.append(e.p1.p[0])
            #ys.append(e.p1.p[1])
            #plt.plot(xs,ys)
            #e.pruned = Pruned.regions
            #print e.p1,e.p2
            #print self._regions_for_path(came_from, cost_so_far, e)

        # pruning on useless
        #for e in self.edges:

    def _delaunay_triangulate(self):
        array = []
        for point in self.points:
            array.append([point.p[0], point.p[1]])
        points = np.array(array)
        return Delaunay(points)

    def _prune_path(self, came_from, cost_so_far, edge):
        if edge.pruned != Pruned.none:
            return set()

        start = self.tb_grid.coords_to_pixel(edge.p1.p)
        end = self.tb_grid.coords_to_pixel(edge.p2.p)

        def _region(p):
            min_dist = np.inf
            min_region_i = -1
            for i in np.arange(0, len(self.edges)):
                if self.edges[i].pruned != Pruned.none:
                    continue
                dist = self.edges[i].dist(p)
                if dist < min_dist:
                    min_dist = dist
                    min_region_i = i
            return min_region_i

        allowed = set()
        allowed.add(-1)
        allowed.add(_region(edge.p1.p))
        allowed.add(_region(edge.p2.p))
        print
        print "-----------------------------"
        print "edge:",edge.p1,edge.p2
        for neighbor in edge.neighbors(self):
            print neighbor.p1.p, neighbor.p2.p, _region(neighbor.p1.p), _region(neighbor.p2.p)
            allowed.add(_region(neighbor.p1.p))
            allowed.add(_region(neighbor.p2.p))

        not_allowed = set()
        curr = came_from[end]
        while curr is not None:
            currp = self.tb_grid.pixel_to_coords(curr, center = True)
            r = _region(currp)
            if r not in allowed:
                not_allowed.add(r)
                print "not_allowed", r, self.edges[r].p1, self.edges[r].p2

            curr = came_from[curr]
        if len(not_allowed) > 0:
            print "neighbors", len(edge.neighbors(self))
            print "pruned not allowed"
            print not_allowed
            print allowed
            print edge.p1,_region(edge.p1.p)
            print edge.p2,_region(edge.p2.p)
            for r in allowed:
                if r >= 0:
                    print "allowed", r, self.edges[r].p1, self.edges[r].p2
            edge.pruned = Pruned.regions

    def _split_triangles(self, names):
        tri = self._delaunay_triangulate()
        any_split = False
        for triangle in tri.simplices:
            a = self.points[triangle[0]]
            b = self.points[triangle[1]]
            c = self.points[triangle[2]]
            if self._split_triangle(a, b, c, names):
                any_split = True
        if any_split:
            self._split_triangles(names)

    def _split_triangle(self, a, b, c, names):
        x = (a.p[0] + b.p[0] + c.p[0]) / 3
        y = (a.p[1] + b.p[1] + c.p[1]) / 3
        split_thresh = settings.mesh['split_thresh']
        grid = 10 / split_thresh
        name = str(int(x*grid)) + ' ' + str(int(y*grid))
        p = self.tb_grid.closest_free((x, y))
        if (p is None):
            return False
        centroid = Point.p(p, PointType.mesh)
        if (centroid is not None and
                name not in names and 
                a.dist(centroid) > split_thresh and
                b.dist(centroid) > split_thresh  and
                c.dist(centroid) > split_thresh):
            self.points.append(centroid)
            names.add(name)
            return True
        return False

class OccupancyGrid(object):

    """This class is used to calculate a mesh from occupancy grid and obtain 
    good paths to use for the planner.

    Attributes:
        grid (numpy.ndarray): occupancy grid from turtlebot scan
        meta (dict): information about resolution, origin, and thresholds
    """
    def __init__(self, grid, meta):
        self.grid = grid
        self.meta = meta
        
    @classmethod
    def from_pgm(cls, base_file_name):
        with open(base_file_name + '.yaml') as stream:
            meta = yaml.load(stream)
    
        with open(base_file_name + '.pgm') as stream:
            grid = imread(stream)
    
        return cls(grid, meta)

    def free_thresh(self):
        return (1-self.meta['free_thresh'])*255

    def occupied_thresh(self):
        return (1-self.meta['occupied_thresh'])*255

    def shortest_path(self, p1, p2):
        """Calculates shortest path from p1 to p2
        Uses a_star 

        Returns:
            dist_matrix (): Lookup of costs from a,b
            came_from (): Use to iterate shortest path by prev index
        """
        x,y = self.grid.shape
        if not hasattr(self, 'astar'):
            self.astar = GridWithWeights(x, y)
            for i in range(0, x):
                for j in range(0, y):
                    if self.grid[i,j] < self.free_thresh():
                        self.astar.walls.append((i,j))

        px1 = self.coords_to_pixel(p1)
        px2 = self.coords_to_pixel(p2)
        return a_star_search(self.astar, px1, px2)

    def closest_free(self, point, max_dist = 1000):
        sx,sy = self.grid.shape
        px = self.coords_to_pixel(point)
        h = []
        heapq.heappush(h, (0, px))
        ixs = [[1,0], [0,1], [-1,0], [0,-1]]
        for i in np.arange(0, max_dist):
            if not h:
                return None
            p = heapq.heappop(h)[1]
            if self.grid[p[0], p[1]] > self.free_thresh():
                return self.pixel_to_coords(p, center=True)
            for cand in map(lambda x:[x[0]+p[0],p[1]+p[1]], ixs):
                if cand[0]<sx and cand[1]<sy:
                    dist = (point[0] - cand[0])**2 + (point[1] - cand[1])**2
                    heapq.heappush(h, (dist, cand))
        return None
        
    def downsample_grid(self, fact):
        meta = deepcopy(self.meta)
        meta['resolution'] = meta['resolution'] * fact
        res = meta['resolution']

        x, y = self.grid.shape
        sx, sy = (int(round(x/float(fact))), int(round(y/float(fact))))
        img = np.zeros((sx,sy))

        # threshold for how many pixels should be occupied for subpixel to be occupied
        occpixels = fact**2 * 0.05

        for xi in np.arange(0, sx):
            for yi in np.arange(0, sy):
                arr = self.grid[xi*fact:(xi+1)*fact, yi*fact:(yi+1)*fact]
                free = np.where(arr > self.free_thresh())
                occ = np.where(arr < self.occupied_thresh())
                if occ[0].shape[0] > occpixels:
                    img[xi,yi] = 0
                elif free[0].shape[0] > 0:
                    img[xi,yi] = 255
                else:
                    img[xi,yi] = (self.free_thresh()+self.occupied_thresh())/2

        return OccupancyGrid(img, meta)

    def pixel_to_coords(self, px, center = False):
        orig = self.meta['origin']
        res = self.meta['resolution']
        i_max = self.grid.shape[0]
        j_max = self.grid.shape[1]
        x_min = orig[0]
        x_max = i_max*res+orig[0]
        y_min = orig[1]
        y_max = j_max*res+orig[1]
        x = self._translate(px[0], 0, i_max, x_min, x_max)
        y = self._translate(px[1], 0, j_max, y_min, y_max)
        if px[0] > i_max or px[1] > j_max:
            raise Exception('px [' + str(px[0]) + ',' + str(px[1]) + '] out of bounds')
        if center == True:
            x += res/2
            y += res/2
        return (x,y)

    def coords_to_pixel(self, p):
        orig = self.meta['origin']
        res = self.meta['resolution']
        i_max = self.grid.shape[0]
        j_max = self.grid.shape[1]
        x_min = orig[0]
        x_max = i_max*res+orig[0]
        y_min = orig[1]
        y_max = j_max*res+orig[1]
        p1 = self._translate(p[0], x_min, x_max, 0, i_max)
        p2 = self._translate(p[1], y_min, y_max, 0, j_max)
        if p[0] > x_max or p[1] > y_max:
            raise Exception('p [' + str(p[0]) + ',' + str(p[1]) + '] out of bounds')
        return (int(p1), int(p2))

    def _translate(self, src, src_min, src_max, res_min, res_max):
        return float((src - src_min)) / \
                float((src_max - src_min)) * \
                float((res_max - res_min)) + res_min


class Edge(object):
    """Represents an undirected path between two points

    Attributes:
        p1 (Point): first point
        p2 (Point): second point
        pruned (Pruned): whether it should be used or not
    """

    def __init__(self, p1, p2):
        assert isinstance(p1, Point)
        assert isinstance(p2, Point)
        self.p1 = p1
        self.p2 = p2
        self.pruned = Pruned.none

    def dist(self, point):
        assert isinstance(point, tuple)
        a = point[0] - self.p1.p[0]
        b = point[1] - self.p1.p[1]
        c = self.p2.p[0] - self.p1.p[0]
        d = self.p2.p[1] - self.p1.p[1]
        dot = a*c + b*d
        len_sq = c*c + d*d
        param = -1
        if len_sq != 0:
            param = dot / len_sq
        if param < 0:
            xx = self.p1.p[0]
            yy = self.p1.p[1]
        elif param > 1:
            xx = self.p2.p[0]
            yy = self.p2.p[1]
        else:
            xx = self.p1.p[0] + param*c
            yy = self.p1.p[1] + param*d
        dx = point[0] - xx
        dy = point[1] - yy
        dist = np.sqrt(dx*dx + dy*dy)
        return dist

    def translate_to_pixel(self, grid):
        p1px = grid.coords_to_pixel(self.p1.p)
        p2px = grid.coords_to_pixel(self.p2.p)
        return Edge(Point.p(p1px, self.p1.type), Point.p(p2px, self.p2.type))

    def neighbors(self, map):
        neighbors = []
        for edge in map.edges:
            if self.p1 == edge.p1 or self.p1 == edge.p2 or \
                    self.p2 == edge.p1 or self.p2 == edge.p2:
                neighbors.append(edge)
        return neighbors

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return other.p1 == self.p1 and other.p2 == self.p2 or \
                    other.p1 == self.p2 and other.p2 == self.p1
        return NotImplemented

    def __hash__(self):
        return hash(self.p1) ^ hash(self.p2)

class Point(object):
    def __init__(self, x, y, type):
        self.p = (float(x), float(y))
        self.type = type

    @classmethod
    def p(cls, p, type):
        return cls(p[0], p[1], type)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return other.p[0] == self.p[0] and other.p[1] == self.p[1]
        return NotImplemented

    def __hash__(self):
        return hash(self.p)

    def dist(self, other):
        return np.sqrt((self.p[0] - other.p[0])**2 + (self.p[1] - other.p[1])**2)

    def __str__(self):
        return '(' + '{:.3f}'.format(self.p[0]) + ',' + '{:.3f}'.format(self.p[1]) + ')'


class Pruned(Enum):
    none = 1
    access = 2
    regions = 3

class PointType(Enum):
    initial = 1
    mesh = 2
        
if __name__ == "__main__":
    grid = OccupancyGrid.from_pgm('../../data/mapToG2')
    tb_grid = grid.downsample_grid(7)
    p1 = Point(-1.51, -2.63, PointType.initial)
    p2 = Point(-1.53, -1.45, PointType.initial)
    tb_grid.plot_shortest_path(p1.p, p2.p)
