#!/usr/bin/env python

from __future__ import generators

import settings

from copy import *
import heapq
import random

import numpy as np
from scipy.spatial import Delaunay

class Map(object):

    """This class is used to calculate a mesh from occupancy grid and obtain 
    good paths to use for the planner.

    Attributes:
        occupancy_grid (OccupancyGrid): occupancy grid from laser scan
        points (list): list containing the initial set of point (as tuples)
    """
    def __init__(self, occupancy_grid, points, consistency=True):
        assert type(occupancy_grid) == OccupancyGrid
        for p in points:
            assert type(p) == tuple

        print("info: creating mesh")
        self.initial_points = points
        self.occupancy_grid = occupancy_grid
        self.nodes = []
        self.neighbors = []
        self.moved_points = {}
        
        radius = settings.mesh['padding_radius'] / occupancy_grid.res()
        #pad_grid = occupancy_grid
        pad_grid = occupancy_grid.pad_occupied(radius)
        self.pad_grid = pad_grid

        self.pad_grid = pad_grid
        # Make sure initial points are not on occupied cells
        for i, point in enumerate(points):
            if not pad_grid.in_bounds(pad_grid.pixel(point)):
                print('warning: initial point ' + str(point) + ' not in bounds')
                continue
            update = pad_grid.closest_free(point)
            if update is not None and update != point:
                print('warning: initial point ' + str(point) + 
                        ' is occupied, relocating to ' + str(update))
                self.moved_points[point] = update
                points[i] = update
            elif update is None:
                print('warning: initial point ' + str(point) + 
                        ' is occupied, no free point found')

        if len(points) < 3:
            print('warning: not enough waypoints')
            return


        # Next compute a* search for each path on Delaunay triangulation.
        triangles = Delaunay(points)
        nodes = {}       # pixel to coords: tuple->tuple
        neighbors = {}   # pixel to neighbors: tuple->[tuple]
        searched = set() # coords: tuple

        initial_px = set()
        for p in points:
            px = pad_grid.pixel(p)
            nodes[px] = (p[0], p[1])
            neighbors[px] = set()
            initial_px.add(px)

        print("info: searching paths")
        for i,tri in enumerate(triangles.simplices):
            def path(ix1,ix2):
                px1 = pad_grid.pixel(points[ix1])
                px2 = pad_grid.pixel(points[ix2])
                if pad_grid.is_occupied(px1):
                    print('warning: path occupied ' + str(points[ix1]))
                    return
                if pad_grid.is_occupied(px2):
                    print('warning: path occupied ' + str(points[ix2]))
                    return

                # Don't need to do search sometimes because all occur twice 
                if (ix1,ix2) in searched or (ix2,ix1) in searched:
                    return
                searched.add((ix1,ix2))
                start = (px1[0], px1[1])
                goal = (px2[0], px2[1])
                came_from, costs = a_star_search(pad_grid, start, goal)
                if goal not in came_from or start not in came_from:
                    print('warning: initial path not reachable ' + 
                            str(points[ix1]) + ' ' + str(points[ix2]))
                    return
                mypath = reconstruct_path(came_from, start, goal)
                for i in np.arange(0, len(mypath)-1):
                    p1 = mypath[i]
                    p2 = mypath[i+1]
                    if p1 not in nodes:
                        nodes[p1] = pad_grid.coords(p1)
                        neighbors[p1] = set()
                    if p2 not in nodes:
                        nodes[p2] = pad_grid.coords(p2)
                        neighbors[p2] = set()
                    neighbors[p1].add(p2)
                    neighbors[p2].add(p1)
            a = tri[0]
            b = tri[1]
            c = tri[2]
            path(a,b)
            path(a,c)
            path(b,c)

        def check_consistency():
            if consistency:
                for px,p in nodes.iteritems():
                    assert type(px) == tuple
                    assert type(p) == tuple
                    assert len(neighbors[px]) > 0, "unreachable point " + str(p)
                    assert px not in neighbors[px], "self neighbor " + str(p)
                    for nb in neighbors[px]:
                        assert px in neighbors[nb], "not undirected graph " + str(p)

        print("info: pruning graph")

        # this prunes graph by straighten paths and then joining adjacent paths,
        # until conditions on min_dist are met.
        check_consistency()
        while True:
            pre_len = len(nodes)
            self.straighten_paths(nodes, neighbors, initial_px)
            self.join_adjacent(pad_grid, nodes, neighbors, initial_px)
            if pre_len == len(nodes):
                break
            check_consistency()

        # remove any errors caused by pruning
        self.remove_overlapping(nodes, neighbors)
        check_consistency()

        ix = 0
        index = {}
        for ix,node in nodes.iteritems():
            index[ix] = len(self.nodes)
            self.nodes.append(node)
        for ix,ineighbors in neighbors.iteritems():
            jx = index[ix]
            nbs = []
            self.neighbors.insert(jx, nbs)
            for nb in ineighbors:
                nbs.append(index[nb])

    def straighten_paths(self, nodes, neighbors, initial_px, only_straight = False):
        for px in deepcopy(nodes):
            if px in neighbors and len(neighbors[px]) < 3 and px not in initial_px:
                if len(neighbors[px]) == 2:
                    aslist = list(neighbors[px])
                    px1 = aslist[0]
                    px2 = aslist[1]
                    if only_straight:
                        if px1[0] != px2[0] and px1[1] != px2[1]:
                            continue
                    neighbors[px1].remove(px)
                    neighbors[px2].remove(px)
                    neighbors[px1].add(px2)
                    neighbors[px2].add(px1)
                elif len(neighbors[px]) == 1:
                    p1 = neighbors[px].pop()
                    neighbors[p1].remove(px)

                del neighbors[px]
                del nodes[px]

    def join_adjacent(self, grid, nodes, neighbors, initial_px):
        def closestpair():
            dist = np.inf
            pair = (None, None)
            for px in nodes:
                p = nodes[px]
                for nb in neighbors[px]:
                    nbp = nodes[nb]
                    d = np.hypot(p[0]-nbp[0], p[1]-nbp[1])
                    if px in initial_px and nb in initial_px:
                        continue
                    if d < dist:
                        dist = d
                        pair = (px,nb)
            return (dist, pair[0], pair[1])

        while True:
            dist, px1, px2 = closestpair()
            if px1 is None or dist > settings.mesh['mesh_min_dist']:
                return

            p1 = nodes[px1]
            p2 = nodes[px2]
            if dist > settings.mesh['mesh_min_dist']:
                return

            def join(jpx1, jpx2):
                for neighbor in neighbors[jpx2]:
                    if neighbor != jpx1:
                        neighbors[neighbor].remove(jpx2)
                        neighbors[neighbor].add(jpx1)
                neighbors[jpx1] |= neighbors[jpx2]
                if jpx1 in neighbors[jpx1]:
                    neighbors[jpx1].remove(jpx1)
                if jpx2 in neighbors[jpx1]:
                    neighbors[jpx1].remove(jpx2)
                del neighbors[jpx2]
                del nodes[jpx2]

            if px1 in initial_px:
                join(px1, px2)
            elif px2 in initial_px:
                join(px2, px1)
            else:
                p1 = nodes[px1]
                p2 = nodes[px2]
                centroid = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
                if np.hypot(p1[0] - centroid[0], p1[1] - centroid[1]) < \
                        np.hypot(p2[0] - centroid[0], p2[1] - centroid[1]):
                    join(px1,px2)
                else:
                    join(px2,px1)

    def remove_overlapping(self, nodes, neighbors):
        thresh = settings.mesh['initial_min_dist']

        def line_dist(linep1, linep2, point):
            assert isinstance(point, tuple)
            a = point[0] - linep1[0]
            b = point[1] - linep1[1]
            c = linep2[0] - linep1[0]
            d = linep2[1] - linep1[1]
            dot = a*c + b*d
            len_sq = c*c + d*d
            param = -1
            if len_sq != 0:
                param = dot / len_sq
            if param < 0:
                xx = linep1[0]
                yy = linep1[1]
            elif param > 1:
                xx = linep2[0]
                yy = linep2[1]
            else:
                xx = linep1[0] + param*c
                yy = linep1[1] + param*d
            dx = point[0] - xx
            dy = point[1] - yy
            dist = np.sqrt(dx*dx + dy*dy)
            return dist


        # prune edges which are too close to non-neighbor points
        for px1 in nodes:
            p1 = nodes[px1]
            for px2 in deepcopy(neighbors[px1]):
                p2 = nodes[px2]
                for nx in nodes:
                    if nx == px1 or nx == px2 or px1 not in neighbors[px2] or px2 not in neighbors[px1]:
                        continue
                    n = nodes[nx]
                    if line_dist(p1, p2, n) < settings.mesh['initial_min_dist']:
                        neighbors[px1].remove(px2)
                        neighbors[px2].remove(px1)
                        print("info: pruning edge" + str(p1) + " to " + str(p2) + 
                                ", too close to " + str(n))

        def line_intersect(a,b,c,d):
            def ccw(p1,p2,p3):
                return (p3[1]-p1[1]) * (p2[0]-p1[0]) > (p2[1]-p1[1]) * (p3[0]-p1[0])

            return ccw(a,c,d) != ccw(b,c,d) and ccw(a,b,c) != ccw(a,b,d)

        # prune overlapping edges
        edges = set()
        for px1 in nodes:
            for px2 in deepcopy(neighbors[px1]):
                edges.add((px1,px2))
        removed_edges = set()
        for e1 in edges:
            if e1 in removed_edges:
                continue
            for e2 in edges:
                if e2 in removed_edges:
                    continue
                if e1[0] == e2[0] or e1[0] == e2[1] or \
                        e1[1] == e2[0] or e1[1] == e2[1]:
                    continue
                prune = e1
                save = e2
                # prune longest
                g = self.pad_grid
                ex1 = (g.coords(e1[0]), g.coords(e1[1]))
                ex2 = (g.coords(e2[0]), g.coords(e2[1]))
                if np.hypot(ex1[0][0]-ex1[1][0], ex1[0][1]-ex1[1][1]) > \
                        np.hypot(ex2[0][0]-ex2[1][0], ex2[0][1]-ex2[1][1]):
                    prune = e1
                    save = e2
                else:
                    prune = e2
                    save = e1

                if line_intersect(e1[0], e1[1], e2[0], e2[1]):
                    try:
                        removed_edges.add(prune)
                        neighbors[prune[0]].remove(prune[1])
                        neighbors[prune[1]].remove(prune[0])
                        prunex = (g.coords(prune[0]), g.coords(prune[1]))
                        savex = (g.coords(save[0]), g.coords(save[1]))
                        print("info: pruning edge " + str(prunex[0]) + " to " + str(prunex[1]) + 
                                ", overlapping " + str(savex[0]) + " to " + str(savex[1]))
                    except KeyError:
                        pass


    def plot(self, show=True, nodes=True):
        import matplotlib.pyplot as plt
        grid = self.occupancy_grid
        px,py = grid.grid.shape
        orig = grid.meta['origin']

        ext = [orig[0], orig[0]+py*grid.res(), orig[1], orig[1]+px*grid.res()]

        plt.imshow(grid.grid,  interpolation='none', cmap=plt.cm.gray, \
                aspect='equal', extent=ext)
        plt.imshow(self.pad_grid.grid,  interpolation='none', cmap=plt.cm.gray, \
                aspect='equal', extent=ext, alpha=.2)

        if nodes:
            initial_xs = []
            initial_ys = []
            xs = []
            ys = []
            for node in self.nodes:
                if node in self.initial_points:
                    initial_xs.append(node[0])
                    initial_ys.append(node[1])
                else:
                    xs.append(node[0])
                    ys.append(node[1])

            plt.plot(xs, ys, 'go')
            plt.plot(initial_xs, initial_ys,'ro')

            for ix,node in enumerate(self.nodes):
                for nbix in self.neighbors[ix]:
                    nb = self.nodes[nbix]
                    plt.plot([node[0], nb[0]], [node[1], nb[1]], color='0.1')
        if show:
            plt.show()

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

    def in_bounds(self, px):
        x,y = px
        return 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]

    def cost(self, px1, px2):
        # TODO use surrounding obstacles AND unexplored gets high cost
        a,b = px1
        c,d = px2
        return np.hypot(a-c, b-d)

       
    @classmethod
    def from_pgm(cls, base_file_name):
        import yaml
        from scipy.misc import imread
        with open(base_file_name + '.yaml') as stream:
            meta = yaml.load(stream)
    
        with open(base_file_name + '.pgm') as stream:
            grid = imread(stream).T
    
        return cls(grid, meta)
    
    """
    radius is in pixels
    """
    def pad_occupied(self, radius):
        sx,sy = self.grid.shape
        new_grid = self.grid.copy()
        for i in np.arange(0, sx):
            for j in np.arange(0, sy):
                if self.grid[i,j] < self.occupied_thresh():
                    x,y = np.ogrid[-i:sx-i, -j:sy-j]
                    new_grid[x*x + y*y <= radius*radius] = 0
        return OccupancyGrid(new_grid, self.meta)

    def free_thresh(self):
        return (1-self.meta['free_thresh'])*255

    def occupied_thresh(self):
        return (1-self.meta['occupied_thresh'])*255

    def res(self):
        return self.meta['resolution']

    def closest_free(self, point, radius=100):
        if self.is_free(point, pixel=False):
            return point
        start = self.pixel(point)
        h = []
        queue = PriorityQueue()
        queue.put(start, 0)
        ixs = [[1,0], [0,1], [-1,0], [0,-1]]
        added = set()
        while not queue.empty():
            c,px = queue.get()
            if c > radius:
                return None
            if self.in_bounds(px) and self.is_free(px):
                return self.coords(px)
            for cand in map(lambda x:(x[0]+px[0], x[1]+px[1]), ixs):
                if cand not in added:
                    candp = self.coords(cand)
                    dist = np.hypot(point[0] - candp[0], point[1] - candp[1])
                    if dist <= radius:
                        queue.put(cand, dist)
                        added.add(cand)
        return None

    def is_free(self, point, pixel = True):
        thresh = self.free_thresh()
        if settings.plan['walk_unknown']:
            thresh = self.occupied_thresh()+1
        if not pixel:
            px = self.pixel(point)
        else:
            px = point
        return self.in_bounds(px) and self.grid[px[0], px[1]] > thresh

    def is_occupied(self, point, pixel = True):
        thresh = self.occupied_thresh()
        if not settings.plan['walk_unknown']:
            thresh = self.free_thresh()-1
        if not pixel:
            px = self.pixel(point)
        else:
            px = point
        return not self.in_bounds(px) or self.grid[px[0], px[1]] < thresh
        
    def coords(self, px):
        orig = self.meta['origin']
        res = self.meta['resolution']
        i_min = self.grid.shape[0]
        j_min = self.grid.shape[1]
        x_min = orig[0]
        y_min = orig[1]
        x_max = j_min*res+orig[0]
        y_max = i_min*res+orig[1]
        x = self._translate(px[1], 0, j_min, x_min, x_max)
        y = self._translate(px[0], 0, i_min, y_max, y_min)
        x += res/2
        y -= res/2
        return (x,y)

    def pixel(self, p):
        orig = self.meta['origin']
        res = self.meta['resolution']
        i_min = self.grid.shape[0]
        j_min = self.grid.shape[1]
        x_min = orig[0]
        y_min = orig[1]
        x_max = j_min*res+orig[0]
        y_max = i_min*res+orig[1]
        p1 = self._translate(p[1], y_max, y_min, 0, i_min)
        p2 = self._translate(p[0], x_min, x_max, 0, j_min)
        return (int(p1), int(p2))

    def _translate(self, src, src_min, src_max, res_min, res_max):
        return float((src - src_min)) / \
                float((src_max - src_min)) * \
                float((res_max - res_min)) + res_min

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    return path

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)

def a_star_search(grid, start, goal, diagonal = True):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    def can_walk(px1, px2):
        if not grid.in_bounds(px2):
            return False
        if np.hypot(px1[0]-px2[0], px1[1]-px2[1]) > 1:
            dx = px2[0]-px1[0]
            dy = px2[1]-px1[1]
            return not grid.is_occupied(px2) and \
                    not grid.is_occupied((px1[0]+dx, px1[1])) and \
                    not grid.is_occupied((px1[0], px1[1]+dy))
        else:
            return not grid.is_occupied(px2)
    
    while not frontier.empty():
        current = frontier.get()[1]
        
        if current == goal:
            break

        x,y = current
        neighbors = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        if diagonal:
            neighbors += [(x+1,y+1),(x+1,y-1),(x-1,y+1),(x-1,y-1)]

        neighbors = filter(lambda x: can_walk(current, x), neighbors)

        # serves as random tie-breaker, creating prettier paths  
        random.shuffle(neighbors)

        for next in neighbors:
            new_cost = cost_so_far[current] + grid.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                heuristic = np.hypot(goal[0]-next[0], goal[1]-next[1])
                priority = new_cost + heuristic
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

if __name__ == "__main__":
    from world import World
    grid = OccupancyGrid.from_pgm('../../data/mapToG2')
    #w = World.from_json('../../data/mapToG2_simple.json')
    w = World.from_json('../../data/mapToG2_big.json')

    #grid = OccupancyGrid.from_pgm('../../data/willow-full')
    #w = World.from_json('../../data/willow-full_big.json')

    x,y = grid.grid.shape
    points = []
    for waypoint in w.waypoints:
        points.append(waypoint.point)
    m = Map(grid, points, consistency=True)
    w.add_map_info(m)
    m.plot()
