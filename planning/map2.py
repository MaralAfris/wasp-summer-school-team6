#!/usr/bin/env python

from __future__ import generators
import settings
from copy import deepcopy
import heapq
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

        self.initial_points = points
        self.occupancy_grid = occupancy_grid
        downsample_factor = int(np.ceil(settings.mesh['padding_radius']/occupancy_grid.res()))
        downsample = occupancy_grid.downsample_grid(downsample_factor)
        radius = settings.mesh['padding_radius']# / downsample.res()
        downsample = OccupancyGrid(downsample.pad_occupied(radius), downsample.meta)
        self.downsample = downsample

        # Make sure initial points are not on occupied cells
        for i, point in enumerate(points):
            update = downsample.closest_free(point)
            if update is not None and update != point:
                print('warning: initial point ' + str(point) + 
                        ' is occupied, relocating to ' + str(update))
                points[i] = update
            elif update is None:
                print('warning: initial point ' + str(point) + 
                        ' is occupied, no free point found')


        # Next compute a* search for each path on Delaunay triangulation.
        triangles = Delaunay(points)
        nodes = {}       # pixel to coords: tuple->tuple
        neighbors = {}   # pixel to neighbors: tuple->[tuple]
        searched = set() # coords: tuple

        initial_px = set()
        for p in points:
            px = downsample.pixel(p)
            nodes[px] = (p[0], p[1])
            neighbors[px] = set()
            initial_px.add(px)

        for i,tri in enumerate(triangles.simplices):
            def path(ix1,ix2):
                px1 = downsample.pixel(points[ix1])
                px2 = downsample.pixel(points[ix2])
                if downsample.grid[px1[0], px1[1]] < downsample.occupied_thresh() or \
                        downsample.grid[px2[0], px2[1]] < downsample.occupied_thresh():
                    print('warning: goal or start occupied ' + 
                            str(points[ix1]) + ' ' + str(points[ix2]))
                    return

                # Don't need to do search sometimes because all occur twice 
                if (ix1,ix2) in searched or (ix2,ix1) in searched:
                    return
                searched.add((ix1,ix2))
                start = (px1[0], px1[1])
                goal = (px2[0], px2[1])
                came_from, costs = a_star_search(downsample, start, goal)
                if goal not in came_from or start not in came_from:
                    print('warning: initial path not reachable ' + 
                            str(points[ix1]) + ' ' + str(points[ix2]))
                    return
                mypath = reconstruct_path(came_from, start, goal)
                for i in np.arange(0, len(mypath)-1):
                    p1 = mypath[i]
                    p2 = mypath[i+1]
                    if p1 not in nodes:
                        nodes[p1] = downsample.coords(p1, center=True)
                        neighbors[p1] = set()
                    if p2 not in nodes:
                        nodes[p2] = downsample.coords(p2, center=True)
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


        while True:
            pre_len = len(nodes)
            self.straighten_paths(nodes, neighbors, initial_px, 0.01)
            self.join_adjacent(downsample, nodes, neighbors, initial_px)
            self.straighten_paths(nodes, neighbors, initial_px, 0)
            if pre_len == len(nodes):
                break
            check_consistency()

        ix = 0
        index = {}
        self.nodes = []
        self.neighbors = []
        for px,p in nodes.iteritems():
            index[px] = len(self.nodes)
            self.nodes.append(p)
        for px,p in nodes.iteritems():
            nbs = []
            self.neighbors.append(nbs)
            for nb in neighbors[px]:
                nbs.append(index[nb])

    """ 
    ratio is the minimum ratio between area and length
    """
    def straighten_paths(self, nodes, neighbors, initial_px, ratio):
        for px in deepcopy(nodes):
            if px in neighbors and len(neighbors[px]) < 3 and px not in initial_px:
                if len(neighbors[px]) == 2:
                    aslist = list(neighbors[px])
                    px1 = aslist[0]
                    px2 = aslist[1]
                    if ratio > 0:
                        p = nodes[px]
                        p1 = nodes[px1]
                        p2 = nodes[px2]
                        a = np.hypot(p1[0]-p[0], p1[1]-p[1])
                        b = np.hypot(p2[0]-p[0], p2[1]-p[1])
                        c = np.hypot(p1[0]-p2[0], p1[1]-p2[1])
                        s = (a+b+c)/2
                        area = np.sqrt(s*abs(s-a)*abs(s-b)*abs(s-c))
                        length = np.max([a,b,c])
                        if area/length > ratio:
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
        skip_pairs = set()
        while True:
            for n in nodes:
                assert n != None
                assert nodes[n] != None
            # TODO make O(nlogn)
            def closest(points):
                min_dist = np.inf
                for px in points:
                    for bx in points:
                        p = nodes[px]
                        b = nodes[bx]
                        dist = np.hypot(p[0]-b[0], p[1]-b[1])
                        if (bx,px) not in skip_pairs and b != p and dist < min_dist:
                            min_dist = dist
                            p1 = px
                            p2 = bx
                return (min_dist, p1, p2)

            dist, px1, px2 = closestpair(list(nodes.keys()), nodes, skip_pairs)

            if dist > settings.mesh['min_dist']:
                break

            def join(px1, px2, center=False):
                if center:
                    p1 = nodes[px1]
                    p2 = nodes[px2]
                    centroid = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
                    nodes[px1] = grid.closest_free(centroid, radius=1/grid.res())
                for neighbor in neighbors[px2]:
                    if neighbor != px1:
                        neighbors[neighbor].remove(px2)
                        neighbors[neighbor].add(px1)
                neighbors[px1].update(neighbors[px2])
                if px1 in neighbors[px1]:
                    neighbors[px1].remove(px1)
                if px2 in neighbors[px1]:
                    neighbors[px1].remove(px2)
                del neighbors[px2]
                del nodes[px2]
                return px1

            if px1 in initial_px and px2 in initial_px:
                skip_pairs.add((px1, px2))
                skip_pairs.add((px2, px1))
                continue
            elif px1 in initial_px:
                join(px1, px2)
            elif px2 in initial_px:
                join(px2, px1)
            else:
                join(px1, px2, center=True)

    def plot(self, show=True):
        import matplotlib.pyplot as plt
        grid = self.occupancy_grid
        px,py = grid.grid.shape
        min_coords = grid.coords((0, 0))
        max_coords = grid.coords((px, py))

        ext = [min_coords[0], max_coords[0], min_coords[1], max_coords[1]]
        plt.imshow(grid.grid.T,  interpolation='none', cmap=plt.cm.gray, \
                aspect='equal', extent=ext, origin='lower')
        #plt.imshow(self.downsample.grid.T,  interpolation='none', cmap=plt.cm.gray, \
                #aspect='equal', extent=ext, origin='lower')

        xs = []
        ys = []
        for node in self.initial_points:
            xs.append(node[0])
            ys.append(node[1])

        for ix,node in enumerate(self.nodes):
            for nbix in self.neighbors[ix]:
                nb = self.nodes[nbix]
                plt.plot([node[0], nb[0]], [node[1], nb[1]], color='0.1')
        plt.plot(xs,ys,'o')
        if show:
            plt.show()



def closestpair(L, nodes, skip_pairs):

    def dist(px,qx):
        p = nodes[px]
        q = nodes[qx]
        return np.hypot(p[0]-q[0],p[1]-q[1])

    best = [dist(L[0],L[1]), (L[0],L[1])]

    # check whether pair (p,q) forms a closer pair than one seen already
    def testpair(px,qx):
        if (px,qx) in skip_pairs:
            return
        d = dist(px,qx)
        if d < best[0]:
            best[0] = d
            best[1] = (px,qx)
            
    # merge two sorted lists by y-coordinate
    def merge(A,B):
        i = 0
        j = 0
        while i < len(A) or j < len(B):
            if j >= len(B) or (i < len(A) and A[i][1] <= B[j][1]):
                yield A[i]
                i += 1
            else:
                yield B[j]
                j += 1

    # Find closest pair recursively; returns all points sorted by y coordinate
    def recur(L):
        if len(L) < 2:
            return L
        split = len(L)/2
        splitx = L[split][0]
        L = list(merge(recur(L[:split]), recur(L[split:])))
        E = [p for p in L if abs(p[0]-splitx) < best[0]]
        for i in range(len(E)):
            for j in range(1,8):
                if i+j < len(E):
                    testpair(E[i],E[i+j])
        return L
    
    L.sort()
    recur(L)
    return (best[0], best[1][0], best[1][1])

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

    def passable(self, px):
        return not self.is_occupied(px)

    def cost(self, px1, px2):
        a,b = px1
        c,d = px2
        return np.hypot(a-c, b-d)

    def neighbors(self, px):
        x,y = px
        results = [(x+1, y), 
                (x+1,y-1),
                (x, y-1), 
                (x-1, y-1),
                (x-1, y),
                (x-1, y+1),
                (x, y+1),
                (x+1, y+1)]
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results
        
    @classmethod
    def from_pgm(cls, base_file_name):
        import yaml
        from scipy.misc import imread
        with open(base_file_name + '.yaml') as stream:
            meta = yaml.load(stream)
    
        with open(base_file_name + '.pgm') as stream:
            grid = imread(stream)
    
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
                    y,x = np.ogrid[-i:sx-i, -j:sy-j]
                    mask = x*x + y*y <= radius*radius
                    new_grid[mask] = 0
        return new_grid

    def free_thresh(self):
        return (1-self.meta['free_thresh'])*255

    def occupied_thresh(self):
        return (1-self.meta['occupied_thresh'])*255

    def res(self):
        return self.meta['resolution']

    def closest_free(self, point, radius=1):
        # TODO use HeapQueue
        if self.is_free(point, pixel=False):
            return point
        start = self.pixel(point)
        h = []
        heapq.heappush(h, (0, start))
        ixs = [[1,0], [0,1], [-1,0], [0,-1]]
        added = set()
        while True:
            if not h:
                print "none"
                return None
            c,px = heapq.heappop(h)
            if c > radius:
                print "none"
                return None
            if self.in_bounds(px) and self.is_free(px):
                return self.coords(px, center=True)
            for cand in map(lambda x:(x[0]+px[0], x[1]+px[1]), ixs):
                if cand not in added:
                    candp = self.coords(cand, center=True)
                    dist = np.hypot(point[0] - candp[0], point[1] - candp[1])
                    heapq.heappush(h, (dist, cand))
                    added.add(cand)

    def is_free(self, point, pixel = True):
        if not pixel:
            px = self.pixel(point)
        else:
            px = point
        return self.grid[px[0], px[1]] > self.free_thresh()

    def is_occupied(self, point, pixel = True):
        if not pixel:
            px = self.pixel(point)
        else:
            px = point
        return self.grid[px[0], px[1]] < self.occupied_thresh()
        
    def coords(self, px, center = False):
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
        if center == True:
            x += res/2
            y += res/2
        return (x,y)

    def pixel(self, p):
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
        return (int(p1), int(p2))

    def _translate(self, src, src_min, src_max, res_min, res_max):
        return float((src - src_min)) / \
                float((src_max - src_min)) * \
                float((res_max - res_min)) + res_min

    def downsample_grid(self, fact):
        meta = deepcopy(self.meta)
        meta['resolution'] = meta['resolution'] * fact
        res = meta['resolution']

        x, y = self.grid.shape
        sx, sy = (int(round(x/float(fact))), int(round(y/float(fact))))
        img = np.zeros((sx,sy))

        for xi in np.arange(0, sx):
            for yi in np.arange(0, sy):
                arr = self.grid[xi*fact:(xi+1)*fact, yi*fact:(yi+1)*fact]
                free = np.where(arr > self.free_thresh())
                occ = np.where(arr < self.occupied_thresh())
                if occ[0].shape[0] > 0:
                    img[xi,yi] = 0
                elif free[0].shape[0] > (fact**2)/2:
                    img[xi,yi] = 255
                else:
                    img[xi,yi] = (self.free_thresh()+self.occupied_thresh())/2

        return OccupancyGrid(img, meta)

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
        return heapq.heappop(self.elements)[1]

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            # symmetry breaking noise, still deterministic
            r = (current.__hash__() % 100)/10000.0
            new_cost = cost_so_far[current] + graph.cost(current, next) + r
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                heuristic = np.hypot(goal[0]-next[0], goal[1]-next[1])
                priority = new_cost + heuristic
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

if __name__ == "__main__":
    grid = OccupancyGrid.from_pgm('../../data/mapToG2')
    #grid = OccupancyGrid.from_pgm('../../data/willow-full')
    from world import World
    w = World.from_json('../../data/problems/room.json')
    #w = World.from_json('../../data/problems/willow-big.json')
    points = []
    for waypoint in w.waypoints:
        points.append(waypoint.point)
    m = Map(grid, points, consistency=False)
    w.add_map_info(m)
    w.generate_problem('test.pddl')
    m.plot()
