#!/usr/bin/env python
 
import matplotlib.pyplot as plt
from scipy.spatial import voronoi_plot_2d
from map import *
from world import *

def display(world, map):
    # plots tb_grid waypoints
    #tb_grid = map.occupancy_grid.downsample_grid(7)
    #for wp in world.waypoints:
        #p1 = wp.point
        #pixels = tb_grid.coords_to_pixel(p1.x, p1.y)
        #coords = tb_grid.pixel_to_coords(pixels[0], pixels[1])
        #plt.plot(coords[0], coords[1], 'o')

    # plot regions
    #for region in map.regions:
        #plt.triplot(region.points[:,0], region.points[:,1], region.simplices.copy())

    grid = map.occupancy_grid
    px,py = grid.grid.shape

    min_coords = grid.pixel_to_coords((0, 0))
    max_coords = grid.pixel_to_coords((px, py))

    ext = [min_coords[0], max_coords[0], min_coords[1], max_coords[1]]

    #plt.imshow(map.occupancy_grid.downsample_grid(7).grid.T,  interpolation='none', cmap=plt.cm.gray, \
            #aspect='equal', extent=ext, origin='lower')
    plt.imshow(map.occupancy_grid.grid.T,  interpolation='none', cmap=plt.cm.gray, \
            aspect='equal', extent=ext, origin='lower')
    #plt.imshow(map.occupancy_grid.downsample_grid(7).grid.T, alpha = 0.5, interpolation='none', \
            #aspect='equal', extent=ext, origin='lower')
    #for e in world.edges:
        #if e.pruned == Pruned.none or e.pruned == Pruned.regions:
        #if e.pruned == Pruned.none:
            #plt.plot([e.p1.p[0], e.p2.p[0]], [e.p1.p[1], e.p2.p[1]], color='0.1')

    #point_xs = []
    #point_ys = []
    #for point in map.points:
        #point_xs.append(point.p[0])
        #point_ys.append(point.p[1])
    #plt.plot(point_xs, point_ys, 'sk')


    agent_xs = []
    agent_ys = []
    for a in world.agents:
        wp = world.waypoint(a.location)
        plt.annotate(a.name, xy=wp.point.p)
        agent_xs.append(wp.point.p[0])
        agent_ys.append(wp.point.p[1])
    plt.plot(agent_xs, agent_ys, 'o', color='b')

    victim_xs = []
    victim_ys = []
    for v in world.persons:
        if not v.handled:
            wp = world.waypoint(v.location)
            plt.annotate(v.name, xy=wp.point.p)
            victim_xs.append(wp.point.p[0])
            victim_ys.append(wp.point.p[1])
    plt.plot(victim_xs, victim_ys, 'o', color='r')

    boxes_xs = []
    boxes_ys = []
    for b in world.boxes:
        if b.free:
            wp = world.waypoint(b.location)
            boxes_xs.append(wp.point.p[0])
            boxes_ys.append(wp.point.p[1])
    plt.plot(boxes_xs, boxes_ys, 'gs')

    xmin = float("inf")
    xmax = -float("inf")
    ymin = float("inf")
    ymax = -float("inf")
    for wp in world.waypoints:
        xmin = min(wp.point.p[0], xmin)
        xmax = max(wp.point.p[0], xmax)
        ymin = min(wp.point.p[1], ymin)
        ymax = max(wp.point.p[1], ymax)

    xscale = xmax-xmin
    yscale = ymax-ymin
    #plt.xlim(xmin - xscale*0.3, xmax + xscale*0.3)
    #plt.ylim(ymin - yscale*0.3, ymax + yscale*0.3)

    plt.show()


if __name__ == "__main__":
    world = World.from_json('../../data/problems/room.json')

    points = []
    for waypoint in world.waypoints:
        points.append(waypoint.point)
    grid = OccupancyGrid.from_pgm('../../data/mapToG2')
    map = Map(grid, points)
    #world.add_map_info(map)
    display(world, map)
