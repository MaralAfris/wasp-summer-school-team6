#!/usr/bin/env python

from world import *
from action import *
from map import *
from planner import *

from copy import *

import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Simulation(World):
    def __init__(self, world, graph, actions, map):
        super(Simulation, self).__init__(world.agents, world.waypoints, world.boxes, world.persons)
        print("info: initializing simulation")

        self.current_actions = [graph]
        self.time = 0
        graph.start_time = 0
        self.paths = {}
        self.path_lengths = {}

        # calculates a path for each walkable path
        for i,node in enumerate(map.nodes):
            px1 = map.pad_grid.pixel(node)
            for neighbor in map.neighbors[i]:
                nnode = map.nodes[neighbor]
                if (node,nnode) in self.paths:
                    continue
                px2 = map.pad_grid.pixel(nnode)
                came_from, costs = a_star_search(map.pad_grid, px1, px2, diagonal = True)
                mypath = [map.pad_grid.coords(x) for x in reconstruct_path(came_from, px1, px2)]
                mypath.insert(0,nnode)
                mypath.append(node)
                self.paths[(node,nnode)] = mypath
                reversepath = deepcopy(mypath)
                reversepath.reverse()
                self.paths[(nnode,node)] = reversepath

                # calculate length
                length = 0
                for i in np.arange(1, len(mypath)-1):
                    p1 = mypath[i]
                    p2 = mypath[i+1]
                    length += np.hypot(p1[0]-p2[0], p1[1]-p2[1])
                self.path_lengths[(node,nnode)] = length
                self.path_lengths[(nnode,node)] = length

    # update world simulation delta_t seconds
    def tick(self, delta_t):
        for action in copy(self.current_actions):
            if self.time >= action.start_time + action.duration:
                action.complete_action()
                self.current_actions.remove(action)
                for succ_action in action.succ:
                    can_run = True
                    for pre_action in succ_action.pre:
                        if not pre_action.completed:
                            can_run = False
                            break
                    if can_run:
                        if not isinstance(succ_action, Delay):
                            print(str(self.time) + ": " + succ_action.format())
                        self.current_actions.append(succ_action)
                        succ_action.start_time = self.time

                        # calculate move duration based on speed of agent
                        if isinstance(succ_action, Move):
                            agent = succ_action.agent
                            if agent.agent_type == 'turtlebot':
                                speed = settings.plan['turtle_speed']
                            elif agent.agent_type == 'drone':
                                if agent.carrying is None:
                                    speed = settings.plan['drone_speed']
                                else:
                                    speed = settings.plan['drone_carry_speed']
                            else:
                                raise Exception("Unknown agent_type " + agent.agent_type)
                            s = succ_action.start.point
                            t = succ_action.to.point
                            succ_action.duration = self.path_lengths[(s,t)] / speed
            elif isinstance(action, Move):
                # move actions have an intermediate position where we have to update agent position
                s = action.start.point
                t = action.to.point
                path = self.paths[(s,t)]

                completed = self.time / float(action.start_time + action.duration)

                path_length = self.path_lengths[(s,t)]
                dist = 0
                current_point = path[0]
                next_point = path[1]
                point_completed = 0
                point_ix = 0

                def interpolate(p1, p2, pct):
                    if pct <= 0:
                        return p1
                    if pct >= 1:
                        return p2
                    inter = (p2[0]*pct+p1[0]*(1-pct), p2[1]*pct+p1[1]*(1-pct))
                    return inter    

                for i in np.arange(0,len(path)-1):
                    p = path[i]
                    next = path[i+1]
                    d = np.hypot(p[0]-next[0], p[1]-next[1])
                    if (dist+d) / self.path_lengths[(s,t)] >= completed:
                        current_point = p
                        next_point = path[i+1]
                        comp_before = dist / path_length
                        comp_after = (dist+d) / path_length
                        pct = (completed-comp_before) / (comp_after-comp_before)
                        action.agent.location = Waypoint(
                                action.agent.name+"_int", interpolate(p,next,pct), PointType.location)
                        break
                    dist += d

        self.time += delta_t



if __name__ == "__main__":
    
    use_old = False
    if len(sys.argv) > 1:
        print("info: using old plan")
        use_old = True

    initial_state_json = '../../data/mapToG2_simple.json'
    grid = OccupancyGrid.from_pgm('../../data/mapToG2')
    #initial_state_json = '../../data/willow-full_big.json'
    #grid = OccupancyGrid.from_pgm('../../data/willow-full')

    world = World.from_json(initial_state_json)

    map = Map(grid, world.points())

    world.add_map_info(map)

    if not use_old:
        planner_file = generate_plan(world, map)
    else:
        planner_file = sys.argv[1]

    actions = parse_plan(planner_file, world)
    actions = add_delays(actions, world)
    graph = construct_graph(actions, world)

    sim = Simulation(world, graph, actions, map)

    dt = 0.01
    while len(sim.current_actions) > 0:
        sim.tick(dt)

    sys.exit(0)

    #mpl.rcParams['toolbar'] = 'None'
    fig = plt.figure()

    px,py = grid.grid.shape
    orig = grid.meta['origin']
    ext = [orig[0], orig[0]+py*grid.res(), orig[1], orig[1]+px*grid.res()]
    plt.imshow(grid.grid,  interpolation='none', cmap=plt.cm.gray, \
            aspect='equal', extent=ext)
    plt.imshow(self.pad_grid.grid,  interpolation='none', cmap=plt.cm.gray, \
            aspect='equal', extent=ext, alpha=.5)

    #initial_xs = []
    #initial_ys = []
    #xs = []
    #ys = []
    #for node in self.nodes:
        #if node in self.initial_points:
            #initial_xs.append(node[0])
            #initial_ys.append(node[1])
        #else:
            #xs.append(node[0])
            #ys.append(node[1])

    #plt.plot(xs, ys, 'go')
    #plt.plot(initial_xs, initial_ys,'ro')

    # drone \bigotimes
    # turtlebot \bigodot
    # medbox \boxplus
    # person ur'$\u1f6b9$'

    def plot(ax, style):
        return ax.plot(x, y, style, animated=True)[0]

    lines = [plot(ax, style) for ax, style in zip(axes, styles)]

    def animate(i):
        for j, line in enumerate(lines, start=1):
            line.set_ydata(np.sin(j*x + i/10.0))
        return lines

    # We'd normally specify a reasonable "interval" here...
    ani = animation.FuncAnimation(fig, animate, xrange(1, int(2*np.pi*200) ),
                                  interval=5, blit=True)
    #ani.save('animation.mp4')
    plt.show()
    ###

    #for ix,node in enumerate(self.nodes):
        #for nbix in self.neighbors[ix]:
            #nb = self.nodes[nbix]
            #plt.plot([node[0], nb[0]], [node[1], nb[1]], color='0.1')


