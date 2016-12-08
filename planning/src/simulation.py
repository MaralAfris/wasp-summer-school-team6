#!/usr/bin/env python

from world import *
from action import *
from map import *
from planner import *

from copy import *

import time
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

        for agent in world.agents:
            agent.path = None

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
                self.paths[(nnode,node)] = mypath
                reversepath = deepcopy(mypath)
                reversepath.reverse()
                self.paths[(node,nnode)] = reversepath

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
                if hasattr(action, 'agent'):
                    action.agent.path = None
                self.current_actions.remove(action)
                for succ_action in action.succ:
                    can_run = True
                    for pre_action in succ_action.pre:
                        if not pre_action.completed:
                            can_run = False
                            break
                    if can_run:
                        if not isinstance(succ_action, PathPlanning):
                            print(str(self.time) + ": " + succ_action.format())
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
                            succ_action.agent.path = copy(self.paths[(s,t)])
                        self.current_actions.append(succ_action)
            elif isinstance(action, Move):
                # Move actions have an intermediate position where we have to update agent 
                # position.
                s = action.start.point
                t = action.to.point
                whole_path = self.paths[(s,t)]

                completed = (self.time-action.start_time) / float(action.duration)

                path_length = self.path_lengths[(s,t)]
                dist = 0
                current_point = whole_path[0]
                next_point = whole_path[1]
                point_completed = 0
                point_ix = 0

                def interpolate(p1, p2, pct):
                    if pct <= 0:
                        return p1
                    if pct >= 1:
                        return p2
                    inter = (p2[0]*pct+p1[0]*(1-pct), p2[1]*pct+p1[1]*(1-pct))
                    return inter

                for i in np.arange(0,len(whole_path)-1):
                    p = whole_path[i]
                    next = whole_path[i+1]
                    d = np.hypot(p[0]-next[0], p[1]-next[1])
                    if (dist+d) / self.path_lengths[(s,t)] >= completed:
                        current_point = p
                        next_point = whole_path[i+1]
                        comp_before = dist / path_length
                        comp_after = (dist+d) / path_length
                        pct = (completed-comp_before) / (comp_after-comp_before)
                        action.agent.location = Waypoint(
                                action.agent.name+"_int",
                                interpolate(p,next,pct), 
                                PointType.location)
                        action.agent.path = copy(whole_path[i:len(whole_path)])
                        break
                    dist += d

        self.time += delta_t

    def draw(self, map, fig, ax):
        from matplotlib.patches import Circle, Rectangle

        offset = 1
        for a in self.current_actions:
            #plt.text(self.xlim[0]+0.5, self.ylim[1]-1.5*offset, a.format(), 
                    #fontsize=16)
            plt.text(self.xlim[0]+0.1, self.ylim[0]+0.5*offset, a.format(), 
                    fontsize=16)
            offset += 1

        turtlebots_xs = []
        turtlebots_ys = []
        drones_xs = []
        drones_ys = []
        def draw_path(agent):
            if agent.path is not None:
                xs = []
                ys = []
                for p in agent.path:
                    xs.append(p[0])
                    ys.append(p[1])
                plt.plot(xs, ys, 'r-')

        remap = dict((v, k) for k, v in map.moved_points.iteritems())
        for person in self.persons:
            color = 'r'
            if person.handled:
                color = 'b'
            xy = person.location.point
            if xy in remap:
                xy = remap[xy]
            xy = (xy[0], xy[1])
            ax.add_artist(Circle(
                xy=xy, color=color, radius=0.25))
            ax.add_artist(Circle(
                xy=xy, color='k', fill=False, radius=0.25))

        for box in self.boxes:
            if hasattr(box, 'location') and box.location is not None:
                if box.free:
                    color = '#b24013'
                else:
                    color = '#5e2d1a'
                xy = box.location.point
                if xy in remap:
                    xy = remap[xy]
                xy = (xy[0]-0.1, xy[1]-0.1)
                ax.add_artist(Rectangle(
                    xy = xy, color=color, width=0.20, height=0.20))
                ax.add_artist(Rectangle(
                    xy = xy, color='k', fill=False, width=0.20, height=0.20))
        for agent in self.agents:
            if agent.agent_type == 'turtlebot':
                ax.add_artist(Circle(
                    xy=agent.location.point, color='#555555', radius=0.175))
                ax.add_artist(Circle(
                    xy=agent.location.point, color='k', fill=False, radius=0.175))
                draw_path(agent)
                plt.text(agent.location.point[0], agent.location.point[1], agent.name)

        for agent in self.agents:
            if agent.agent_type == 'drone':
                xy = agent.location.point
                ax.add_artist(Rectangle(
                    xy=(xy[0],xy[1]-np.hypot(0.3,0.3)/2), 
                    color='g', width=0.30, height=0.30, angle=45))
                ax.add_artist(Rectangle(
                    xy=(xy[0],xy[1]-np.hypot(0.3,0.3)/2), fill=False,
                    color='k', width=0.30, height=0.30, angle=45))
                draw_path(agent)        
                plt.text(agent.location.point[0], agent.location.point[1], agent.name)
        fig.canvas.draw()
        
if __name__ == "__main__":
    
    use_old = False
    if len(sys.argv) > 1:
        print("info: using old plan")
        use_old = True

    initial_state_json = '../../data/mapToDemo3_multi.json'
    grid = OccupancyGrid.from_pgm('../../data/mapToDemo3')
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

    grid = map.occupancy_grid
    px,py = grid.grid.shape
    orig = grid.meta['origin']
    #xlim = (orig[0], orig[0]+py*grid.res())
    #ylim = (orig[1], orig[1]+px*grid.res())
    xlim = (np.inf, -np.inf)
    ylim = (np.inf, -np.inf)
    for i in np.arange(0, px):
        for j in np.arange(0, py):
            if grid.is_free((i,j)):
                coord = grid.coords((i,j))
                xlim = (np.min((xlim[0], coord[0])), np.max((xlim[1], coord[0])))
                ylim = (np.min((ylim[0], coord[1])), np.max((ylim[1], coord[1])))

    sim.xlim = xlim
    sim.ylim = ylim

    #plt.ylim(ylim)
    #plt.xlim(xlim)

    plt.ion()
    dpi = 96
    dt = 0.04 # 25fps
    fig = plt.figure(figsize=(1920/dpi,1080/dpi), dpi=dpi, frameon=False)
    fig.set_size_inches(1920/dpi,1080/dpi)
    ax = fig.gca()


    while len(sim.current_actions) > 0:
        ax.cla()
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        map.plot(show=False,nodes=False)
        sim.tick(dt)
        sim.draw(map, fig, ax)
        t = round(sim.time,2)
        ststr = str(int(t)).zfill(4)
        msstr = '%.2f' % (t-int(t))
        tstr = ststr + msstr[1:len(msstr)]
        #plt.savefig('video/simulation_'+tstr+'.eps', format='eps', bbox_inches='tight', dpi=dpi), 
        plt.savefig('video/simulation_'+tstr+'.png',  bbox_inches='tight', dpi=dpi), 

    for i in range(0,int(1/dt)):
        sim.tick(dt)
        t = round(sim.time,2)
        ststr = str(int(t)).zfill(4)
        msstr = '%.2f' % (t-int(t))
        tstr = ststr + msstr[1:len(msstr)]
        #plt.savefig('video/simulation_'+tstr+'.eps', format='eps', bbox_inches='tight', dpi=dpi), 
        plt.savefig('video/simulation_'+tstr+'.png', bbox_inches='tight', dpi=dpi), 

    sim.draw(map, fig, ax)
    time.sleep(2**31-1)
