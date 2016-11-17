#!/usr/bin/env python

from world import World
from action import Action,construct_graph
from map import *

import subprocess,os,sys,inspect
import inspect, os

MAX_PLAN_TIME = 10 # for real scenario, use 30
PLAN = "_out.plan"
PROBLEM = "_problem.pddl"
    
if __name__ == "__main__":
    
    #initial_state_json = sys.argv[1]
    initial_state_json = '../data/problems/room.json'

    abspath = os.path.abspath(inspect.getfile(inspect.currentframe()))
    script_dir = os.path.dirname(abspath)

    world = World.from_json(initial_state_json)

    points = []
    for waypoint in world.waypoints:
        points.append(waypoint.point)
    grid = OccupancyGrid.from_pgm('../data/mapToG2')
    map = Map(grid, points)

    world.add_map_info(map)

    if os.path.isfile(PROBLEM):
        os.unlink(PROBLEM)
    world.generate_problem(PROBLEM)

    domain_file = os.path.join(script_dir, 'domain.pddl')
    if os.path.isfile(PLAN):
        os.unlink(PLAN)

    out = subprocess.call('yahsp3 -v 0 -K 0.01,0.01 -t ' + str(MAX_PLAN_TIME) + 
            ' -o ' + domain_file + ' -f ' + PROBLEM + ' -H ' + PLAN, shell = True)
    # planning failed to complete, try again with anytime behaviour
    if not os.path.isfile(PLAN):
        # TODO output ROS log
        out = subprocess.call('yahsp3 -y 1 -v 0 -K 0.01,0.01 '
            ' -o ' + domain_file + ' -f ' + PROBLEM + ' -H ' + PLAN, shell = True)

    # timer interruption is 104
    if out != 0 and out != 104:
        raise Exception("Planner failed: " + str(out))

    (graph, actions) = construct_graph(PLAN, world)
    for a in actions:
        print(a.format())

    world.to_json('_before.json')

    # Execute some actions for funsies
    for action in graph.succ:
        action.complete_action()

    world.to_json('_after.json')

