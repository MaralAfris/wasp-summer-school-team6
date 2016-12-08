#!/usr/bin/env python

import settings
from world import *
from action import *
from map import *
import coordinator

import subprocess,os,sys,inspect
import inspect, os

PLAN = "_out.plan"
PROBLEM = "_problem.pddl"

def generate_plan(world, map):
    abspath = os.path.abspath(inspect.getfile(inspect.currentframe()))
    script_dir = os.path.dirname(abspath)
    if os.path.isfile(PROBLEM):
        print("info: unlinking old file " + PROBLEM)
        os.unlink(PROBLEM)
    world.generate_problem(PROBLEM)

    domain_file = os.path.join(script_dir, 'domain.pddl')
    if os.path.isfile(PLAN):
        print("info: unlinking old file " + PLAN)
        os.unlink(PLAN)

    for i in xrange(1,sys.maxint):
        f = PLAN+"."+str(i)
        if os.path.isfile(f):
            print("info: unlinking old file " + f)
            os.unlink(f)
        else:
            break

    if settings.plan['optimal_search'] > 0:
        print("info: running optimal planning with yahsp3 for " +
            str(settings.plan['optimal_search']) + " secs.")
        out = subprocess.call('yahsp3 -v 0 -K 0.01,0.01 -t ' + str(settings.plan['optimal_search']) +
                ' -o ' + domain_file + ' -f ' + PROBLEM + ' -H ' + PLAN, shell = True)
    else:
        out = -1

    planner_file = PLAN

    if out != 0:
        if settings.plan['optimal_search'] > 0:
            print("info: failed to find optimal plan [" + str(out) + "]")
        print("info: running heuristic suboptimal planning with yahsp3 for " +
                str(settings.plan['suboptimal_search']) + " secs.")
        out = subprocess.call('yahsp3 -N -y 1 -v 0 -K 0.01,0.01 -t ' +
                str(settings.plan['suboptimal_search']) + ' -o ' + domain_file + ' -f ' +
                PROBLEM + ' -H ' + PLAN + " > /dev/null", shell = True)
        # search for latest plan in the list
        for i in xrange(1,sys.maxint):
            f = PLAN+"."+str(i)
            if os.path.isfile(f):
                planner_file = f
            else:
                print("info: using suboptimal plan: " + planner_file)
                break
    else:
        print("info: using optimal plan: " + planner_file)
    if not os.path.isfile(planner_file):
        raise Exception('failed to create plan: ' + str(out))

    return planner_file

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
	actions.append(Land(['drone1'], 1, world))
	actions.insert(0, TakeOff(['drone1'], 1, world))
    graph = construct_graph(actions, world)

    coordinator.start(graph, actions)
