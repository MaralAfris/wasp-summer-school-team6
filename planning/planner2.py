#!/usr/bin/env python

from world import World
from action import Action,construct_graph

import subprocess,os,sys

MAX_PLAN_TIME = 5
PLAN = "_out.plan"
PROBLEM = "_problem.pddl"
    
if __name__ == "__main__":
    
    initial_state_json = sys.argv[1]

    world = World.from_json(initial_state_json)
    world.create_triangulation()
    world.generate_problem(PROBLEM)

    out = subprocess.call('yahsp3 -K 0.01,0.01 -t ' + str(MAX_PLAN_TIME) + ' -o domain.pddl -f ' + PROBLEM + ' -H ' + PLAN, shell = True)

    # timer interruption is 104
    if out != 0 and out != 104:
        raise Exception("Planner failed")


    plan = '_plan'

    graph = construct_graph(plan, world)
    graph.print_deps()

    os.unlink(PROBLEM)
    os.unlink(PLAN)
