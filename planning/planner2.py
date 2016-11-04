#!/usr/bin/env python

from world import World
from action import Action,construct_graph

import subprocess,os,sys

MAX_PLAN_TIME = 10 # for real scenario, use 30
PLAN = "_out.plan"
PROBLEM = "_problem.pddl"
    
if __name__ == "__main__":
    
    initial_state_json = sys.argv[1]

    world = World.from_json(initial_state_json)
    world.generate_problem(PROBLEM)

    out = subprocess.call('yahsp3 -y 1 -v 0 -K 0.01,0.01 -t ' + str(MAX_PLAN_TIME) + 
            ' -o domain.pddl -f ' + PROBLEM + ' -H ' + PLAN, shell = True)

    # timer interruption is 104
    if out != 0 and out != 104:
        raise Exception("Planner failed")

    (graph, actions) = construct_graph(PLAN, world)
    for a in actions:
        print(a.format())

    #os.unlink(PROBLEM)
    #os.unlink(PLAN)


    world.to_json('_before.json')

    # Execute some actions for funsies
    for action in graph.succ:
        action.complete_action()

    world.to_json('_after.json')
    world.plot()

