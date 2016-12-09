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
ACTIONS = "_actions.plan"

def generate_plan(world, map):
    if settings.plan['use_auction']:
        actions = generate_available_moves(world, map)
        f = open(ACTIONS, 'w')
        for a in actions:
            f.write(a.pddl_format() + '\n')
        f.close()

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
        arg = 'yahsp3 -v 1 -K 0.01,0.01 -t ' + str(settings.plan['optimal_search']) + \
                    ' -o ' + domain_file + ' -f ' + PROBLEM + ' -H ' + PLAN
        if settings.plan['use_auction']:
            arg += ' -z ' + ACTIONS
        out = subprocess.call(arg, shell = True)
    else:
        out = -1

    planner_file = PLAN

    if out != 0:
        if settings.plan['optimal_search'] > 0:
            print("info: failed to find optimal plan [" + str(out) + "]")
        print("info: running heuristic suboptimal planning with yahsp3 for " +
                str(settings.plan['suboptimal_search']) + " secs.")
        arg = 'yahsp3 -N -y 1 -v 1 -K 0.01,0.01 -t ' + \
                str(settings.plan['suboptimal_search']) + ' -o ' + domain_file + ' -f ' + \
                PROBLEM + ' -H ' + PLAN
        if settings.plan['use_auction']:
            arg += ' -z ' + ACTIONS
        out = subprocess.call(arg, shell = True)
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

def generate_available_moves(world, map):
    # assign boxes to persons
    # assign boxes and turtlebots to persons
    # assign drones to hand-overs

    mworld = deepcopy(world)
    points = mworld.points()
    costs = wp_cost(mworld, map)

    for agent in mworld.agents:
        agent.acum_cost = 0

    action_list = set()

    for person in filter(lambda x: not x.handled, mworld.persons):
        pp = person.location.point
        pi = points.index(pp)

        persons_box = None
        dist = np.inf
        # find closests box
        for box in filter(lambda x: x.free, mworld.boxes):
            bp = box.location.point
            bi = points.index(bp)
            d = costs[bi,pi]
            if d < dist:
                persons_box = box
                dist = d
        
        box = persons_box
        box.free = False
        bp = box.location.point
        bi = points.index(bp)
        # find closests turtlebot to box

        dist = np.inf
        persons_turtlebot = None
        for turtlebot in filter(lambda x: x.agent_type == 'turtlebot', mworld.agents):
            tp = turtlebot.location.point
            ti = points.index(tp)
            d = costs[ti,bi] + turtlebot.acum_cost
            if d < dist:
                persons_turtlebot = turtlebot
                dist = d

        turtlebot = persons_turtlebot
        turtlebot.acum_cost += dist - turtlebot.acum_cost
        # assign turtlebot to person

        # find drone closest to box
        dist = np.inf
        drone_box = None
        for drone in filter(lambda x: x.agent_type == 'drone', mworld.agents):
            dp = drone.location.point
            di = points.index(dp)
            d = costs[di,bi] + drone.acum_cost
            if d < dist:
                drone_box = drone 
                dist = d

        drone_box.acum_cost += dist - drone_box.acum_cost
        action_list.add(PickUp([drone_box.name, box.name, box.location.name + '_air', box.location.name], 0, mworld))
        #action_list.add(HandOver([turtlebot.name, drone_box.name, box.name, box.location.name + '_air', box.location.name], 0, mworld))
        action_list.add(HandOver([drone_box.name, turtlebot.name, box.name, box.location.name + '_air', box.location.name], 0, mworld))

        # find drone clostest to person
        dist = np.inf
        drone_person = None
        for drone in filter(lambda x: x.agent_type == 'drone', mworld.agents):
            dp = drone.location.point
            di = points.index(dp)
            d = costs[di,pi] + drone.acum_cost
            if d < dist:
                drone_person = drone 
                dist = d

        drone_person.acum_cost += dist - drone_person.acum_cost
        action_list.add(HandOver(
            [turtlebot.name, drone_person.name, box.name, person.location.name + '_air', person.location.name], 0, mworld))
        action_list.add(Deliver(
            [drone_person.name, box.name, person.location.name + '_air', person.location.name, person.name], 0, mworld))
        
        drone_box.location = box.location
        drone_person.location = person.location
        turtlebot.location = person.location

    for edge in mworld.edges:
        for turtlebot in filter(lambda x: x.agent_type == 'turtlebot', mworld.agents):
            action_list.add(Move([turtlebot.name, edge.p1.name, edge.p2.name], 0, mworld))
            action_list.add(Move([turtlebot.name, edge.p2.name, edge.p1.name], 0, mworld))
        for drone in filter(lambda x: x.agent_type == 'drone', mworld.agents):
            action_list.add(Move([drone.name, edge.p1.name + '_air', edge.p2.name + '_air'], 0, mworld))
            action_list.add(Move([drone.name, edge.p2.name + '_air', edge.p1.name + '_air'], 0, mworld))
    return action_list

def wp_cost(world, map):
    points = world.points()
    from scipy.sparse import dok_matrix, csr_matrix
    cs_graph = dok_matrix((len(points), len(points)))
    for i,wp1 in enumerate(points):
        cs_graph[i,i] = 0
        for neighbor in map.neighbors[i]:
            wp2 = points[neighbor]
            d = np.hypot(wp1[0]-wp2[0], wp1[1]-wp2[1])
            cs_graph[i,neighbor] = d
            cs_graph[neighbor,i] = d

    from scipy.sparse.csgraph import shortest_path
    matrix = csr_matrix(cs_graph)
    return shortest_path(matrix, directed = False)

if __name__ == "__main__":

    use_old = False
    if len(sys.argv) > 1:
        print("info: using old plan")
        use_old = True

    initial_state_json = '../../data/mapToDemo3.json'
    grid = OccupancyGrid.from_pgm('../../data/mapToDemo3')

    world = World.from_json(initial_state_json)

    map = Map(grid, world.points())

    world.add_map_info(map)

    if not use_old:
        planner_file = generate_plan(world, map)
    else:
        planner_file = sys.argv[1]

    actions = parse_plan(planner_file, world)
    land = Land(['drone1'], 1, world)
    land.index = len(actions)
    actions.append(land)
    #takeoff = TakeOff(['drone1'], -1, world)
    #actions.insert(0, takeoff)
    graph = construct_graph(actions, world)

    coordinator.start(graph, actions)
