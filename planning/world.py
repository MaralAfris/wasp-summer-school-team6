#!/usr/bin/env python

import sys

TURTLE_SPEED = 0.5 
DRONE_SPEED = 0.1

class World:

    def __init__(self, agents, waypoints, boxes, persons):
        self.agents = agents
        self.waypoints = waypoints
        self.boxes = boxes
        self.persons = persons

    @classmethod
    def from_json(cls, json_file):
        import json
        with open(json_file) as data_file:    
            data = json.load(data_file)

        agents = []
        waypoints = []
        boxes = []
        persons = []

        for agent in data["agents"]:
            agents.append(Agent(agent["name"], agent["agent_type"], agent["location"], agent["carrying"]))

        for waypoint in data["waypoints"]:
            waypoints.append(Waypoint(waypoint["name"], waypoint["x"], waypoint["y"]))

        for box in data["boxes"]:
            boxes.append(Box(box["name"], box["location"], box["free"]))

        for person in data["persons"]:
            persons.append(Person(person["name"], person["location"], person["handled"]))

        obj = cls(agents, waypoints, boxes, persons)
        return obj

    def generate_problem(self, problem_file):
        import math
        f = open(problem_file, 'w')
        f.write('(define (problem emergency-template)\n')
        f.write('(:domain emergency)\n')
        f.write('(:objects\n')
        for agent in self.agents:
            f.write('  ' + agent.name + ' - ' + agent.agent_type + '\n')

        empty_waypoint_set = set()
        occupied_waypoint_set = set()
        f.write('  ')
        for waypoint in self.waypoints:
            f.write(waypoint.name + ' ')
            empty_waypoint_set.add(waypoint.name)
        f.write('- waypoint\n')

        f.write('  ')
        for waypoint in self.waypoints:
            f.write(waypoint.name + '_air' + ' ')
            empty_waypoint_set.add(waypoint.name + '_air')
        f.write('- airwaypoint\n')

        f.write('  ')
        for box in self.boxes:
            f.write(box.name + ' ')
        f.write('- box\n')

        f.write('  ')
        for person in self.persons:
            f.write(person.name + ' ')
        f.write('- person)\n')

        f.write('\n')
        f.write('(:init\n')

        for waypoint in self.waypoints:
            for connection in waypoint.connections:
                dist = math.sqrt((waypoint.x - connection.x)**2 + (waypoint.y - connection.y)**2)
                tdist = int(math.ceil(dist / TURTLE_SPEED))
                ddist = int(math.ceil(dist / DRONE_SPEED))
                f.write('  (= (move-duration ' + waypoint.name + ' ' + connection.name + ') ' + str(tdist) + ')\n')
                f.write('  (= (move-duration ' + waypoint.name + '_air ' + connection.name + '_air) ' + str(ddist) + ')\n')
            f.write('\n')

        for waypoint in self.waypoints:
            f.write('  (over ' + waypoint.name + '_air ' + waypoint.name + ')\n')
        f.write('\n')

        for box in self.boxes:
            if box.free:
                f.write('  (free ' + box.name+')\n')
                f.write('  (at ' + box.name + ' ' + box.location + ')\n')
        f.write('\n')

        for agent in self.agents:
            if agent.carrying is None:
                f.write('  (empty ' + agent.name + ')\n')
            else:
                f.write('  (carrying ' + agent.name + ' ' + agent.carrying + ')\n')
            if agent.agent_type == 'drone':
                l = agent.location + '_air'
            else:
                l = agent.location
            f.write('  (at ' + agent.name + ' ' + l + ')\n')
            f.write('\n')
            empty_waypoint_set.discard(l)
            occupied_waypoint_set.add(l)
        f.write('\n')

        for w in empty_waypoint_set:
            f.write('  (empty ' + w + ')\n')
        f.write('\n')

        for w in occupied_waypoint_set:
            f.write('  (occupied ' + w + ')\n')
        f.write('\n')

        for person in self.persons:
            f.write('  (at ' + person.name + ' ' + person.location + ')\n')
        f.write(')\n')
        f.write('\n')

        f.write('(:goal (and\n')
        for person in self.persons:
            if person.handled == False:
                f.write('            (handled ' + person.name + ')\n')
        f.write(')))\n')
        

    def create_triangulation(self):
        from scipy.spatial import Delaunay
        import numpy as np
        array = [];
        for waypoint in self.waypoints:
            array.append([waypoint.x, waypoint.y])
        points = np.array(array)
        tri = Delaunay(points)
        for triangle in tri.simplices:
            a = self.waypoints[triangle[0]]
            b = self.waypoints[triangle[1]]
            c = self.waypoints[triangle[2]]
            a.add_connection(b)
            a.add_connection(c)
            b.add_connection(a)
            b.add_connection(c)
            c.add_connection(a)
            c.add_connection(b)

    def plot(self):
        import matplotlib.pyplot as plt
        xs = []
        ys = []
        for w in self.waypoints:
            xs.append(w.x)
            ys.append(w.y)
            plt.annotate(w.name, xy=(w.x*1.01, w.y*1.01))
            for c in w.connections:
                plt.plot([w.x, c.x], [w.y, c.y])
        plt.plot(xs, ys, 'o')
        plt.axis('equal')
        plt.show()


class Agent:
    def __init__(self, name, agent_type, location, carrying):
        self.name = name
        self.agent_type = agent_type
        self.location = location
        self.carrying = carrying

class Waypoint:
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y
        self.connections = []
    
    def add_connection(self, waypoint):
        self.connections.append(waypoint)

class Box:
    def __init__(self, name, location, free):
        self.name = name
        self.location = location
        self.free = free

class Person:
    def __init__(self, name, location, handled):
        self.name = name
        self.location = location
        self.handled = handled

def main(argv):
    import getopt
    inputfile = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
        print 'world.py -i <inputfile> -o <outputfile>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'world.py -i <inputfile> -o <outputfile>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    if not inputfile:
        inputfile = 'problems/big.json'
    if not outputfile:
        outputfile = 'out.pddl'
    world = World.from_json(inputfile)
    world.create_triangulation()
    world.generate_problem(outputfile)
    world.plot()


if __name__ == "__main__":
    main(sys.argv[1:])
