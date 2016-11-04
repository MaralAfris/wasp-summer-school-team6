#!/usr/bin/env python

from scipy.spatial import Delaunay
import numpy as np
import sys

TURTLE_SPEED = 1 
DRONE_SPEED = 1

MIN_TRIANGLE_SPLIT = 1
VIRTUAL_WAYPOINT_PREFIX = 'vt_'
MAX_MESH_SIZE = 500

class World(object):
    
    def __init__(self, agents, waypoints, boxes, persons):
        self.agents = agents
        self.waypoints = waypoints
        self.boxes = boxes
        self.persons = persons
        self._create_mesh()

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

        # waypoints must be sorted to make virtual waypoints consistently generated
        waypoints.sort(key=lambda waypoint: waypoint.name)

        for box in data["boxes"]:
            boxes.append(Box(box["name"], box["location"], box["free"]))

        for person in data["persons"]:
            persons.append(Person(person["name"], person["location"], person["handled"]))

        obj = cls(agents, waypoints, boxes, persons)
        return obj

    def to_json(self, json_file):
        import json
        data = {}

        data['agents'] = _list_as_dict(self.agents)
        data['waypoints'] = _list_as_dict(self.waypoints)
        data['boxes'] = _list_as_dict(self.boxes)
        data['persons'] = _list_as_dict(self.persons)

        j = json.dumps(data, indent=4)
        f = open(json_file, 'w')
        print >> f, j
        f.close()


    def agent(self, name):
        for agent in self.agents:
            if agent.name == name:
                return agent
        raise Exception('unknown agent ' + name)

    def waypoint(self, name):
        for waypoint in self.waypoints:
            if waypoint.name == name or waypoint.name + '_air' == name:
                return waypoint 
        raise Exception('unknown waypoint ' + name)

    def box(self, name):
        for box in self.boxes:
            if box.name == name:
                return box 
        raise Exception('unknown box ' + name)

    def person(self, name):
        for person in self.persons:
            if person.name == name:
                return person 
        raise Exception('unknown person ' + name)

    def generate_problem(self, problem_file):
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
                dist = waypoint.dist(connection)
                tdist = dist / TURTLE_SPEED
                ddist = dist / DRONE_SPEED
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

        f.close()
        

    def _create_mesh(self):
        self._split_triangles(set())
        self._create_connections()
        names = set()
        for waypoint in self.waypoints:
            if waypoint.name in names:
                raise Exception('duplicate name ' + waypoint.name)
            names.add(waypoint.name)

    def _triangulate(self):
        array = [];
        for waypoint in self.waypoints:
            array.append([waypoint.x, waypoint.y])
        points = np.array(array)
        return Delaunay(points)

    def _split_triangles(self, waypoint_names):
        tri = self._triangulate()
        any_split = False
        for triangle in tri.simplices:
            a = self.waypoints[triangle[0]]
            b = self.waypoints[triangle[1]]
            c = self.waypoints[triangle[2]]
            if self._split_triangle(a, b, c, waypoint_names):
                any_split = True
        if any_split:
            self._split_triangles(waypoint_names)

    def _split_triangle(self, a, b, c, waypoint_names):
        x = (a.x + b.x + c.x) / 3
        y = (a.y + b.y + c.y) / 3
        print x,y
        grid = 10 / MIN_TRIANGLE_SPLIT
        name = VIRTUAL_WAYPOINT_PREFIX + str(int(x*grid)) + '_' + str(int(y*grid))
        centroid = Waypoint(name, x, y)
        if (name not in waypoint_names and 
                a.dist(centroid) > MIN_TRIANGLE_SPLIT and
                b.dist(centroid) > MIN_TRIANGLE_SPLIT and
                c.dist(centroid) > MIN_TRIANGLE_SPLIT):
            self.waypoints.append(centroid)
            waypoint_names.add(name)
            return True
        return False

    def _create_connections(self):
        tri = self._triangulate()
        for triangle in tri.simplices:
            a = self.waypoints[triangle[0]]
            b = self.waypoints[triangle[1]]
            c = self.waypoints[triangle[2]]
            a.add_connection(b)
            b.add_connection(a)
            a.add_connection(c)
            c.add_connection(a)
            b.add_connection(c)
            c.add_connection(b)
        for triangle in tri.simplices:
            a = self.waypoints[triangle[0]]
            b = self.waypoints[triangle[1]]
            c = self.waypoints[triangle[2]]
            self._prune(a, b, c)
            self._prune(a, c, b)
            self._prune(b, c, a)

    def _prune(self, a, b, c):
        x = (a.x + b.x) / 2
        y = (a.y + b.y) / 2
        if Waypoint('', x, y).dist(c) < MIN_TRIANGLE_SPLIT and len(a.connections) > 1 and len(b.connections) > 1:
            a.connections.remove(b)
            b.connections.remove(a)

    def plot(self):
        import matplotlib.pyplot as plt
        xs = []
        ys = []
        for w in self.waypoints:
            if not w.name.startswith(VIRTUAL_WAYPOINT_PREFIX):
                plt.annotate(w.name, xy=(w.x*1.01, w.y*1.01))
                xs.append(w.x)
                ys.append(w.y)
            for c in w.connections:
                plt.plot([w.x, c.x], [w.y, c.y])
        plt.plot(xs, ys, 'o')
        plt.axis('equal')
        plt.show()

def _list_as_dict(alist):
    new_list = []
    for a in alist:
        new_list.append(a.dict())
    return new_list


class JsonSerializable(object):
    def dict(self):
        return self.__dict__

class Agent(JsonSerializable):
    def __init__(self, name, agent_type, location, carrying):
        self.name = name
        self.agent_type = agent_type
        self.location = location
        self.carrying = carrying

class Waypoint(JsonSerializable):
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y
        self.connections = []
    
    def add_connection(self, waypoint):
        self.connections.append(waypoint)

    def dist(self, waypoint):
        return np.sqrt((self.x - waypoint.x)**2 + (self.y - waypoint.y)**2)

    def dict(self):
        return {'name': self.name, 'x': self.x, 'y': self.y}

class Box(JsonSerializable):
    def __init__(self, name, location, free):
        self.name = name
        self.location = location
        self.free = free

class Person(JsonSerializable):
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
    world.generate_problem(outputfile)
    world.plot()


if __name__ == "__main__":
    main(sys.argv[1:])
