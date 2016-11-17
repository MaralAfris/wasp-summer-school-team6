import sys
from settings import *
from map2 import *
from copy import deepcopy
from enum import Enum
import numpy as np

class World(object):
    
    def __init__(self, agents, waypoints, boxes, persons):
        self.agents = agents
        self.waypoints = waypoints
        self.boxes = boxes
        self.persons = persons

    def add_map_info(self, map):
        self.edges = []

        wp_as_nodes = set()
        mesh = 1
        for point in map.nodes:
            if point not in wp_as_nodes:
                self.waypoints.append(Waypoint('mesh'+str(mesh), point, PointType.mesh))
                mesh += 1
        self.edges = []
        visited = set()
        for i,nbs in enumerate(map.neighbors):
            for node in nbs:
                if (i,node) in visited:
                    continue
                visited.add((i,node))
                visited.add((node,i))
                self.edges.append(Edge(self.waypoints[i], self.waypoints[node]))
        #print visited
        #print self.edges
            #self.neighbors = []
            #wp_edge = deepcopy(edge)
            #wp_edge.wp1 = next(p for p in self.waypoints if p.point == edge.p1)
            #wp_edge.wp2 = next(p for p in self.waypoints if p.point == edge.p2)
            #self.edges.append(wp_edge)

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
            agents.append(Agent(agent["name"], agent["agent_type"], 
                agent["location"], agent["carrying"]))

        for waypoint in data["waypoints"]:
            x = waypoint["x"]
            y = waypoint["y"]
            waypoints.append(Waypoint(waypoint["name"], (x, y), PointType.initial))

        # waypoints must be sorted to make virtual waypoints consistently generated
        waypoints.sort(key=lambda waypoint: waypoint.name)

        for box in data["boxes"]:
            boxes.append(Box(box["name"], box["location"], box["free"]))

        for person in data["persons"]:
            persons.append(Person(person["name"], person["location"], 
                person["handled"]))

        obj = cls(agents, waypoints, boxes, persons)
        return obj

    def to_json(self, json_file):
        import json
        data = {}

        data['agents'] = _list_as_dict(self.agents)
        data['waypoints'] = _list_as_dict(filter(lambda wp: wp.point_type == PointType.initial, self.waypoints))
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
            f.write(waypoint.as_air().name + ' ')
            empty_waypoint_set.add(waypoint.as_air().name)
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

        def print_edge(wp1, wp2, dist):
            f.write('  (= (move-duration ' + wp1.name + \
                    ' ' + wp2.name + ') ' + str(dist) + ')\n')

        for edge in self.edges:
            dist = edge.p1.dist(edge.p2)
            tdist = dist / settings.plan['turtle_speed']
            ddist = dist / settings.plan['drone_speed']
            print_edge(edge.p1, edge.p2, tdist)
            print_edge(edge.p2, edge.p1, tdist)
            print_edge(edge.p1.as_air(), edge.p2.as_air(), ddist)
            print_edge(edge.p2.as_air(), edge.p1.as_air(), ddist)
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
    def __init__(self, name, point, point_type):
        assert type(point) == tuple
        assert type(point_type) == PointType
        self.name = name
        self.point = point
        self.point_type = point_type

    def as_air(self):
        return Waypoint(self.name + '_air', self.point, self.point_type)

    def dict(self):
        return {'name': self.name, 'x': self.point[0], 'y': self.point[1]}

    def dist(self, other):
        return np.hypot(self.point[0]-other.point[0], self.point[1]-other.point[1])

class PointType(Enum):
    initial = 1
    mesh = 2

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

class Edge(object):
    def __init__(self, p1, p2):
        assert type(p1) == Waypoint
        assert type(p2) == Waypoint
        self.p1 = p1
        self.p2 = p2
