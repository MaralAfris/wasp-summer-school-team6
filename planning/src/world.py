import sys
from settings import *
from map import *
from copy import deepcopy
import numpy as np

class World(object):
    
    def __init__(self, agents, waypoints, boxes, persons):
        self.agents = agents
        self.waypoints = waypoints
        self.boxes = boxes
        self.persons = persons

    def add_map_info(self, map):
        for point,relocate in map.moved_points.iteritems():
            for waypoint in self.waypoints:
                if waypoint.point == point:
                    waypoint.point = relocate

        wp_as_nodes = set()
        for wp in self.waypoints:
            wp_as_nodes.add(wp.point)

        mesh = 1
        for point in map.nodes:
            if point not in wp_as_nodes:
                self.waypoints.append(Waypoint('mesh'+str(mesh), point, PointType.mesh))
                mesh += 1

        self.edges = []
        visited = set()

        for i,node in enumerate(map.nodes):
            for j in map.neighbors[i]:
                neighbor = map.nodes[j]
                if (i,j) in visited:
                    continue
                visited.add((i,j))
                visited.add((j,i))
                self.edges.append(Edge(self.point_to_waypoint(node), self.point_to_waypoint(neighbor)))

    @classmethod
    def from_json(cls, json_file):
        import json
        with open(json_file) as data_file:
            data = json.load(data_file)

        agents = []
        waypoints = []
        boxes = []
        persons = []
        waypoint_dict = {}

        names = set()
        def check_name(name):
            if name in names:
                raise Exception("names must be unique among all objects (yahsp3 limitation)" \
                        + ", offender: " + name)
            names.add(name)

        def get_location(data, source):
            if isinstance(data, basestring):
                return waypoint_dict[data]
            else:
                data = (data[0], data[1])
                for wp in waypoints:
                    if np.hypot(data[0]-wp.point[0],data[1]-wp.point[1]) \
                             < settings.mesh['initial_min_dist']:
                        print('info: joining waypoint ' + str(data) + ' with ' + str(wp.point))
                        return wp
                wp = Waypoint(source + "_wp", data, PointType.location)
                waypoints.append(wp)
                return wp

        if "waypoints" in data:
            for waypoint in data["waypoints"]:
                x = waypoint["x"]
                y = waypoint["y"]
                if "type" in waypoint:
                    waypoint_type = PointType.from_string(waypoint["type"])
                else:
                    waypoint_type = PointType.initial
                name = waypoint["name"]
                wp = Waypoint(name, (x, y), PointType.initial)
                waypoints.append(wp)
                waypoint_dict[name] = wp
                check_name(name)

        for agent in data["agents"]:
            if "carrying" not in agent:
                carrying = None
            else:
                carrying = agent["carrying"]
            name = agent["name"]
            agent_type = agent["agent_type"]
            location = get_location(agent["location"], name)
            agents.append(Agent(name, agent_type, location, carrying))

        for box in data["boxes"]:
            name = box["name"]
            location = get_location(box["location"], name)
            boxes.append(Box(name, location, box["free"]))

        for person in data["persons"]:
            name = person["name"]
            location = get_location(person["location"], name)
            persons.append(Person(name, location, person["handled"]))

        # waypoints must be sorted to make virtual waypoints consistently generated
        waypoints.sort(key=lambda waypoint: waypoint.name)

        obj = cls(agents, waypoints, boxes, persons)
        obj.to_json('test.json')
        return obj

    def points(self):
        points = []
        for waypoint in self.waypoints:
            points.append(waypoint.point)
        return points

    def to_json(self, json_file):
        import json
        data = {}

        data['agents'] = self._list_as_dict(self.agents)
        data['waypoints'] = self._list_as_dict(filter(lambda wp: wp.point_type == PointType.initial, self.waypoints))
        data['boxes'] = self._list_as_dict(self.boxes)
        data['persons'] = self._list_as_dict(self.persons)

        j = json.dumps(data, indent=4)
        f = open(json_file, 'w')
        print >> f, j
        f.close()

    def agent(self, name):
        for agent in self.agents:
            if agent.name == name:
                return agent
        raise Exception('unknown agent ' + name)

    def point_to_waypoint(self, point):
        for waypoint in self.waypoints:
            if waypoint.point == point:
                return waypoint
        raise Exception('point not found: ' + str(point))

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

        def print_edge(fname, wp1, wp2, dist):
            f.write('  (= (' + fname + ' ' + wp1.name + \
                    ' ' + wp2.name + ') ' + str(dist) + ')\n')

        for edge in self.edges:
            dist = edge.p1.dist(edge.p2)
            tdist = settings.plan['turtle_delay'] + dist / settings.plan['turtle_speed']
            ddist = settings.plan['drone_delay'] + dist / settings.plan['drone_speed']
            dcdist = settings.plan['drone_delay'] + dist / settings.plan['drone_carry_speed']
            print_edge('move-duration', edge.p1, edge.p2, tdist)
            print_edge('move-duration', edge.p2, edge.p1, tdist)
            print_edge('fly-duration', edge.p1.as_air(), edge.p2.as_air(), ddist)
            print_edge('fly-duration', edge.p2.as_air(), edge.p1.as_air(), ddist)
            print_edge('fly-carry-duration', edge.p1.as_air(), edge.p2.as_air(), dcdist)
            print_edge('fly-carry-duration', edge.p2.as_air(), edge.p1.as_air(), dcdist)
        f.write('\n')

        for waypoint in self.waypoints:
            f.write('  (over ' + waypoint.as_air().name + ' ' + waypoint.name + ')\n')
        f.write('\n')

        for box in self.boxes:
            if box.free:
                f.write('  (free ' + box.name+')\n')
                f.write('  (at ' + box.name + ' ' + box.location.name + ')\n')
        f.write('\n')

        for agent in self.agents:
            if agent.carrying is None:
                f.write('  (empty ' + agent.name + ')\n')
            else:
                f.write('  (carrying ' + agent.name + ' ' + agent.carrying + ')\n')
            if agent.agent_type == 'drone':
                l = agent.location.as_air().name
            else:
                l = agent.location.name
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
            f.write('  (at ' + person.name + ' ' + person.location.name + ')\n')
        f.write(')\n')
        f.write('\n')

        f.write('(:goal (and\n')
        for person in self.persons:
            if person.handled == False:
                f.write('            (handled ' + person.name + ')\n')
        f.write(')))\n')

        f.close()

    def _list_as_dict(self, alist):
        new_list = []
        for a in alist:
            new_list.append(a.dict())
        return new_list

class JsonSerializable(object):
    def dict(self):
        return self.__dict__

    def location_json(self, wp):
        if wp is None:
            return None
        if wp.point_type == PointType.initial:
            return wp.name
        return list(wp.point)

class Agent(JsonSerializable):
    def __init__(self, name, agent_type, location, carrying):
        self.name = name
        self.agent_type = agent_type
        self.location = location
        self.carrying = carrying

    def dict(self):
        return {
                'name': self.name,
                'agent_type': self.agent_type,
                'location': self.location_json(self.location),
                'carrying': self.carrying}

class Waypoint(JsonSerializable):
    def __init__(self, name, point, point_type):
        assert type(point) == tuple
        self.name = name
        self.point = point
        self.point_type = point_type

    def as_air(self):
        return Waypoint(self.name + '_air', self.point, PointType.air)

    def dict(self):
        return {'name': self.name, 'x': self.point[0], 'y': self.point[1]}

    def dist(self, other):
        return np.hypot(self.point[0]-other.point[0], self.point[1]-other.point[1])

class PointType(object):
    # initial is a waypoint defined in waypoint list
    # location is an infered waypoint based on agent/box/person location
    # mesh is a generated waypoint from calculations
    initial, location, mesh, air = range(4)

    @classmethod
    def from_string(cls, string):
        print "waypoint type", string
        if string == "initial":
            return PointType.initial
        if string == "location":
            return PointType.location
        if string == "mesh":
            return PointType.mesh
        if string == "air":
            return PointType.air
        raise Exception("Unknown point type " + string)

class Box(JsonSerializable):
    def __init__(self, name, location, free):
        self.name = name
        self.location = location
        self.free = free

    def dict(self):
        return {
                'name': self.name,
                'location': self.location_json(self.location),
                'free': self.free}

class Person(JsonSerializable):
    def __init__(self, name, location, handled):
        self.name = name
        self.location = location
        self.handled = handled

    def dict(self):
        return {
                'name': self.name,
                'location': self.location_json(self.location),
                'handled': self.handled}

class Edge(object):
    def __init__(self, p1, p2):
        assert type(p1) == Waypoint
        assert type(p2) == Waypoint
        self.p1 = p1
        self.p2 = p2
