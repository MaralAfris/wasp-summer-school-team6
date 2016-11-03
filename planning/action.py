#!/usr/bin/env python

import sys,re
from world import World, Box, Agent, Person, Waypoint

def construct_graph(plan_file, world):
    actions = []
    with open(plan_file, 'r') as f:
        for line in f:
            action = from_plan(line, world)
            if action is not None:
                actions.append(action)
    actions.reverse()
    root = Root(world)
    for action in actions:
        for dep in root.edges:
            if action.depends(dep):
                root.remove_edge(dep)
                action.add_edge(dep)
        root.add_edge(action)

    return root


def from_plan(line, world):
    planre = re.compile('[0-9.]+: \(([_a-z0-9- ]+)\) \[[0-9]+\]')
    m = planre.match(line)
    if m is not None:
        group = m.group(1)
        parts = group.split(" ")
        args = parts[1:len(parts)]
        action = parts[0]
        if action == 'move':
            return Move(args, world)
        elif action == 'deliver':
            return Deliver(args, world)
        elif action == 'pick-up':
            return PickUp(args, world)
        elif action == 'hand-over':
            return HandOver(args, world)
    return None


class Action(object):
    def __init__(self, args, world):
        self.symbols = set(args)
        self.args = args
        self.edges = []

    def add_edge(self, action):
        self.edges.append(action)

    def remove_edge(self, action):
        self.edges.remove(action)

    def depends(self, other):
        for s in self.symbols:
            if other.has_symbol(s):
                return True
        return False

    def has_symbol(self, s):
        return s in self.symbols

    def format(self):
        return self.__class__.__name__ + ' ' + ', '.join(self.args)

    def print_deps(self):
        print(self.format())
        for a in self.edges:
            a.print_deps()

class Root(Action):
    def __init__(self, world):
        super(Root, self).__init__([], world)

class Move(Action):
    def __init__(self, args, world):
        super(Move, self).__init__(args, world)
        self.agent = world.agent(args[0])
        self.to = world.waypoint(args[2])

    def complete_action(self):
        self.agent.location = self.to.name

class Deliver(Action):
    def __init__(self, args, world):
        super(Deliver, self).__init__(args, world)
        self.agent = world.agent(args[0])
        self.box = world.box(args[1])
        self.person = world.person(args[len(args)-1])

    def complete_action(self):
        self.agent.carrying = None
        self.box.free = False
        self.box.location = agent.location
        self.person.handled = True

class PickUp(Action):
    def __init__(self, args, world):
        super(PickUp, self).__init__(args, world)
        self.agent = world.agent(args[0])
        self.box = world.box(args[1])

    def complete_action(self):
        self.agent.carrying = self.box.name
        self.box.location = None
        self.box.free = False

class HandOver(Action):
    def __init__(self, args, world):
        super(HandOver, self).__init__(args, world)
        self.drone = world.agent(args[0])
        self.turtlebot = world.agent(args[1])
        self.box = world.box(args[2])

    def complete_action(self):
        self.turtlebot.carrying = self.drone.carrying
        self.drone.carrying = None


if __name__ == "__main__":
    world = World.from_json(sys.argv[1])
    world.create_triangulation()
    graph = construct_graph(sys.argv[2], world)
    graph.print_deps()

