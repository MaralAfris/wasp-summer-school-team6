import sys,re
from world import World, Box, Agent, Person, Waypoint

def construct_graph(plan_file, world):
    actions = []
    ix = 1
    with open(plan_file, 'r') as f:
        for line in f:
            action = from_plan(line, world)
            if action is not None:
                actions.append(action)
                action.index = ix
                ix += 1

    actions.reverse()

    root = Root(world)
    root.index = 0

    for action in actions:
        add_deps(root, root, action)
        root.add_succ(action)
    root.succ.reverse()

    actions.reverse()
    for action in actions:
        action.succ.sort(key=lambda action: action.index)
        for succ in action.succ:
            succ.add_pre(action)
        action.pre.sort(key=lambda action: action.index)

    return (root, actions)

def add_deps(root, root_child, new_node):
    for su in list(root_child.succ):
        if su.overlaps(new_node):
            root.remove_succ(su)
            new_node.add_succ(su)
        else:
            add_deps(root, su, new_node)

def from_plan(line, world):
    planre = re.compile('[0-9.]+: \(([_a-z0-9- ]+)\) \[([0-9.]+)\]')
    m = planre.match(line)
    if m is not None:
        group = m.group(1)
        duration = float(m.group(2))
        parts = group.split(" ")
        args = parts[1:len(parts)]
        action = parts[0]
        if action == 'move':
            return Move(args, duration, world)
        elif action == 'deliver':
            return Deliver(args, duration, world)
        elif action == 'pick-up':
            return PickUp(args, duration, world)
        elif action == 'hand-over':
            return HandOver(args, duration, world)
    return None


class Action(object):
    def __init__(self, args, duration, world):
        self.duration = duration
        self.symbols = set(args)
        self.args = args
        self.succ = []
        self.pre = []

    def add_succ(self, action):
        if action not in self.succ:
            self.succ.append(action)

    def add_pre(self, action):
        if action not in self.pre:
            self.pre.append(action)

    def remove_succ(self, action):
        if action in self.succ:
            self.succ.remove(action)

    def overlaps(self, other):
        return not self.symbols.isdisjoint(other.symbols) 

    def has_symbol(self, s):
        return s in self.symbols

    def format(self):
        deps = []
        for e in self.pre:
            deps.append(str(e.index))

        return str(self.index) + ' [' + ', '.join(deps) + '] ' + \
                self.__class__.__name__ + ' ' + ', '.join(self.args)

class Root(Action):
    def __init__(self, world):
        super(Root, self).__init__([], 0, world)

    def overlaps(self, other):
        return True

class Move(Action):
    def __init__(self, args, duration, world):
        super(Move, self).__init__(args, duration, world)
        self.agent = world.agent(args[0])
        self.to = world.waypoint(args[2])

    def complete_action(self):
        self.agent.location = self.to.name

class Deliver(Action):
    def __init__(self, args, duration, world):
        super(Deliver, self).__init__(args, duration, world)
        self.agent = world.agent(args[0])
        self.box = world.box(args[1])
        self.person = world.person(args[len(args)-1])

    def complete_action(self):
        self.agent.carrying = None
        self.box.free = False
        self.box.location = self.agent.location
        self.person.handled = True

class PickUp(Action):
    def __init__(self, args, duration, world):
        super(PickUp, self).__init__(args, duration, world)
        self.agent = world.agent(args[0])
        self.box = world.box(args[1])

    def complete_action(self):
        self.agent.carrying = self.box.name
        self.box.location = None
        self.box.free = False

class HandOver(Action):
    def __init__(self, args, duration, world):
        super(HandOver, self).__init__(args, duration, world)
        self.drone = world.agent(args[0])
        self.turtlebot = world.agent(args[1])
        self.box = world.box(args[2])

    def complete_action(self):
        self.turtlebot.carrying = self.drone.carrying
        self.drone.carrying = None

