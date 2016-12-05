import sys,re
from world import World, Box, Agent, Person, Waypoint
import settings
from coordinator import *

def parse_plan(plan_file, world):
    actions = []
    ix = 1
    with open(plan_file, 'r') as f:
        for line in f:
            action = from_plan(line, world)
            if action is not None:
                actions.append(action)
                action.index = ix
                ix += 1
    return actions

def construct_graph(actions, world):
    actions.reverse()

    symbol_index = {}
    for action in actions:
        add_deps(symbol_index, action)
        #root.add_succ(action)

    actions.reverse()
    for action in actions:
        action.succ = list(action.succ)
        action.succ.sort(key=lambda action: action.index)
        for succ in action.succ:
            succ.add_pre(action)
        action.pre = list(action.pre)
        action.pre.sort(key=lambda action: action.index)

    root = Root(world)
    root.index = 0
    for action in actions:
        if not action.pre:
            root.add_succ(action)
            #action.pre.append(root)
    root.succ = list(root.succ)
    root.succ.sort(key=lambda action: action.index)

    return root

# Adds delay actions before move actions, to simulate path planning calculations.
def add_delays(actions, world):
    action_copy = []
    index = 1
    for action in actions:
        if isinstance(action, Move):
            if action.agent.agent_type == 'turtlebot':
                dur = settings.plan['turtle_delay']
            else:
                dur = settings.plan['drone_delay']
            d = Delay(action.args, dur, world)
            d.index = index
            action_copy.append(d)
            index += 1
        action.index = index
        action_copy.append(action)
        index += 1
    return action_copy

def add_deps(symbol_index, new_node):
    for symbol in new_node.symbols:
        if symbol in symbol_index:
            node = symbol_index[symbol]
            new_node.add_succ(node)
    for symbol in new_node.symbols:
        symbol_index[symbol] = new_node

def from_plan(line, world):
    planre = re.compile('[0-9.]+: \(([_a-z0-9- ]+)\) \[([0-9.]+)\]')
    m = planre.match(line)
    if m is not None:
        group = m.group(1)
        duration = float(m.group(2))
        parts = group.split(" ")
        args = parts[1:len(parts)]
        action = parts[0]
        if action == 'move' or action == 'fly':
            return Move(args, duration, world)
        elif action == 'deliver':
            return Deliver(args, duration, world)
        elif action == 'pick-up':
            return PickUp(args, duration, world)
        elif action == 'hand-over':
            return HandOver(args, duration, world)
        else:
            raise Exception('unknown action: ' + action)
    return None


class Action(object):
    def __init__(self, args, duration, world):
        self.duration = duration
        self.symbols = set(args)
        self.args = args
        self.succ = set()
        self.pre = set()
        self.completed = False

    def add_succ(self, action):
        self.succ.add(action)

    def add_pre(self, action):
        self.pre.add(action)

    def has_symbol(self, s):
        return s in self.symbols

    def format(self):
        deps = []
        for e in self.pre:
            deps.append(str(e.index))

        return str(self.index) + ' [' + ', '.join(deps) + '] ' + \
                self.__class__.__name__ + ' ' + ', '.join(self.args)

    def complete_action(self):
        pass

class Root(Action):
    def __init__(self, world):
        super(Root, self).__init__([], 0, world)

class Move(Action):
    def __init__(self, args, duration, world):
        super(Move, self).__init__(args, duration, world)
        self.agent = world.agent(args[0])
        self.start = world.waypoint(args[1])
        self.to = world.waypoint(args[2])

    def complete_action(self):
        self.agent.location = self.to
        self.completed = True

    def execute(self, turtlebot_publisher, drone_publisher):
        x,y = self.to.point
        if self.agent.agent_type == "turtlebot":
            moveTurtleBot(x, y, 0, self.index, turtlebot_publisher)
        else:
            moveTurtleBot(x, y, 0, self.index, drone_publisher)

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
        self.completed = True

    def execute(self, turtlebot_publisher, drone_publisher):
        # TODO impement me!
        pass

class PickUp(Action):
    def __init__(self, args, duration, world):
        super(PickUp, self).__init__(args, duration, world)
        self.agent = world.agent(args[0])
        self.box = world.box(args[1])

    def complete_action(self):
        self.agent.carrying = self.box.name
        self.box.location = None
        self.box.free = False
        self.completed = True

    def execute(self, publisher,drone_publisher):
        # TODO impement me!
        pass

class HandOver(Action):
    def __init__(self, args, duration, world):
        super(HandOver, self).__init__(args, duration, world)
        self.drone = world.agent(args[0])
        self.turtlebot = world.agent(args[1])
        self.box = world.box(args[2])

    def complete_action(self):
        self.turtlebot.carrying = self.drone.carrying
        self.drone.carrying = None
        self.completed = True

    def execute(self, publisher,drone_publisher):
        # TODO impement me!
        pass

# Delays simulate the time it takes turtlebot/drone to generate new path.
# We dont use this action during actual planning, but instead add it to
# move cost, otherwise planning would be more complex.
# This action is only used during simulation.
class Delay(Action):
    def __init__(self, args, duration, world):
        super(Delay, self).__init__(args, duration, world)
        self.agent = world.agent(args[0])

    def complete_action(self):
        self.completed = True
