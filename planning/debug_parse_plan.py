import sys,re,rospy;

Plan = sys.stdin;
actions = [];

for line in Plan:
    actions.append(line);

waypoints = {'a': [10,20], 'b': [0, 15], 'c': [-10, 10]};

actions = actions[3:len(actions)-1];

planre = re.compile('[0-9.]+: \(([a-z0-9- ]+)\) \[[0-9]+\]');
for a in actions:
    m = planre.match(a);
    action = m.group(1);
    parts = action.split(" ");
    arguments = parts[1:len(parts)];
    type = parts[0];
    print type,arguments
