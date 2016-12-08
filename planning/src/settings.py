mesh = dict(
        # minimum distance between two initial points, ie. agent/box locations
        initial_min_dist = 0.4, # (meters)

        # minimum distance between created extra waypoints
        mesh_min_dist = 0.5, # (meters)

        # minimum distance between two edges and edge to points
        # (edge is a walking path in the waypoint graph)
        edge_min_dist = 2, # (meters)

        # occupancy graph is padded to make sure turtlebot does not get stuck
        # 0.2 seems to be what rviz uses
        padding_radius = 0.2 # (meters)
)
plan = dict(
        # set to True if pathing should allow movement in unexplored regions
        walk_unknown = False,

        # turtlebot speed (m/s)
        turtle_speed = 2,

        # time before turtlebot starts moving (s)
        turtle_delay = 0.5,

        # drone speed (m/s)
        drone_speed = 4,

        # time before drone starts moving (s)
        drone_delay = 0.2,

        # planner will search for optimal result.
        # will not work anyway for big problems, so set it to 0 then.
        optimal_search = 0,

        use_auction = False,

        # search time for suboptimal search, will run if optimal search fails.
        suboptimal_search = 2*60*60
)
