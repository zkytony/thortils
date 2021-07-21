"""
Navigation in ai2thor

See project README; Poses in ai2thor:

 position (tuple): tuple (x, y, z); ai2thor uses (x, z) for robot base
 rotation (tuple): tuple (x, y, z); pitch, yaw, roll.
    Not doing quaternion because in ai2thor the mobile robot
    can only do two of the rotation axes so there's no problem using
    Euclidean.  Will use DEGREES. Will restrict the angles to be
    between 0 to 360 (same as ai2thor).

    yaw refers to rotation of the agent's body.
    pitch refers to rotation of the camera up and down.

 "Full pose" refers to (position, rotation), defined above
 "simplified pose" refers to (x, z, pitch, yaw)

 "action" refers to navigation action of the form (action_name, (forward, h_angle, v_angle))
 "action_delta" or "delta" refers to (forward, h_angle, v_angle)
"""

import math
from collections import deque
from .utils import (PriorityQueue, euclidean_dist,
                    to_radians, normalize_angles, roundany,
                    closest, to_degrees)
from .constants import (MOVEMENTS, MOVEMENT_PARAMS,
                        H_ANGLES, V_ANGLES,
                        GOAL_DISTANCE)
from .object import thor_object_pose, thor_closest_object_of_type
from .agent import thor_reachable_positions

def convert_movement_to_action(movement, movement_params=MOVEMENT_PARAMS):
    """movement (str), a key in the constants.MOVEMENT_PARAMS dictionary
    Returns action tuple in the format:

    ("action name", (forward, h_angle, v_angle))"""
    if movement not in movement_params:
        raise ValueError("Cannot convert movment {}."\
                         "We don't know about it.".format(movement))
    params = movement_params[movement]
    forward, h_angle, v_angle = 0.0, 0.0, 0.0
    if "moveMagnitude" in params:
        forward = params["moveMagnitude"]
    if "degrees" in params and movement.startswith("Rotate"):
        if movement == "RotateLeft":
            h_angle = -params["degrees"]
        else:
            h_angle = params["degrees"]
    if "degrees" in params and movement.startswith("Look"):
        if movement == "LookUp":
            v_angle = -params["degrees"]
        else:
            v_angle = params["degrees"]
    return (movement, (forward, h_angle, v_angle))

def get_navigation_actions(movement_params=MOVEMENT_PARAMS, exclude=set()):
    return [convert_movement_to_action(movement, movement_params)
            for movement in movement_params
            if movement not in exclude]


def _is_full_pose(robot_pose):
    return len(robot_pose) == 2\
        and len(robot_pose[0]) == 3\
        and len(robot_pose[1]) == 3

def _simplify_pose(robot_pose):
    if _is_full_pose(robot_pose):
        x, y, z = robot_pose[0]
        pitch, yaw, roll = robot_pose[1]
        return (x, z, pitch, yaw)
    return robot_pose

def _valid_pose(pose, reachable_positions):
    pose = _simplify_pose(pose)
    return pose[:2] in reachable_positions\
        and 0 <= pose[2] < 360.0\
        and 0 <= pose[3] < 360.0

def _move_by(robot_pose, action_delta,
             grid_size=None, diagonal_ok=False):
    """
    Given 2D robot pose (x, z, pitch, yaw), and an action,
    which is (forward, h_angle, v_angle); The angles are in RADIANS
    Ai2Thor uses this model, as seen by MoveAhead, MoveLeft etc. actions.
    """
    rx, rz, pitch, yaw = robot_pose
    forward, h_angle, v_angle = action_delta
    new_yaw = yaw + h_angle  # angle (degrees)
    new_rx = rx + forward*math.sin(to_radians(new_yaw))
    new_rz = rz + forward*math.cos(to_radians(new_yaw))
    if diagonal_ok:
        new_rx = roundany(new_rx, grid_size)
        new_rz = roundany(new_rz, grid_size)
    new_yaw = new_yaw % 360.0
    new_pitch = (pitch + v_angle) % 360.0
    return (new_rx, new_rz, new_yaw, new_pitch)

def transform_pose(robot_pose, action, schema="vw",
                   diagonal_ok=False, grid_size=None):
    """
    Transform pose of robot in 2D
    robot_pose (tuple): Either 2d pose (x,y,yaw,pitch)
           or a tuple (position, rotation):
               position (tuple): tuple (x, y, z)
               rotation (tuple): tuple (x, y, z); pitch, yaw, roll.
    action:
           ("ActionName", delta), where delta is the change, format dependent on schema

    Returns the transformed pose in the same form as input
    """
    x, z, pitch, yaw = _simplify_pose(robot_pose)
    action_name, delta = action
    new_pose = _move_by((x, z, pitch, yaw), delta,
                        grid_size=grid_size, diagonal_ok=diagonal_ok)
    if _is_full_pose(robot_pose):
        new_rx, new_rz, new_yaw, new_pitch = new_pose
        return (new_rx, robot_pose[0][1], new_rz),\
            (new_pitch, new_yaw, robot_pose[1][2])
    else:
        return new_pose

def _same_pose(pose1, pose2, tolerance=1e-4, angle_tolerance=5):
    """
    Returns true if pose1 and pose2 are of the same pose;
    Only cares about the coordinates that Ai2Thor cares about,
    which are x, z, pitch, yaw.

    pose1 and pose2 can either be full pose (i.e. (position, rotation)),
    or the simplified pose: (x, z, pitch, yaw)

    Args:
       tolerance (float): Euclidean distance tolerance
       angle_tolerance (float): Angular tolerance;
          Instead of relying on this tolerance, you
          should make sure the goal pose's rotation
          can be achieved exactly by taking the
          rotation actions.
    """
    if _is_full_pose(pose1):
        x1, _, z1 = pose1[0]
        pitch1, yaw1, _ = pose1[1]
    else:
        x1, z1, pitch1, yaw1 = pose1

    if _is_full_pose(pose2):
        x2, _, z2 = pose2[0]
        pitch2, yaw2, _ = pose2[1]
    else:
        x2, z2, pitch2, yaw2 = pose1

    return euclidean_dist((x1, z1), (x2, z2)) <= tolerance\
        and abs(pitch1 - pitch2) <= angle_tolerance\
        and abs(yaw1 - yaw2) <= angle_tolerance


def _nav_heuristic(pose, goal):
    """Returns underestimate of the cost from pose to goal
    pose tuple(position, rotation); goal tuple(position, rotation)"""
    return euclidean_dist(pose[0], goal[0])

def _reconstruct_plan(comefrom, end_node, return_pose=False):
    """Returns the plan from start to end_node; The dictionary `comefrom` maps from node
    to parent node and the edge (i.e. action)."""
    plan = deque([])
    node = end_node
    while node in comefrom:
        parent_node, action = comefrom[node]
        if return_pose:
            plan.appendleft({"action": action, "next_pose": _simplify_pose(node)})
        else:
            plan.appendleft(action)
        node = parent_node
    return list(plan)

def _cost(action):
    """
    action is (movement_str, (forward, h_angle, v_angle))
    """
    forward, h_angle, v_angle = action[1]
    cost = 0
    if forward != 0:
        cost += 1
    if h_angle != 0:
        cost += 1
    if v_angle != 0:
        cost += 1
    return cost

def _round_pose(full_pose):
    x, y, z = full_pose[0]
    pitch, yaw, roll = full_pose[1]
    return ((round(x, 4), round(y, 4), round(z, 4)),\
            (round(pitch, 4), round(yaw, 4), round(roll, 4)))

def find_navigation_plan(start, goal, navigation_actions,
                         reachable_positions,
                         goal_distance=0.0,
                         grid_size=None,
                         diagonal_ok=False,
                         return_pose=False,
                         debug=False):
    """Returns a navigation plan as a list of navigation actions. Uses A*

    Recap of A*: A* selects the path that minimizes

    f(n)=g(n)+h(n)

    where n is the next node on the path, g(n) is the cost of the path from the
    start node to n, and h(n) is a heuristic function that estimates the cost of
    the cheapest path from n to the goal.  If the heuristic function is
    admissible, meaning that it never overestimates the actual cost to get to
    the goal, A* is guaranteed to return a least-cost path from start to goal.

    Args:
        start (tuple): position, rotation of the start
        goal (tuple): position, rotation of the goal n
        navigation_actions (list): list of navigation actions,
            represented as ("ActionName", dpos, drot), where
            dpos is (dx,dy,dz), and drot is (dpitch, dyaw, droll).
        goal_distance (bool): acceptable minimum euclidean distance to the goal
        grid_size (float): size of the grid, typically 0.25. Only
            necessary if `diagonal_ok` is True
        diagonal_ok (bool): True if 'MoveAhead' can move
            the robot diagonally.
        return_pose (bool): True if return a list of {"action": <action>, "next_pose": <pose>} dicts
        debug (bool): If true, returns the expanded poses
    Returns:
        a list consisting of elements in `navigation_actions`
    """
    if type(reachable_positions) != set:
        reachable_positions = set(reachable_positions)

    # Map angles in start and goal to be within 0 to 360 (see top comments)
    start_rotation = normalize_angles(start[1])
    goal_rotation = normalize_angles(goal[1])
    start = (start[0], start_rotation)
    goal = (goal[0], goal_rotation)

    # The priority queue
    worklist = PriorityQueue()
    worklist.push(start, _nav_heuristic(start, goal))

    # cost[n] is the cost of the cheapest path from start to n currently known
    cost = {}
    cost[start] = 0

    # comefrom[n] is the node immediately preceding node n on the cheapeast path
    comefrom = {}

    # keep track of visited poses
    visited = set()

    if debug:
        _expanded_poses = []

    while not worklist.isEmpty():
        current_pose = worklist.pop()
        if debug:
            _expanded_poses.append(current_pose)
        if _round_pose(current_pose) in visited:
            continue
        if _same_pose(current_pose, goal,
                      tolerance=goal_distance):
            if debug:
                plan = _reconstruct_plan(comefrom,
                                         current_pose,
                                         return_pose=True)
                return plan, _expanded_poses
            else:
                return _reconstruct_plan(comefrom, current_pose,
                                         return_pose=return_pose)

        for action in navigation_actions:
            next_pose = transform_pose(current_pose, action,
                                       grid_size=grid_size,
                                       diagonal_ok=diagonal_ok)
            if not _valid_pose(_round_pose(next_pose), reachable_positions):
                continue

            new_cost = cost[current_pose] + _cost(action)
            if new_cost < cost.get(next_pose, float("inf")):
                cost[next_pose] = new_cost
                worklist.push(next_pose, cost[next_pose] + _nav_heuristic(next_pose, goal))
                comefrom[next_pose] = (current_pose, action)

        visited.add(current_pose)

    # no path found
    if debug:
        return None, _expanded_poses
    else:
        return None

def get_shortest_path_to_object(controller, object_id,
                                start_position, start_rotation,
                                **kwargs):
    """
    Per this issue: https://github.com/allenai/ai2thor/issues/825
    Ai2thor's own method to compute shortest path is not desirable.
    I'm therefore writing my own using the algorithm above.
    It has almost identical call signature as the function of the same name
    under ai2thor.util.metric.

    Returns:
       If positions_only is True, returns a list of dict(x=,y=,z=) positions
       If return_plan is True, returns a tuple of poses (or positions) and actions
    """
    # Parameters
    v_angles = kwargs.get("v_angles", V_ANGLES)
    h_angles = kwargs.get("h_angles", H_ANGLES)
    movement_params = kwargs.get("movement_params", MOVEMENT_PARAMS)
    goal_distance = kwargs.get("goal_distance", GOAL_DISTANCE)
    diagonal_ok = kwargs.get("diagonal_ok", False)
    positions_only = kwargs.get("positions_only", False)
    return_plan = kwargs.get("return_plan", False)
    as_tuples = kwargs.get("as_tuples", False)

    # reachable positions; must round them to grids otherwise ai2thor has a bug.
    grid_size = controller.initialization_parameters["gridSize"]
    reachable_positions = [
        tuple(map(lambda x: roundany(x, grid_size), pos))
        for pos in thor_reachable_positions(controller)]

    # computer goal pose
    target_position = thor_object_pose(controller,
                                       object_id, as_tuple=True)
    start_pose = (start_position, start_rotation)
    goal_pose = _goal_pose(reachable_positions,
                           target_position,
                           v_angles, h_angles,
                           y=start_position[1],
                           roll=start_rotation[2])

    navigation_actions = get_navigation_actions(movement_params)
    plan = find_navigation_plan(start_pose, goal_pose,
                                navigation_actions,
                                reachable_positions,
                                goal_distance=goal_distance,
                                grid_size=grid_size,
                                diagonal_ok=diagonal_ok,
                                return_pose=True)
    poses = []
    actions = []
    for step in plan:
        actions.append(step["action"])
        x, z, pitch, yaw = step["next_pose"]
        if positions_only:
            y = start_position[1]
            if as_tuples:
                poses.append((x,y,z))
            else:
                poses.append(dict(x=x, y=y, z=z))
        else:
            roll = start_rotation[2]
            if as_tuples:
                poses.append(((x,y,z), (pitch, yaw, roll)))
            else:
                poses.append((dict(x=x, y=start_position[1], z=z),
                              dict(x=pitch, y=yaw, z=roll)))

    if return_plan:
        return poses, actions
    else:
        return poses

def get_shortest_path_to_object_type(controller, object_type, *args, **kwargs):
    """Similar to get_shortest_path_to_object except
    taking object_type as input."""
    obj = thor_closest_object_of_type(controller, object_type)
    return get_shortest_path_to_object(controller, obj["objectId"], *args, **kwargs)


def _pitch_facing(robot_position, target_position, angles):
    """
    Returns a pitch angle rotation such that
    if the robot is at `robot_position` and target is at
    `target_position`, the robot is facing the target.

    Args:
       robot_position (tuple): x, y, z position
       target_position (tuple): x, y, z position
       angles (list): Valid pitch angles
    """
    rx, ry, _ = robot_position
    tx, ty, _ = target_position
    pitch = to_degrees(math.atan2(abs(ry - ty), abs(rx - tx)))
    return closest(angles, pitch) % 360

def _yaw_facing(robot_position, target_position, angles):
    """
    Returns a yaw angle rotation such that
    if the robot is at `robot_position` and target is at
    `target_position`, the robot is facing the target.

    Args:
       robot_position (tuple): x, y, z position
       target_position (tuple): x, y, z position
       angles (list): Valid yaw angles
    """
    angles = normalize_angles(angles)
    rx, _, rz = robot_position
    tx, _, tz = target_position
    yaw = to_degrees(math.atan2(tx - rx, tz - rz)) % 360
    return closest(angles, yaw)


def _goal_pose(reachable_positions,
               target_position,
               v_angles, h_angles,
               y=0.0, roll=0.0):
    """Compute the goal pose; the rotation should be facing the target"""
    x, z =\
        min(reachable_positions,
            key=lambda p : euclidean_dist(p, target_position))
    closest_reachable_pos = (x, y, z)
    goal_pitch = _pitch_facing(closest_reachable_pos,
                               target_position, v_angles)
    goal_yaw = _yaw_facing(closest_reachable_pos,
                           target_position, h_angles)
    #target_position
    goal_pose = (target_position, (goal_pitch, goal_yaw, roll))
    return goal_pose
