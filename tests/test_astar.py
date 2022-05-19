# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

# Test if the astar algorithm works

from pprint import pprint
import matplotlib.pyplot as plt
import time
import random

from thortils.navigation import (find_navigation_plan,
                                 get_navigation_actions,
                                 plot_navigation_search_result)
from thortils.constants import MOVEMENT_PARAMS, GRID_SIZE
from thortils import (thor_reachable_positions,
                      launch_controller,
                      thor_object_with_id,
                      thor_closest_object_of_type,
                      thor_agent_pose)


def read_map(floormap, grid_size=0.25):
    reachable_positions = []
    start, goal = None, None
    width, length = None, None

    lines = floormap.split("\n")[1:-1]
    length = len(lines)
    for z, line in enumerate(lines):
        width = len(line.strip())
        for x, c in enumerate(line.strip()):
            if c in {".", "S", "G"} :
                reachable_positions.append((x*grid_size, z*grid_size))
            if c == "S":
                start = ((x*grid_size, 0, z*grid_size), (0, 90, 0))
            if c == "G":
                goal = ((x*grid_size, 0, z*grid_size), (0, 270, 0))
    return reachable_positions, start, goal, (width, length)


def test_simple():
    floormap1 =\
    """
    .S.........
    .xxx.......
    ..x......xx
    ..........G
    """

    floormap2 =\
    """
    S..
    .x.
    .xG
    """
    reachable_positions, start, goal, dim = read_map(floormap1, GRID_SIZE)
    navigation_actions = get_navigation_actions(MOVEMENT_PARAMS)

    _start_time = time.time()
    plan, expanded_poses = find_navigation_plan(start, goal,
                                                navigation_actions,
                                                reachable_positions,
                                                diagonal_ok=True,
                                                grid_size=GRID_SIZE,
                                                debug=True)
    if plan is not None:
        print("Plan found in {:.3f}s".format(time.time() - _start_time))
        pprint([step["action"] for step in plan])
        fig, ax = plt.subplots()
        plot_navigation_search_result(start, goal, plan, expanded_poses,
                                      reachable_positions, GRID_SIZE, ax=ax)
        ax.set_xlim(-GRID_SIZE, dim[0] * GRID_SIZE)
        ax.set_ylim(-GRID_SIZE, dim[1] * GRID_SIZE)
        plt.show(block=True)



def test_thor_scene():
    controller = launch_controller({"scene": "FloorPlan1"})
    reachable_positions = thor_reachable_positions(controller)
    start = thor_agent_pose(controller, as_tuple=True)
    target = thor_closest_object_of_type(controller, "Bowl")
    target_position = (target["position"]["x"],
                       target["position"]["z"])
    # gx, gz = random.sample(reachable_positions, 1)[0]
    gx, gz = target_position
    goal = (gx, 0, gz), (0, 135, 0)
    navigation_actions = get_navigation_actions(MOVEMENT_PARAMS)

    _start_time = time.time()

    plan, expanded_poses = find_navigation_plan(start, goal,
                                                navigation_actions,
                                                reachable_positions,
                                                grid_size=GRID_SIZE,
                                                diagonal_ok=False,
                                                goal_distance=1.0,
                                                debug=True)
    if plan is not None:
        print("Plan found in {:.3f}s".format(time.time() - _start_time))
        pprint([step["action"] for step in plan])
        fig, ax = plt.subplots()
        plot_navigation_search_result(start, goal, plan, expanded_poses,
                                      reachable_positions, GRID_SIZE, ax=ax)
        plt.show(block=True)

if __name__ == "__main__":
    test_simple()
