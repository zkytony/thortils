# Test if the astar algorithm works
from thortils.navigation import (find_navigation_plan,
                                 get_navigation_actions)
from thortils.constants import MOVEMENT_PARAMS, GRID_SIZE
from pprint import pprint

def read_map(floormap, grid_size=0.25):
    reachable_positions = []
    start, goal = None, None

    for z, line in enumerate(floormap.split("\n")[1:-1]):
        for x, c in enumerate(line.strip()):
            if c in {".", "S", "G"} :
                reachable_positions.append((x*grid_size, z*grid_size))
            if c == "S":
                start = ((x*grid_size, 0, z*grid_size), (0, 0, 0))
            if c == "G":
                goal = ((x*grid_size, 0, z*grid_size), (0, 0, 0))
    return reachable_positions, start, goal

def test():
    floormap1 =\
    """
    G.....
    .xxxx.
    .x....
    .....S
    """

    floormap2 =\
    """
    S.
    .G
    """
    reachable_positions, start, goal = read_map(floormap1, GRID_SIZE)
    navigation_actions = get_navigation_actions(MOVEMENT_PARAMS)
    plan = find_navigation_plan(start, goal, navigation_actions,
                                reachable_positions)
    print(start)
    pprint(plan)
    print(goal)


if __name__ == "__main__":
    test()
