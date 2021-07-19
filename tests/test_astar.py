# Test if the astar algorithm works
from thortils.navigation import (find_navigation_plan,
                                 get_navigation_actions)
from thortils.constants import MOVEMENT_PARAMS, GRID_SIZE
from pprint import pprint
import matplotlib.pyplot as plt

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
                start = ((x*grid_size, 0, z*grid_size), (0, 0, 0))
            if c == "G":
                goal = ((x*grid_size, 0, z*grid_size), (0, 0, 0))
    return reachable_positions, start, goal, (width, length)

def plot_map(ax, reachable_positions, start, goal):
    x = [p[0] for p in reachable_positions]
    z = [p[1] for p in reachable_positions]
    ax.scatter(x, z, s=300, c='gray', zorder=1)

    xs, _, zs = start[0]
    ax.scatter([xs], [zs], s=200, c='red', zorder=4)

    xg, _, zg = goal[0]
    ax.scatter([xg], [zg], s=200, c='green', zorder=4)
    ax.invert_yaxis()

def test():
    floormap1 =\
    """
    S.....
    xxx...
    .x....
    .....G
    """

    floormap2 =\
    """
    S..
    .x.
    .xG
    """
    reachable_positions, start, goal, dim = read_map(floormap1, GRID_SIZE)
    navigation_actions = get_navigation_actions(MOVEMENT_PARAMS, exclude={"LookUp", "LookDown"})

    fig, ax = plt.subplots()
    ax.set_xlim(-GRID_SIZE, dim[0] * GRID_SIZE)
    ax.set_ylim(-GRID_SIZE, dim[1] * GRID_SIZE)
    plot_map(ax, reachable_positions, start, goal)
    plan, expanded_poses = find_navigation_plan(start, goal,
                                                navigation_actions,
                                                reachable_positions,
                                                debug=True)
    x = [p[0][0] for p in expanded_poses]
    z = [p[0][2] for p in expanded_poses]
    c = [i for i in range(0, len(expanded_poses))]
    ax.scatter(x, z, s=120, c=c, zorder=2, cmap="bone")

    for step in plan:
        x, z, _, _ = step["next_pose"]
        ax.scatter([x], [z], s=120, zorder=2, c="orange")

    plt.show()


if __name__ == "__main__":
    test()
