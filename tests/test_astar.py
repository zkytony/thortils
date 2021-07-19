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
    ax.scatter(x, z, s=200, c='gray')

    xs, _, zs = start[0]
    ax.scatter([xs], [zs], s=150, c='red')

    xg, _, zg = goal[0]
    ax.scatter([xg], [zg], s=150, c='green')
    ax.invert_yaxis()

def test():
    floormap1 =\
    """
    G.....
    xxx...
    .x....
    .....S
    """

    floormap2 =\
    """
    S..
    .x.
    .xG
    """
    reachable_positions, start, goal, dim = read_map(floormap2, GRID_SIZE)
    navigation_actions = get_navigation_actions(MOVEMENT_PARAMS, exclude={"LookUp", "LookDown"})

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(-GRID_SIZE, dim[0] * GRID_SIZE)
    ax.set_ylim(-GRID_SIZE, dim[1] * GRID_SIZE)
    plot_map(ax, reachable_positions, start, goal)
    plt.show()
    poses_plotted = set()

    # plan = find_navigation_plan(start, goal, navigation_actions, reachable_positions)
    # print(plan)

    for ret in find_navigation_plan(start, goal, navigation_actions,
                                             reachable_positions, debug=True):
        if type(ret) == list:
            plan = ret
            print(start)
            pprint(plan)
            print(goal)
            break

        pose, worklist = ret
        # worklist_positions = set((p[0][0], p[0][2]) for p in worklist)
        # pprint(worklist_positions)
        # print("----")

        x, _, z = pose[0]
        if (x, z) not in poses_plotted:
            ax.scatter([x], [z], s=120, c='blue')
            fig.canvas.draw()
            fig.canvas.flush_events()
            poses_plotted.add((x,z))



if __name__ == "__main__":
    test()
