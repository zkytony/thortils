import time
from thortils import (launch_controller,
                      convert_scene_to_grid_map)
from thortils.scene import SceneDataset
from thortils.utils.visual import GridMapVisualizer
from thortils.agent import thor_reachable_positions


def main():
    floor_plan = "FloorPlan1"
    scene_info = SceneDataset.load_single("../scenes", floor_plan)
    controller = launch_controller({"scene":floor_plan})
    grid_map = convert_scene_to_grid_map(controller, floor_plan, 0.25)

    print(floor_plan)
    for y in range(grid_map.length):
        row = []
        for x in range(grid_map.width):
            if (x,y) in grid_map.free_locations:
                row.append(".")
            else:
                assert (x,y) in grid_map.obstacles
                row.append("x")
        print("".join(row))

    # Highlight reachable positions
    reachable_positions = thor_reachable_positions(controller)
    highlights = []
    for thor_pos in reachable_positions:
        highlights.append(grid_map.to_grid_pos(*thor_pos))
    viz = GridMapVisualizer(grid_map=grid_map, res=30)
    img = viz.render()
    img = viz.highlight(img, highlights,
                        color=(25, 214, 224), show_progress=True)
    viz.show_img(img)
    time.sleep(10)


if __name__ == "__main__":
    main()
